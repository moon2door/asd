#include "OS2_32.h"
#include <rapidjson/document.h>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>
#include <string>
#include <vector>


namespace Device
{
	namespace OS2_32
	{
		COS2_32::COS2_32(CObserverOS2_32* pObserver)
			: m_pObserver(pObserver)
		{
			m_udpLidar = new DevLib::IO::Socket::CReceiverObject(this);
			m_udpImu = new DevLib::IO::Socket::CReceiverObject(this);
			m_clientCommand = new DevLib::IO::Socket::CClientObject(this);

			// Calc rate
			countFrameRate = 0;

			// Calc Rpm
			prevAngle = 0;
			accumAngle = 0;
			startTime = 0;
			countRPM = 0;

			// Calc max distance
			max = 0;
			countMaxDistance = 0;
		}
		
		COS2_32::~COS2_32()
		{
			delete m_udpLidar;
			delete m_udpImu;
			delete m_clientCommand;

			if (m_timerCommand.IsTimerOn())
			{
				Destroy();
			}
		}
		
		void COS2_32::SetObserver(CObserverOS2_32* pObserver)
		{
			m_pObserver = pObserver;
		}

		bool COS2_32::Create(int portCmd, int portLidar, int portImu, char* addr, char* bindAddr)
		{
			bool ret = false;
			ret = m_udpLidar->Create(SOCK_DGRAM);
			m_udpLidar->SetPacketSize(6464);
			ret = m_udpLidar->Bind(portLidar, bindAddr);
			ret = m_udpLidar->ServiceStart();

			ret = m_udpImu->Create(SOCK_DGRAM);
			ret = m_udpImu->Bind(portImu, bindAddr);
			ret = m_udpImu->ServiceStart();

			ret = m_clientCommand->ServiceStart();
			ret = m_clientCommand->Create(addr, portCmd);
			//ret = m_clientCommand->Connect();
			ret = m_clientCommand->AutoConnectStart();

			m_timerCommand.StartTimer(10000, COS2_32::_OnTimer, this, 5000);
			return ret;
		}

		void COS2_32::Destroy()
		{
			m_udpLidar->Destroy();
			m_udpImu->Destroy();
			m_clientCommand->Destroy();
			m_timerCommand.StopTimer();
		}
		
		bool COS2_32::Command(const char* command, ...)
		{
			char cmd[512] = { 0, };
			va_list ap;
			va_start(ap, command);
			vsprintf_s(cmd, command, ap);
			va_end(ap);

			unsigned int length = strlen(cmd);
			cmd[length] = ' ';
			cmd[length + 1] = '\n';
			cmd[length + 2] = 0;
			return m_clientCommand->Send(cmd, strlen(cmd)) > 0;
		}

		void COS2_32::OnReceiverData(char* ip, int port, void* data, int size)
		{
			if (size == sizeof(Device::OS2_32::StLidarData))
			{
				OnReceiverLidar(static_cast<Device::OS2_32::StLidarData*>(data));
			}
			else if (size == sizeof(Device::OS2_32::StIMUData))
			{
				OnReceiverImu(static_cast<Device::OS2_32::StIMUData*>(data));
			}
			else
			{
			}
		}
		void COS2_32::OnReceiverImu(Device::OS2_32::StIMUData* data)
		{
			if (m_pObserver) m_pObserver->OnReceiverImu(data);
		}

		const float div90112 = 1.0f / 90112;

		void COS2_32::OnReceiverLidar(Device::OS2_32::StLidarData* data)
		{
			// Update frame rate
			if (++countFrameRate >= 1000)
			{
				double e = elapsedFps.GetElapseTime() * 0.000001;
				double fps = 1000.0 / e;
				countFrameRate = 0;

				UpdateFrameRate(fps);
			}

			// Update rpm
			double rpm2 = 0;
			double time = elapsedRpm.GetElapseTimeContinue() * 0.000001;
			double angle = 360.f*data->azimuthBlock[0].encoderCount*div90112;
			unsigned int encoderCount = data->azimuthBlock[0].encoderCount;
			
			double dangle = angle - prevAngle;
			if (dangle < 0) dangle += 360.0;
			accumAngle += dangle;

			if (++countRPM >= 1000)
			{
				rpm2 = 60 * accumAngle / ((time - startTime)*360.0);
				accumAngle = dangle;
				startTime = time;
				countRPM = 0;

				UpdateRpm(rpm2);
			}
			prevAngle = angle;

			// Update timestamp
			UpdateTimestamp(time);

			// Update max distance
			for (unsigned int i = 0; i < 32; i++)
			{
				for (unsigned int layer = 0; layer < 32; layer++)
				{
					if (data->azimuthBlock[i].dataBlock[layer].range * 0.001 < 250.0)
					{
						max = (std::max)(data->azimuthBlock[i].dataBlock[layer].range, max);
					}
				}
			}

			if (++countMaxDistance >= 2560)
			{
				UpdateMaxDistance(max * 0.001);
				countMaxDistance = 0;
				max = 0;
			}

			// Callback
			if (m_pObserver) m_pObserver->OnReceiverLidar(data);
		}

		std::vector<std::string> SepetateMessage(std::string message, char seperator)
		{
			std::vector<std::string> ret;
			size_t cur = 0, find = 0;
			while (find != std::string::npos)
			{
				find = message.find(seperator, cur);
				ret.push_back(message.substr(cur, find - cur));
				cur = find + 1;
			}
			return ret;
		}

		void COS2_32::OnServerData(char* ip, int port, void* data, int size)
		{
			// Parse server data
			char* message = static_cast<char*>(data);
			std::vector<std::string> messages = SepetateMessage(message, '\n');

			for (unsigned int i=0; i<messages.size(); i++)
			{
				// Update sensor info
				rapidjson::Document document;
				if (!document.Parse(messages[i].c_str()).HasParseError())
				{
					if (document.HasMember("status"))
					{
						std::string str = document["status"].GetString();
						UpdateSensorInfo(str);
					}
					if (document.HasMember("beam_altitude_angles"))
					{
						std::vector<float> altitudeAngles;
						unsigned int size = document["beam_altitude_angles"].GetArray().Size();

						for (unsigned int i = 0; i < size; i++)
						{
							altitudeAngles.push_back(document["beam_altitude_angles"].GetArray()[i].GetFloat());
						}
						UpdateAltitudeAngles(altitudeAngles);
					}
					if (document.HasMember("beam_azimuth_angles"))
					{
						std::vector<float> azimuthAngles;
						unsigned int size = document["beam_azimuth_angles"].GetArray().Size();

						for (unsigned int i = 0; i < size; i++)
						{
							azimuthAngles.push_back(document["beam_azimuth_angles"].GetArray()[i].GetFloat());
						}
						UpdateAzimuthAngles(azimuthAngles);
					}
				}
			}

			// Callback
			if (m_pObserver) m_pObserver->OnServerData(ip, port, data, size);
		}

		void COS2_32::OnConnected(char *ip, int port)
		{
			if (m_pObserver) m_pObserver->OnConnected(ip, port);
		}

		void COS2_32::OnDisConnected(char *ip, int port)
		{
			if (m_pObserver) m_pObserver->OnDisConnected(ip, port);
		}


		void COS2_32::OnTimer()
		{
			Command("get_sensor_info");
		}

	}
}