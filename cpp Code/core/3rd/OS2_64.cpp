#include <cstdio>
#include <cstdarg>
#include <string>
#include <vector>
#include <rapidjson/document.h>
#include "OS2_64.h"

namespace Device
{
	namespace OS2_64
	{
		COS2_64::COS2_64()
		{
			RegisterCallbackRecieveImu(&COS2_64::OnRecieveImu, this);
			RegisterCallbackRecieveLidar(&COS2_64::OnRecieveLidar, this);
			RegisterCallbackRecieveStatus(&COS2_64::OnRecieveStatus, this);
		}

		bool COS2_64::Create(uint16_t portCmd, uint16_t portLidar, uint16_t portImu, const std::string& LidarIp, const std::string& bindIp)
		{
			bool ret = false;
			if (m_udpLidar.Create(Routine::IO::SOCK_TYPE_UDP, sizeof(Device::OS2_64::StLidarData)))
			{
				m_udpLidar.RegisterCallbackReceivedData(&COS2_64::OnReceivedLidarPacket, this);
				m_udpLidar.GetSocket().Bind(portLidar, bindIp);
				m_udpLidar.StartWatchDog();

				m_udpImu.Create(Routine::IO::SOCK_TYPE_UDP, sizeof(Device::OS2_64::StIMUData));
				m_udpImu.RegisterCallbackReceivedData(&COS2_64::OnReceivedImuPacket, this);
				m_udpImu.GetSocket().Bind(portImu, bindIp);
				m_udpImu.StartWatchDog();

				m_clientCommand.RegisterCallbackReceiveData(&COS2_64::OnReceiveTcpApi, this);
				m_clientCommand.CreateTcpClient(LidarIp, portCmd);

				m_timerCommand.StartTimer(10000, &COS2_64::OnTimerQuerySensorInfo, this);
				ret = true;
			}
			return ret;
		}

		void COS2_64::Destroy()
		{
			m_clientCommand.Destroy();
			m_timerCommand.StopTimer();
		}

		bool COS2_64::SendCommand(const char* command, ...)
		{
			char cmd[512] = { 0, };
			va_list ap;
			va_start(ap, command);
			vsprintf_s(cmd, command, ap);  // NOLINT(cert-err33-c)
			va_end(ap);

			const size_t length = strlen(cmd);
			cmd[length] = ' ';
			cmd[length + 1] = '\n';
			cmd[length + 2] = 0;

			return m_clientCommand.Send(cmd, strlen(cmd)) > 0;
		}

		std::vector<std::string> SeparateMessage(const std::string& message, char separator)
		{
			std::vector<std::string> ret;
			size_t cur = 0, find = 0;
			while (find != std::string::npos)
			{
				find = message.find(separator, cur);
				ret.push_back(message.substr(cur, find - cur));
				cur = find + 1;
			}
			return ret;
		}

		void COS2_64::OnTimerQuerySensorInfo()
		{
			SendCommand("get_sensor_info");
		}

		// ReSharper disable once CppMemberFunctionMayBeConst
		void COS2_64::OnReceivedImuPacket(Routine::IO::CSocket& sock, Routine::void_ptr pData, int32_t dataSize)
		{
			if (dataSize == sizeof(Device::OS2_64::StIMUData))
			{
				m_callbackRecieveImu(static_cast<Device::OS2_64::StIMUData*>(pData));
			}
		}

		// ReSharper disable once CppMemberFunctionMayBeConst
		void COS2_64::OnReceivedLidarPacket(Routine::IO::CSocket& sock, Routine::void_ptr pData, int32_t dataSize)
		{
			if (dataSize == sizeof(Device::OS2_64::StLidarData))
			{
				const auto data = static_cast<Device::OS2_64::StLidarData*>(pData);
				m_callbackRecieveLidar(data);
			}
		}

		// ReSharper disable once CppMemberFunctionMayBeConst
		void COS2_64::OnReceiveTcpApi(const Routine::IO::CSocket& sock, const std::string& ip, uint16_t port, Routine::void_ptr pData, int32_t size)
		{
			// Parse server data
			const char* message = static_cast<char*>(pData);
			const std::vector<std::string> messages = SeparateMessage(message, '\n');

			for (const auto& msg : messages)
			{
				// Update sensor info
				rapidjson::Document document;
				if (!document.Parse(msg.c_str()).HasParseError())
				{
					if (document.HasMember("status"))
					{
						std::string str = document["status"].GetString();
						m_callbackRecieveStatus(std::ref(str));
					}
				}
			}
		}

	}
}
