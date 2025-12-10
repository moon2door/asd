#include <Windows.h>
#include "Pala720dsp.h"

namespace Device
{
	namespace Pala720dsp
	{
		struct StCommand
		{
			uint8_t head[2]{};
			uint8_t id;
			uint8_t length;
			uint8_t data[256]{};

			explicit StCommand(uint8_t _id = 0, uint8_t _length = 0)
				: id(_id), length(_length)
			{
				head[0] = 'F'; head[1] = 'M';
				memset(data, 0, sizeof(data));
			}

			[[nodiscard]] int32_t Size() const { return (4 + length + 1); }
			void UpdateCRC()
			{
				uint8_t crc = head[0] ^ head[1] ^ id ^ length;
				for (int32_t i = 0; i < length; i++)
				{
					crc = crc ^ data[i];
				}
				data[length] = crc;
			}
		};

		struct StStatus
		{
			uint16_t encoderPulse;
			uint16_t harmonicDrive;
			uint16_t absResolution;
			uint16_t absoluteHome;
		};

		struct StRotorDataRaw
		{
			uint8_t hour;
			uint8_t	minute;
			uint16_t msec;
			uint16_t yaw_enc;
			uint16_t yaw_rpm;
		};

		CStatusPala720dsp::~CStatusPala720dsp()
		= default;

		bool CPala720dsp::CreatePala720Dsp(uint8_t id, uint16_t portDsp, uint16_t portDspCmd, const std::string& addrDsp, const std::string& addrBind)
		{
			bool ret = false;
			m_id = id;
			if(m_udpDspCmd.Create(Routine::IO::SOCK_TYPE_UDP))
			{
				m_udpDspCmd.RegisterCallbackReceivedData(&CPala720dsp::OnReceivedPala720DspMessage, this);
				m_udpDspCmd.GetSocket().Bind(portDspCmd, addrBind);
				m_udpDspCmd.StartWatchDog();
				m_addrDsp = addrDsp;
				m_portDspCmd = portDspCmd;
				
				ret = m_udpDsp.Create(Routine::IO::SOCK_TYPE_UDP);
				m_udpDsp.RegisterCallbackReceivedData(&CPala720dsp::OnReceivedPala720Dsp, this);
				m_udpDsp.GetSocket().Bind(portDsp, addrBind);
				m_udpDsp.StartWatchDog();
				
				m_timerSendAlive.StartTimer(1000, &CPala720dsp::CommandAlive, this);
				
				CommandReqInitialStat();
			}			
			return ret;
		}

		void CPala720dsp::Destroy()
		{
			m_timerSendAlive.StopTimer();
			m_udpDspCmd->Destroy();
			m_udpDsp->Destroy();
		}
		
		bool CPala720dsp::CommandModeChange()
		{
			constexpr uint8_t data[] = { 'C', 'H', 'A', 0xDF, 1, 1 };
			StCommand message(0x11, sizeof(data));
			memcpy(message.data, data, sizeof(data));
			message.UpdateCRC();

			return m_udpDspCmd->SendTo(&message, message.Size(), m_addrDsp, m_portDspCmd) > 0;
		}

		bool CPala720dsp::CommandStop()
		{
			constexpr int8_t data[] = "Stop";
			StCommand message(0xF0, sizeof(data));
			memcpy(message.data, data, sizeof(data));
			message.UpdateCRC();

			return m_udpDspCmd->SendTo(&message, message.Size(), m_addrDsp, m_portDspCmd) > 0;
		}

		bool CPala720dsp::CommandReqInitialStat()
		{
			constexpr int8_t data[] = "ZERO";
			StCommand message(0xF0, sizeof(data));
			memcpy(message.data, data, sizeof(data));
			message.UpdateCRC();

			return m_udpDspCmd->SendTo(&message, message.Size(), m_addrDsp, m_portDspCmd) > 0;
		}

		bool CPala720dsp::CommandRpmSet(float rpm)
		{
			const uint16_t _rpm = htons(static_cast<uint16_t>(rpm * 100));
			StCommand message(0x49, sizeof(_rpm));
			memcpy(message.data, &_rpm, sizeof(_rpm));
			message.UpdateCRC();

			return m_udpDspCmd->SendTo(&message, message.Size(), m_addrDsp, m_portDspCmd) > 0;
		}

		bool CPala720dsp::CommandAlive()
		{
			constexpr int8_t data[] = "Alive";
			StCommand message(0x4A, sizeof(data));
			memcpy(message.data, data, sizeof(data));
			message.UpdateCRC();

			return m_udpDspCmd->SendTo(&message, message.Size(), m_addrDsp, m_portDspCmd) > 0;
		}

		bool CPala720dsp::Reconnect(uint8_t id, uint16_t portDsp, uint16_t portDspCmd, const std::string& addrDsp, const std::string& addrBind)
		{
			Destroy();
			countFrameRate = 0;
			prevAngle = 0;
			accumAngle = 0;
			startTime = 0;
			countRPM = 0;
			bInitRpm = true;
			return CreatePala720Dsp(id, portDsp, portDspCmd, addrDsp, addrBind);
		}

		void CPala720dsp::OnReceivedPala720DspMessage(Routine::IO::CSocket& sock, Routine::void_ptr pData, int32_t dataSize)
		{
			StCommand message;
			memcpy(&message, pData, dataSize);
			const auto pStatus = reinterpret_cast<StStatus*>(message.data);

			switch (message.id)
			{
			case 0x37:
				OnAckMode();
				break;
			case 0x32:
				OnAckCommand();
				break;
			case 0x34:
				OnAckZero(
					htons(pStatus->absoluteHome),
					htons(pStatus->absResolution),
					htons(pStatus->encoderPulse),
					htons(pStatus->harmonicDrive));
				break;
			case 0x3A:
				OnAckRpm();
				break;
			case 0x39:
				OnAckBit();
				break;
			default:
				break;
			}
		}

		void CPala720dsp::OnReceivedPala720Dsp(Routine::IO::CSocket& sock, Routine::void_ptr pData, int32_t dataSize)
		{
			if (bInitRpm)
			{
				CommandRpmSet(36.0);
				bInitRpm = false;
			}
			const auto data = static_cast<StRotorDataRaw*>(pData);

			// parse packet
			const auto minute = static_cast<double>(data->minute);
			const auto msec = static_cast<double>(htons(data->msec));
			const double time = minute * 60.0 + msec * 0.001;
			//auto rpm = htons(data->yaw_rpm) * 0.01; // rpm이 올라오나, 부정확하여 rpm2로 직접 계산함
			const auto angle = htons(data->yaw_enc) * m_yawConvValue;

			// Calculate Frame Rate
			if (++countFrameRate >= 1000)
			{
				const double e = elapsed.GetElapseTime() * 0.000001;
				const double fps = 1000.0 / e;
				countFrameRate = 0;

				UpdateFrameRate(fps);
			}

			// Calculate RPM
			double rpm2 = 0;

			double dangle = angle - prevAngle;

			if (dangle < 0) dangle += 360.0;
			accumAngle += dangle;

			if (++countRPM >= 50)
			{
				rpm2 = 60 * accumAngle / ((time - startTime) * 360.0);
				accumAngle = dangle;
				startTime = time;
				countRPM = 0;

				UpdateRpm(rpm2);
			}
			prevAngle = angle;

			// Update timestamp
			UpdateTimestamp(time);

			const StRotorData rotorData(time, rpm2, angle);
			m_callbackRotorData(m_id, &rotorData);
		}

		void CPala720dsp::OnAckMode()
		{
		}

		void CPala720dsp::OnAckCommand()
		{
		}

		void CPala720dsp::OnAckZero(uint16_t absoluteHome, uint16_t absResolution, uint16_t encoderPulse, uint16_t harmonicDrive)
		{
			m_yawConvValue = 1.0 / ((static_cast<double_t>(harmonicDrive) * static_cast<double_t>(encoderPulse)) / 360.0);
		}

		void CPala720dsp::OnAckRpm()
		{
		}

		void CPala720dsp::OnAckBit()
		{
		}
		

	}
}