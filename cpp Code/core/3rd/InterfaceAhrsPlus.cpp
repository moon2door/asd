#include "InterfaceAhrsPlus.h"


namespace SHI
{
	namespace Device
	{
		namespace MyAhrsPlus
		{
			CInterfaceAhrsPlus::CInterfaceAhrsPlus(std::string port /*= ""*/, unsigned int baudrate /*= 115200*/)
				: WithRobot::iMyAhrsPlus(port, baudrate), m_pObserver(NULL)
			{

			}

			CInterfaceAhrsPlus::~CInterfaceAhrsPlus()
			{

			}

			void CInterfaceAhrsPlus::SetObserver(CObserverAhrsPlus* pObserver)
			{
				m_pObserver = pObserver;
			}

			bool CInterfaceAhrsPlus::Initialize()
			{
				bool ok = false;
				do {
					if (start() == false) break;
					if (cmd_binary_data_format("QUATERNION, IMU") == false) break;
					if (cmd_divider(DIVIDER) == false) break;
					if (cmd_mode("BC") == false) break;
					ok = true;
				} while (0);

				return ok;
			}

			WithRobot::SensorData CInterfaceAhrsPlus::GetData()
			{
				WithRobot::LockGuard _l(m_lock);
				return m_data;
			}

			void CInterfaceAhrsPlus::OnSensorData(int sensor_id, WithRobot::SensorData data)
			{
				{
					WithRobot::LockGuard _l(m_lock);
					m_data = data;
					m_data.euler_angle = m_data.quaternion.to_euler_angle();
				}

				if (m_pObserver) m_pObserver->OnSensorData(sensor_id, m_data);
			}

			void CInterfaceAhrsPlus::OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
			{
				if (m_pObserver) m_pObserver->OnAttributeChange(sensor_id, attribute_name, value);
			}
		}
	}
}

