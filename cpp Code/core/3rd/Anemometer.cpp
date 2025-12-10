#include "Anemometer.h"
#include <windows.h>

namespace Device
{
	namespace Anemometer
	{
		typedef struct _StAnemometerData
		{
			union
			{
				struct
				{
					unsigned char id;				// 0x01
					unsigned char length;			// 0x06
					unsigned char windDirection;	// 0(0deg) ~ 127(360deg)
					unsigned char temperature;		// unused
					unsigned short windSpeed;		// big endian, Ω«¡¶ «≥º”¿∫ /40 [m/sec]
					unsigned char checksum;
				} data;
				unsigned char byte[8];
			};

			_StAnemometerData()
			{
				data.id = 0;
				data.length = 0;
				data.windDirection = 0;
				data.temperature = 0;
				data.windSpeed = 0;
				data.checksum = 0;
			}

			float windSpeed()
			{
				float ret = static_cast<float>(htons(data.windSpeed)) / 40.0f;
				return ret;
			}

			float windDirection()
			{
				return static_cast<float>(data.windDirection) * 360.0f / 127.0f;
			}

			unsigned char CalcChecksum()
			{
				unsigned char sum = 0x33;
				for (unsigned int i=0; i<6; i++)
				{
					sum ^= byte[i];
				}
				return sum;
			}
		} StAnemometerData;

		CInterfaceAnemometer::CInterfaceAnemometer(CObserverAnemometer* pObserver)
			: m_pObserver(pObserver)
		{
		}

		CInterfaceAnemometer::~CInterfaceAnemometer()
		{
		}

		bool CInterfaceAnemometer::Start(char * port)
		{
			DevLib::CThread::ServiceStart();
			return m_serial.OpenPort(port, 2400);
		}

		void CInterfaceAnemometer::Stop()
		{
			DevLib::CThread::ServiceStop();
			m_serial.ClosePort();
		}

		void CInterfaceAnemometer::SetObserver(CObserverAnemometer* pObserver)
		{
			m_pObserver = pObserver;
		}

		void CInterfaceAnemometer::Run()
		{
			StAnemometerData buffer;
			while (DevLib::CThread::IsRun())
			{
				for(unsigned int retry=10; retry>0; retry--)
				{
					m_serial.Recv(&buffer.byte[0], 1);
					if (buffer.byte[0] != 0x01) continue;

					m_serial.Recv(&buffer.byte[1], 1);
					if (buffer.byte[1] != 0x06) continue;

					m_serial.Recv(&buffer.byte[2], 5);
					if (buffer.CalcChecksum() == buffer.data.checksum)
					{
						if(m_pObserver) m_pObserver->OnDataAnemometer(buffer.windSpeed(), buffer.windDirection());
						break;
					}
				}				
				Sleep(10);
			}
		}

	}
}