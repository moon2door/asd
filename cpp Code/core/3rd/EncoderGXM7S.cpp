#include "EncoderGXM7S.h"
#include <WinSock2.h>
#pragma comment(lib,"ws2_32")

namespace Device
{
	namespace EncoderGXM7S
	{
		typedef struct
		{
			union
			{
				struct
				{
					unsigned char start;
					unsigned char soh;
					unsigned short revolution;
					unsigned short step;
					unsigned char checksum;
					unsigned char end;
				} data;
				unsigned char byte[8];
				unsigned long long val;
			};
		} StEncoderData;

		CInterfaceEncoderGXM7S::CInterfaceEncoderGXM7S(CObserverEncoderGXM7S* pObserver)
			: m_pObserver(pObserver)
		{
		}

		CInterfaceEncoderGXM7S::~CInterfaceEncoderGXM7S()
		{
		}

		bool CInterfaceEncoderGXM7S::Start(char * port)
		{
			m_thread.StartThread(&CInterfaceEncoderGXM7S::Run, this);
			return m_serial.OpenPort(port, 38400);
		}

		void CInterfaceEncoderGXM7S::Stop()
		{
			m_thread.StopThread();
			m_serial.ClosePort();
		}

		void CInterfaceEncoderGXM7S::SetObserver(CObserverEncoderGXM7S* pObserver)
		{
			m_pObserver = pObserver;
		}

		void CInterfaceEncoderGXM7S::Run()
		{
			StEncoderData buffer;
			float angle = 0;
			while (m_thread.IsRunThread())
			{
				unsigned char cmd[] = { 0x01, 0x80, 0x02, 0x80, 0x04 };
				m_serial.Send(cmd, sizeof(cmd));
				if (m_serial.Recv(buffer.byte, 8) > 0)
				{
					// start, end bit check
					if (buffer.data.start != 0x01 || buffer.data.end != 0x04) continue;

					// CRC check
					unsigned char check = (buffer.byte[1] ^ buffer.byte[2] ^ buffer.byte[3] ^ buffer.byte[4] ^ buffer.byte[5]);
					if (check != buffer.data.checksum) continue;

					// endian convert
					buffer.data.revolution = htons(buffer.data.revolution);
					buffer.data.step = htons(buffer.data.step);
					
					// return
					angle = 360.0 * buffer.data.step / m_maxStep;
					OnDataEncoderGXM7S(buffer.data.revolution, angle);
				}
				Sleep(10);
			}
		}

		void CInterfaceEncoderGXM7S::OnDataEncoderGXM7S(unsigned int count, float angle)
		{
			if (m_pObserver)
			{
				m_pObserver->OnDataEncoderGXM7S(count, angle);
			}
		}

	}
}