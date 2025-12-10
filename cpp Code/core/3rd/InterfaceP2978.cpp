#include "InterfaceP2978.h"

namespace Device
{
	namespace P2978
	{
		CInterfaceP2978::CInterfaceP2978(CObserverP2978* pObserver)
			: m_pObserver(pObserver)
		{
		}


		CInterfaceP2978::~CInterfaceP2978()
		{
		}

		bool CInterfaceP2978::Open(char* port)
		{
			return DevLib::IO::Serial::CCommSerial::OpenPort(port, 9600);
		}

		void CInterfaceP2978::Close()
		{
			DevLib::IO::Serial::CCommSerial::ClosePort();
		}

		void CInterfaceP2978::SetObserver(CObserverP2978* pObserver)
		{
			m_pObserver = pObserver;
		}

		bool CInterfaceP2978::SendCmdSingleOutput()
		{
			char cmd[] = { 0xAA, 0x30 };
			int nSend = DevLib::IO::Serial::CCommSerial::Send(cmd, sizeof(cmd));
			return  (nSend == 2);
		}

		bool CInterfaceP2978::SendCmdRelativeAngle()
		{
			char cmd[] = { 0xAA, 0x31 };
			int nSend = DevLib::IO::Serial::CCommSerial::Send(cmd, sizeof(cmd));
			return  (nSend == 2);
		}

		bool CInterfaceP2978::SendCmdAbsoluteAngle()
		{
			char cmd[] = { 0xAA, 0x32 };
			int nSend = DevLib::IO::Serial::CCommSerial::Send(cmd, sizeof(cmd));
			return  (nSend == 2);
		}

		bool CInterfaceP2978::SendCalibrationStart()
		{
			char cmd[] = { 0xAA, 0x33 };
			int nSend = DevLib::IO::Serial::CCommSerial::Send(cmd, sizeof(cmd));
			return  (nSend == 2);
		}

		bool CInterfaceP2978::SendCalibrationEnd()
		{
			char cmd[] = { 0xAA, 0x35 };
			int nSend = DevLib::IO::Serial::CCommSerial::Send(cmd, sizeof(cmd));
			return  (nSend == 2);
		}

		void CInterfaceP2978::OnDataP2978(float angle)
		{
			if (m_pObserver)
			{
				m_pObserver->OnDataP2978(angle);
			}
		}

		void CInterfaceP2978::OnReceive()
		{
			char buffer[1024] = "";
			while (GetQueueSize() > 0)
			{
				int nRead = DevLib::IO::Serial::CCommSerial::Recv(buffer, 8);
				printf("read %d bytes : \n", nRead);
				for (unsigned int i = 0; i < nRead; i++)
				{
					printf(" 0x%X", buffer[i]);
				}
				printf("remained size = %d\n", GetQueueSize());
			}
		}


	}
}