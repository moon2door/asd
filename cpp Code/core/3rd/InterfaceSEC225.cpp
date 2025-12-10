#include <windows.h>
#include "InterfaceSEC225.h"

namespace Device
{
	namespace SEC225
	{
		int bcd2dec16(unsigned short hex)
		{
			int dec = ((hex & 0x0F00) >> 8) * 100 + ((hex & 0x00F0) >> 4) * 10 + (hex & 0x000F);
			if ((hex & 0xF000) >> 12 == 1) dec = -dec;
			return dec;
		}

		unsigned short dec2bcd16(int dec)
		{
			unsigned short bcd = 0;
			if (dec < 0)
			{
				bcd = 0x1000;
				bcd |= (-dec % 10);
				bcd |= ((-dec % 100) / 10) << 4;
				bcd |= ((-dec % 1000) / 100) << 8;
			}
			else
			{
				bcd |= (dec % 10);
				bcd |= ((dec % 100) / 10) << 4;
				bcd |= ((dec % 1000) / 100) << 8;
			}
			return bcd;
		}

		int bcd2dec24(unsigned int hex)
		{
			int dec =
				((hex & 0x0F0000) >> 16) * 10000 +
				((hex & 0x00F000) >> 12) * 1000 +
				((hex & 0x000F00) >> 8) * 100 +
				((hex & 0x0000F0) >> 4) * 10 +
				(hex & 0x00000F);
			if ((hex & 0xF00000) >> 20 == 1) dec = -dec;
			return dec;
		}

		unsigned int dec2bcd24(int dec)
		{
			unsigned int bcd = 0;
			if (dec < 0)
			{
				bcd = 0x100000;
				bcd |= (-dec % 10);
				bcd |= ((-dec % 100) / 10) << 4;
				bcd |= ((-dec % 1000) / 100) << 8;
				bcd |= ((-dec % 10000) / 1000) << 12;
				bcd |= ((-dec % 100000) / 10000) << 16;
			}
			else
			{
				bcd |= (dec % 10);
				bcd |= ((dec % 100) / 10) << 4;
				bcd |= ((dec % 1000) / 100) << 8;
				bcd |= ((dec % 10000) / 1000) << 12;
				bcd |= ((dec % 100000) / 10000) << 16;
			}
			return bcd;
		}

		bool CInterfaceSEC225::SendMessage(std::vector<unsigned char>& msg)
		{
			// checksum
			unsigned char sum = 0;
			for (unsigned int i = 1; i < msg.size(); i++) sum += msg[i];
			msg.push_back(sum);

			// send
			int nSend = DevLib::IO::Serial::CCommSerial::Send(msg.data(), msg.size());
			return  (nSend == msg.size());
		}

		CInterfaceSEC225::CInterfaceSEC225(CObserverSEC225* pObserver)
			: m_pObserver(pObserver)
		{
		}


		CInterfaceSEC225::~CInterfaceSEC225()
		{
		}

		bool CInterfaceSEC225::Open(char* port)
		{
			bool ret = DevLib::IO::Serial::CCommSerial::OpenPort(port, 9600);
			if(ret) ServiceStart();
			return ret;
		}

		bool CInterfaceSEC225::Start()
		{
			return m_threadRead.ServiceStart(_ReadThread, this);
		}

		void CInterfaceSEC225::Close()
		{
			DevLib::IO::Serial::CCommSerial::ClosePort();
		}

		void CInterfaceSEC225::SetObserver(CObserverSEC225* pObserver)
		{
			m_pObserver = pObserver;
		}

		bool CInterfaceSEC225::ReadDirection()
		{
			//std::vector<unsigned char> msg = { 0x77, 0x04, 0x00, 0x01 };
			std::vector<unsigned char> msg = { 0x77, 0x04, 0x00, 0x03 };
			return SendMessage(msg);
		}
		
		bool CInterfaceSEC225::SetBaudRate(BaudRate baudrate)
		{
			std::vector<unsigned char> msg = { 0x77, 0x05, 0x00, 0x0B };
			msg.push_back((unsigned char)baudrate);
			return SendMessage(msg);
		}

		bool CInterfaceSEC225::SetMagneticDeclination(float declination)
		{
			int dec = int(declination * 100);
			unsigned short code = dec2bcd16(dec);

			std::vector<unsigned char> msg = { 0x77, 0x06, 0x00, 0x06 };
			msg.push_back(unsigned char(code));
			msg.push_back(unsigned char(code >> 8));
			return SendMessage(msg);
		}

		bool CInterfaceSEC225::ReadMagneticDevlination()
		{
			std::vector<unsigned char> msg = { 0x77, 0x04, 0x00, 0x07 };
			return SendMessage(msg);
		}

		bool CInterfaceSEC225::SetAddress(unsigned char address)
		{
			std::vector<unsigned char> msg = { 0x77, 0x05, 0x00, 0x0F };
			msg.push_back(address);
			return SendMessage(msg);
		}

		bool CInterfaceSEC225::QueryAddress()
		{
			std::vector<unsigned char> msg = { 0x77, 0x04, 0x00, 0x1F };
			return SendMessage(msg);
		}

		bool CInterfaceSEC225::SetOutputMode(OutputMode mode)
		{
			std::vector<unsigned char> msg = { 0x77, 0x05, 0x00, 0x0C };
			msg.push_back((unsigned char)mode);
			return SendMessage(msg);
		}

		bool CInterfaceSEC225::SaveSetting()
		{
			std::vector<unsigned char> msg = { 0x77, 0x04, 0x00, 0x0A };
			return SendMessage(msg);
		}

		void CInterfaceSEC225::OnReceive()
		{
			unsigned char buffer[256] = "";
			while (GetQueueSize() > 0)
			{
				// find header
				int nRead = 0;
				do
				{
					nRead = DevLib::IO::Serial::CCommSerial::Recv(buffer, 1);
					if (nRead != 1) break;
				} while (buffer[0] != 0x77);

				// read remained message
				if (nRead == 1)
				{
					// length
					DevLib::IO::Serial::CCommSerial::Recv(&buffer[1], 1);
					unsigned int size = buffer[1];
					if (size >= 256) break;

					// data
					DevLib::IO::Serial::CCommSerial::Recv(&buffer[2], size);

					// checksum
					unsigned char checksum = 0;
					for (unsigned int i=1; i<size; i++)
					{
						checksum += buffer[i];
					}
					if (buffer[size] != checksum)
					{
						printf("checksum error\n");
						printf("checksum %x, %x\n", buffer[size], checksum);
						break;
					}

					// parse data
					unsigned int bcd = 0;
					int dec = 0;

					switch (buffer[3])
					{
					case 0x83:
						bcd = buffer[4] << 16 | buffer[5] << 8 | buffer[6];
						dec = bcd2dec24(bcd);
						OnResponseReadDirection(CorrectAngle(dec * 0.01));
						break;
					case 0x81:
						bcd = buffer[4] << 16 | buffer[5] << 8 | buffer[6];
						dec = bcd2dec24(bcd);
						OnResponseReadDirection(CorrectAngle(dec * 0.01));
						break;
					case 0x8B:
						OnResponseSetBaudRate(buffer[4] == 0x00);
						break;
					case 0x86:
						OnResponseSetMagneticDeclination(buffer[4] == 0x00);
						break;
					case 0x87:
						bcd = buffer[4] << 8 | buffer[5];
						dec = bcd2dec16(bcd);
						OnResponseReadMagneticDeclination(dec * 0.01);
						break;
					case 0x8F:
						OnResponseSetAddress(buffer[4] == 0x00);
						break;
					case 0x1F:
						OnResponseQueryAddress(buffer[4]);
						break;
					case 0x8C:
						OnResponseSetOutputMode(buffer[4] == 0x00);
						break;
					case 0x8A:
						OnResponseSaveSetting(buffer[4] == 0x00);
						break;
					default:
						break;
					}
				}
			}
		}
		
		void CInterfaceSEC225::ReadThread()
		{
			while (m_threadRead.IsRun())
			{
				ReadDirection();
				Sleep(100);
			}
		}

		CCorrectionTable::CCorrectionTable()
		{
			m_vCalibTable.clear();
			FILE* fp = fopen("table.csv", "rt");
			if (fp)
			{
				double referance = 0, value = 0;
				int nRead = 0;
				while (1)
				{
					nRead = fscanf(fp, "%lf,%lf\n", &referance, &value);
					if (nRead > 0) m_vCalibTable.push_back(std::pair<double, double>(referance, value));
					else break;
				}
				fclose(fp);
			}
		}

		double CCorrectionTable::CorrectAngle(double angle)
		{
			double ret = -1;
			if (m_vCalibTable.size())
			{
				for (unsigned int i = 0; i < m_vCalibTable.size() - 1; i++)
				{
					double x1 = m_vCalibTable[i].first;
					double y1 = m_vCalibTable[i].second;
					double x2 = m_vCalibTable[i + 1].first;
					double y2 = m_vCalibTable[i + 1].second;
					if (x1 <= angle && angle <= x2)
					{
						ret = ((y2 - y1)*(angle - x1) / (x2 - x1)) + y1;
						break;
					}
				}
			}
			else ret = angle;
			while (ret > 360) ret -= 360;
			while (ret < 0) ret += 360;
			return ret;
		}
	}
}