#include "InterfaceGpsTdr2000.h"
#include <stdio.h>
#include <string>
#include <vector>
#include <chrono>
#include <limits>

namespace SHI
{
	namespace Device
	{
		namespace TDR2000
		{
			CInterfaceGpsTdr2000::CInterfaceGpsTdr2000()
				: m_azimuth(0)
			{
				m_pObserver = nullptr;
				ResetLogState(std::chrono::steady_clock::now());
				m_socket.RegisterCallbackConnection(&CInterfaceGpsTdr2000::OnConnected, this);
				m_socket.RegisterCallbackDisConnection(&CInterfaceGpsTdr2000::OnDisConnected, this);
				m_socket.RegisterCallbackReceiveData(&CInterfaceGpsTdr2000::OnServerData, this);
			}

			CInterfaceGpsTdr2000::~CInterfaceGpsTdr2000()
			{
				Destroy();
			}

			void CInterfaceGpsTdr2000::ResetLogState(const std::chrono::steady_clock::time_point& now)
			{
				m_lastLogTime = now;
				m_logFrameCount = 0;
				m_logQualityCount = 0;
				m_logQualitySum = 0.0;
				m_logHdopSum = 0.0;
				m_logQualityMin = INT_MAX;
				m_logQualityMax = INT_MIN;
			}

			void CInterfaceGpsTdr2000::SetObserver(CObserverGpsTdr2000* pObserver)
			{
				m_pObserver = pObserver;
			}

			bool CInterfaceGpsTdr2000::Create(const std::string addr, uint32_t port)
			{
				bool ret = false;
				ret = m_socket.CreateTcpClient(addr, static_cast<uint16_t>(port));
				printf("GPS Socket Create %d =-> IP = %s, port = %d \n", ret, addr.c_str(), port);
				
				return ret;
			}


			void CInterfaceGpsTdr2000::Disconnection()
			{
				m_socket.Disconnect();
			}

			void CInterfaceGpsTdr2000::Destroy()
			{
				m_socket.Destroy();
			}


			std::string ParseMessage(const std::string& message, const std::string& header)
			{
				std::string ret;
				size_t posHeader = message.find(header);
				if (posHeader != std::string::npos)
				{
					ret = message.substr(posHeader, message.find('*', posHeader) - posHeader);
				}
				return ret;
			}

			std::vector<std::string> SepetateMessage(const std::string& message, char seperator)
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

			void CInterfaceGpsTdr2000::OnServerData(const Routine::IO::CSocket& sock, const std::string& ip, uint16_t port, Routine::void_ptr pData, int32_t size)
			{
				std::string message(static_cast<char*>(pData), size);
				const auto vMessage = SepetateMessage(message, '\n');

				for(const auto& msg :vMessage)
				{
					std::string strGNGGA = ParseMessage(msg, "$GNGGA");
					std::string strGNHDT = ParseMessage(msg, "$GNHDT");
					std::string strGPGGA = ParseMessage(msg, "$GPGGA");//사용
					std::string strGPHDT = ParseMessage(msg, "$GPHDT");//사용

					if (strGNHDT.size())
					{
						std::vector<std::string> msg = SepetateMessage(strGNHDT, ',');
						if (msg.size() == 3)
						{
							if (!msg[1].empty())
							{
								m_azimuth = static_cast<float>(atof(msg[1].c_str())); // Heading
							}
							else
							{
								m_azimuth = std::numeric_limits<float>::quiet_NaN();
							}
						}
					}

					if (strGPHDT.size())
					{
						std::vector<std::string> msg = SepetateMessage(strGPHDT, ',');
						if (msg.size() == 3)
						{
							if (!msg[1].empty())
							{
								m_azimuth = static_cast<float>(atof(msg[1].c_str())); // Heading
							}
							else
							{
								m_azimuth = std::numeric_limits<float>::quiet_NaN();
							}
						}
					}

					StGpsData gpsData;
					if (strGNGGA.size())
					{
						std::vector<std::string> msg = SepetateMessage(strGNGGA, ',');
						if (msg.size() == 15)
						{
							gpsData.time = std::stof(msg[1].c_str()); // time
							gpsData.latitude = std::stof(msg[2].c_str()); // Latitude
							gpsData.longitude = std::stof(msg[4].c_str()); // Longitude
							gpsData.numOfSatellites = std::stoi(msg[7].c_str()); // num of satellites
							gpsData.quality = std::stoi(msg[6].c_str()); // Quality
							gpsData.hdop = std::stof(msg[8].c_str()); // HDOP(horizontal dilution of Precision)
							gpsData.height = std::stof(msg[9].c_str()); // height
							gpsData.azimuth = m_azimuth;
						}
					}
					if (strGPGGA.size())
					{
						// GPS 관련 정보 참고
						// https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
						// 
						std::vector<std::string> msg = SepetateMessage(strGPGGA, ',');
						if (msg.size() == 15)
						{
							gpsData.time = std::stof(msg[1].c_str()); // time
							gpsData.latitude = std::stof(msg[2].c_str()); // Latitude
							gpsData.longitude = std::stof(msg[4].c_str()); // Longitude
							gpsData.numOfSatellites = std::stoi(msg[7].c_str()); // num of satellites
							gpsData.quality = std::stoi(msg[6].c_str()); // Quality
							gpsData.hdop = std::stof(msg[8].c_str()); // HDOP(horizontal dilution of Precision)
							gpsData.height = std::stof(msg[9].c_str()); // height
							gpsData.azimuth = m_azimuth;
						}
					}

					if (strGNGGA.size() || strGPGGA.size())
					{
						// update data
						m_gpsData = gpsData;

						// call callback function
						if (m_pObserver) m_pObserver->OnGpsData(m_gpsData);

						const auto nowTime = std::chrono::steady_clock::now();
						++m_logFrameCount;
						m_logHdopSum += m_gpsData.hdop;
						if (m_gpsData.quality >= 0)
						{
							++m_logQualityCount;
							m_logQualityMin = (m_logQualityMin > m_gpsData.quality) ? m_gpsData.quality : m_logQualityMin;
							m_logQualityMax = (m_logQualityMax < m_gpsData.quality) ? m_gpsData.quality : m_logQualityMax;
							m_logQualitySum += m_gpsData.quality;
						}
						LogStatisticsIfNeeded(nowTime);
					}
				}
				if (m_pObserver) m_pObserver->OnGpsConnected(ip.c_str(), port);
			}

			void CInterfaceGpsTdr2000::OnConnected(const std::string& ip, uint16_t port)
			{
				printf("GPS Connected\n");
			}

			void CInterfaceGpsTdr2000::OnDisConnected(const std::string& ip, uint16_t port)
			{
				printf("GPS Disonnected \n");
			}

			void CInterfaceGpsTdr2000::LogStatisticsIfNeeded(const std::chrono::steady_clock::time_point& now)
			{
				const double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - m_lastLogTime).count();
				if (elapsed < 1.0)
				{
					return;
				}

				if (m_logFrameCount == 0)
				{
					ResetLogState(now);
					return;
				}

				const double fps = static_cast<double>(m_logFrameCount) / elapsed;
				const double avgHdop = m_logFrameCount > 0 ? (m_logHdopSum / m_logFrameCount) : 0.0;
				const double avgQuality = m_logQualityCount > 0 ? (m_logQualitySum / m_logQualityCount) : 0.0;
				const int qualityMin = m_logQualityCount > 0 ? m_logQualityMin : -1;
				const int qualityMax = m_logQualityCount > 0 ? m_logQualityMax : -1;

				printf("[GPS] dt=%.2fs fps=%.2f quality[min/avg/max]=%d/%.2f/%d hdop_avg=%.2f lat=%.6f lon=%.6f sat=%d height=%.2f azimuth=%.2f\n",
					elapsed,
					fps,
					qualityMin,
					avgQuality,
					qualityMax,
					avgHdop,
					m_gpsData.latitude,
					m_gpsData.longitude,
					m_gpsData.numOfSatellites,
					m_gpsData.height,
					m_gpsData.azimuth);

				ResetLogState(now);
			}
		}
	}
}

