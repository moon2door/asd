#include "VLP16.h"
#include <DevLib/Include/Core/CElapseTimer/ElapseTimer.h>
#include <rapidjson/document.h>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>

namespace Device
{
	namespace VLP16
	{
		CVLP16::CVLP16(CObserverVLP16* pObserver)
			: m_pObserver(pObserver)
		{
			m_udpLidar = new DevLib::IO::Socket::CReceiverObject(this);
		}
		
		CVLP16::~CVLP16()
		{
			delete m_udpLidar;
		}
		
		void CVLP16::SetObserver(CObserverVLP16* pObserver)
		{
			m_pObserver = pObserver;
		}

		bool CVLP16::Create(int portLidar, char* addr, char* bindAddr)
		{
			bool ret = false;
			ret = m_udpLidar->Create(SOCK_DGRAM);
			m_udpLidar->SetPacketSize(1206);
			ret = m_udpLidar->Bind(portLidar, bindAddr);
			ret = m_udpLidar->ServiceStart();

			return ret;
		}

		void CVLP16::Destroy()
		{
			m_udpLidar->Destroy();
		}

		void CVLP16::OnReceiverData(char* ip, int port, void* data, int size)
		{
			if (size == sizeof(Device::VLP16::StLidarData))
			{
				OnReceiverLidar(static_cast<Device::VLP16::StLidarData*>(data));
			}
		}

		void CVLP16::OnReceiverLidar(Device::VLP16::StLidarData* data)
		{
			// Update frame rate
			static DevLib::CElapseTimer elapsed;
			static int countFrameRate = 0;
			if (++countFrameRate >= 1000)
			{
				double e = elapsed.GetElapseTime() * 0.000001;
				double fps = 1000.0 / e;
				countFrameRate = 0;

				UpdateFrameRate(fps);
			}

			// Update rpm
			static double prevAngle = 0;
			static double accumAngle = 0;
			static double startTime = data->timestamp * 0.000001;
			static double rpm2 = 0;
			double time = data->timestamp * 0.000001;
			double angle = 360.f*data->dataBlock[0].azimuth * 0.01;

			double dangle = angle - prevAngle;
			if (dangle < 0) dangle += 360.0;
			accumAngle += dangle;

			static int countRPM = 0;
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
			static unsigned short max = 0;
			static int countMaxDistance = 0;

			for (unsigned int i = 0; i < 12; i++)
			{
				for (unsigned int layer = 0; layer < 32; layer++)
				{
					max = (std::max)(data->dataBlock[i].layer[layer].distance, max);
				}
			}

			if (++countMaxDistance >= 2560)
			{
				UpdateMaxDistance(max * 0.002);
				countMaxDistance = 0;
				max = 0;
			}

			// Callback
			if (m_pObserver) m_pObserver->OnReceiverLidar(data);
		}
	}
}