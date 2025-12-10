#include "Panda40p.h"
#include <DevLib/Include/Core/CElapseTimer/ElapseTimer.h>
#include <rapidjson/document.h>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>

namespace Device
{
	namespace Panda40p
	{
		CPanda40p::CPanda40p(CObserverPanda40p* pObserver)
			: m_pObserver(pObserver)
		{
			m_udpLidar = new DevLib::IO::Socket::CReceiverObject(this);
		}
		
		CPanda40p::~CPanda40p()
		{
			delete m_udpLidar;
		}
		
		void CPanda40p::SetObserver(CObserverPanda40p* pObserver)
		{
			m_pObserver = pObserver;
		}

		bool CPanda40p::Create(int portLidar, char* addr, char* bindAddr)
		{
			bool ret = false;
			ret = m_udpLidar->Create(SOCK_DGRAM);
			m_udpLidar->SetPacketSize(1262);
			ret = m_udpLidar->Bind(portLidar, bindAddr);
			ret = m_udpLidar->ServiceStart();

			return ret;
		}

		void CPanda40p::Destroy()
		{
			m_udpLidar->Destroy();
		}

		void CPanda40p::OnReceiverData(char* ip, int port, void* data, int size)
		{
			if (size == sizeof(Device::Panda40p::StLidarData))
			{
				OnReceiverLidar(static_cast<Device::Panda40p::StLidarData*>(data));
			}
		}

		void CPanda40p::OnReceiverLidar(Device::Panda40p::StLidarData* data)
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
			double timestamp = data->hour + data->minute + data->second;
			static double prevAngle = 0;
			static double accumAngle = 0;
			static DevLib::CElapseTimer elapsedRpm;
			static double startTime = elapsedRpm.GetElapseTimeContinue() * 0.000001;
			static double rpm2 = 0;
			double time = elapsedRpm.GetElapseTimeContinue() * 0.000001;
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
				UpdateMaxDistance(max * 0.004);
				countMaxDistance = 0;
				max = 0;
			}

			// Callback
			if (m_pObserver) m_pObserver->OnReceiverLidar(data);
		}
	}
}