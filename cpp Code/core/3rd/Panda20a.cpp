#include "Panda20a.h"
#include <rapidjson/document.h>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>

namespace Device
{
	namespace Panda20a
	{
		CPanda20a::CPanda20a(CObserverPanda20a* pObserver)
			: m_pObserver(pObserver)
		{
			m_udpLidar = new DevLib::IO::Socket::CReceiverObject(this);
		}
		
		CPanda20a::~CPanda20a()
		{
			delete m_udpLidar;
		}
		
		void CPanda20a::SetObserver(CObserverPanda20a* pObserver)
		{
			m_pObserver = pObserver;
		}

		bool CPanda20a::Create(int portLidar, char* addr, char* bindAddr)
		{
			bool ret = false;
			ret = m_udpLidar->Create(SOCK_DGRAM);
			m_udpLidar->SetPacketSize(1270);
			ret = m_udpLidar->Bind(portLidar, bindAddr);
			ret = m_udpLidar->ServiceStart();

			return ret;
		}

		void CPanda20a::Destroy()
		{
			m_udpLidar->Destroy();
		}

		void CPanda20a::OnReceiverData(char* ip, int port, void* data, int size)
		{
			if (size == sizeof(Device::Panda20a::StLidarData))
			{
				OnReceiverLidar(static_cast<Device::Panda20a::StLidarData*>(data));
			}
		}

		void CPanda20a::OnReceiverLidar(Device::Panda20a::StLidarData* data)
		{
			// Update frame rate
			if (++countFrameRate >= 1000)
			{
				double e = elapsedFps.GetElapseTime() * 0.000001;
				double fps = 1000.0 / e;
				countFrameRate = 0;

				UpdateFrameRate(fps);
			}

			// Update rpm
			double rpm2 = 0;
			double time = elapsedRpm.GetElapseTimeContinue() * 0.000001;
			double angle = data->dataBlock[0].azimuth * 0.01;

			double dangle = angle - prevAngle;
			if (dangle < 0) dangle += 360.0;
			accumAngle += dangle;

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
			for (unsigned int i = 0; i < 20; i++)
			{
				for (unsigned int layer = 0; layer < 20; layer++)
				{
					maxd = (std::max)(data->dataBlock[i].layer[layer].distance, maxd);
				}
			}

			if (++countMaxDistance >= 2560)
			{
				UpdateMaxDistance(maxd * 0.004);
				countMaxDistance = 0;
				maxd = 0;
			}

			// Callback
			if (m_pObserver) m_pObserver->OnReceiverLidar(data);
		}
	}
}