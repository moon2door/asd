#include "ProtocolPanda20a.h"
#include <DevLib/Include/Mathematics/Mathematics.h>
#include <pcl/common/transforms.h>

namespace Device
{
	namespace Panda20a
	{
		const float div90112 = 1.0f / 90112;

		void Convert2PointCloud(pcl::PointCloud<pcl::PointXYZ> &dst, Device::Panda20a::StLidarData* src)
		{
			pcl::PointCloud<pcl::PointXYZ> tmp;
			tmp.reserve(20 * 20);
			for (int idxBlock = 0; idxBlock < 20; idxBlock++)
			{
				double  stdAngle = src->dataBlock[idxBlock].azimuth * 0.01; // -- 기준 각도(deg)

				for (int idxLayer = 0; idxLayer < 20; idxLayer++)
				{
					unsigned short d = src->dataBlock[idxBlock].layer[idxLayer].distance;
					if(d == 0) continue;

					float distance = d*0.004;
					double angleH = stdAngle + LAYER_AZIMUTH_OFFSET[idxLayer];
					float angleV = LAYER_2_ANGLE[idxLayer];
					double x = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetSinTable(angleH);
					double y = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetCosTable(angleH);
					double z = -distance * DevLib::Mathematics::GetSinTable(angleV);

					tmp.push_back(pcl::PointXYZ(x, y, z));
				}
			}
			dst.insert(dst.end(), tmp.begin(), tmp.end());
		}

		void Convert2PointCloud(pcl::PointCloud<pcl::PointXYZI> &dst, Device::Panda20a::StLidarData* src)
		{
			pcl::PointCloud<pcl::PointXYZI> tmp;
			tmp.reserve(20 * 20);
			for (int idxBlock = 0; idxBlock < 20; idxBlock++)
			{
				double  stdAngle = src->dataBlock[idxBlock].azimuth * 0.01; // -- 기준 각도(deg)

				for (int idxLayer = 0; idxLayer < 20; idxLayer++)
				{
					unsigned short d = src->dataBlock[idxBlock].layer[idxLayer].distance;
					if (d == 0) continue;

					float distance = d*0.004;
					double angleH = stdAngle + LAYER_AZIMUTH_OFFSET[idxLayer];
					float angleV = LAYER_2_ANGLE[idxLayer];
					double x = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetSinTable(angleH);
					double y = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetCosTable(angleH);
					double z = -distance * DevLib::Mathematics::GetSinTable(angleV);
					unsigned char intensity = src->dataBlock[idxBlock].layer[idxLayer].reflectivity;

					pcl::PointXYZI p;
					p.x = static_cast<float>(x);
					p.y = static_cast<float>(y);
					p.z = static_cast<float>(z);
					p.intensity = static_cast<float>(intensity);
					tmp.push_back(p);
				}
			}
			dst.insert(dst.end(), tmp.begin(), tmp.end());
		}
	}
}