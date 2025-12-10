#include "ProtocolVLP16.h"
#include <DevLib/Include/Mathematics/Mathematics.h>
#include <pcl/common/transforms.h>

namespace Device
{
	namespace VLP16
	{
		const float div90112 = 1.0f / 90112;

		void Convert2PointCloud(pcl::PointCloud<pcl::PointXYZ> &dst, StLidarData* src)
		{
			pcl::PointCloud<pcl::PointXYZ> tmp;
			tmp.reserve(32 * 12);
			for (int idxBlock = 0; idxBlock < 12; idxBlock++)
			{
				double  stdAngle = src->dataBlock[0].azimuth * 0.01; // -- 기준 각도(deg)

				for (int idxLayer = 0; idxLayer < 32; idxLayer++)
				{
					unsigned short d = src->dataBlock[idxBlock].layer[idxLayer].distance;
					if(d == 0) continue;
					
					// RPM으로부터 방위 계산
					int seqIndex = idxBlock * 2 + idxLayer % 2;
					int layerIndex = idxLayer % 16;
					double timeOffset = (55.296 * seqIndex) + (2.304 * layerIndex);
					double angleOffset = (600/*=RPM*/ / 60000000.0) * 360 * timeOffset;
					double angleH = stdAngle + angleOffset;

					float distance = d*0.002;
					float angleV = LAYER_2_ANGLE[layerIndex];
					double x = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetSinTable(angleH);
					double y = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetCosTable(angleH);
					double z = distance * DevLib::Mathematics::GetSinTable(angleV);
					unsigned char intensity = src->dataBlock[idxBlock].layer[idxLayer].reflectivity;

					tmp.push_back(pcl::PointXYZ(x, y, z));
				}
			}
			dst.insert(dst.end(), tmp.begin(), tmp.end());
		}

		void Convert2PointCloud(pcl::PointCloud<pcl::PointXYZI> &dst, StLidarData* src)
		{
			pcl::PointCloud<pcl::PointXYZI> tmp;
			tmp.reserve(32 * 12);
			for (int idxBlock = 0; idxBlock < 12; idxBlock++)
			{
				double  stdAngle = src->dataBlock[0].azimuth * 0.01; // -- 기준 각도(deg)

				for (int idxLayer = 0; idxLayer < 32; idxLayer++)
				{
					unsigned short d = src->dataBlock[idxBlock].layer[idxLayer].distance;
					if (d == 0) continue;

					// RPM으로부터 방위 계산
					int seqIndex = idxBlock * 2 + idxLayer % 2;
					int layerIndex = idxLayer % 16;
					double timeOffset = (55.296 * seqIndex) + (2.304 * layerIndex);
					double angleOffset = (600/*=RPM*/ / 60000000.0) * 360 * timeOffset;
					double angleH = stdAngle + angleOffset;

					float distance = d*0.002;
					float angleV = LAYER_2_ANGLE[layerIndex];
					double x = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetSinTable(angleH);
					double y = distance * DevLib::Mathematics::GetCosTable(angleV) * DevLib::Mathematics::GetCosTable(angleH);
					double z = distance * DevLib::Mathematics::GetSinTable(angleV);
					double intensity = src->dataBlock[idxBlock].layer[idxLayer].reflectivity / 255.0;

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