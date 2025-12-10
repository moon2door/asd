
#include "Convert.h"
#include "BuildInfo.h"
#include "../Config/CraneInfo.h"

namespace SHI
{
	namespace Convert
	{
		void XYZ2PointCloud(SHI::PointCloud& dst, SHI::Data::StXYZPoints* src)
		{
			dst.clear();
			dst.resize(src->GetXYZSize());
			//memcpy(dst.points.data(), src->GetXYZ(), src->GetXYZSize() * sizeof(pcl::PointXYZ));
			for (unsigned int i=0; i<src->GetXYZSize(); i++)
			{
				dst[i].x = src->GetXYZ()[i].X;
				dst[i].y = src->GetXYZ()[i].Y;
				dst[i].z = src->GetXYZ()[i].Z;
			}
		}

		bool XYZIndices2Cluster(SHI::Data::StCluster &dst, const SHI::PointCloudPtr cloud, const SHI::PCLIndicesVectorPtr &indices, const std::vector<unsigned char>& labels, const SHI::CraneAttitudePtr& attitude)
		{
			bool ret = true;
			memset(&dst, 0, sizeof(SHI::Data::StCluster));
			dst.version = SHI::BuildInfo::GetBuildDate();
			dst.attitude = *((SHI::Data::StCraneAttitude*)attitude.get());
			dst.InitBuffer();

			// XYZ 데이터
			ret = ret && dst.AllocXYZ(cloud->size());
			if (ret) memcpy(dst.GetXYZ(), cloud->points.data(), cloud->size() * sizeof(SHI::Data::StXYZ));

			// indices 크기 계산
			unsigned int sizeCluster = 0;
			for (size_t i = 0; i < indices->size(); i++)
			{
				sizeCluster += indices->data()[i].indices.size();
			}

			// 클러스터 데이터
			ret = ret && dst.AllocClusterInfo(indices->size());
			ret = ret && dst.AllocClusterIndices(sizeCluster);
			ret = ret && dst.AllocLabel(labels.size());
			if (ret)
			{
				unsigned int index = 0;
				for (size_t i = 0; i < indices->size(); i++)
				{
					// info 작성
					SHI::Data::StClusterInfo info;
					info.Index = index;
					info.Size = indices->data()[i].indices.size();
					dst.GetClusterInfo()[i] = info;

					// indices 작성
					memcpy(dst.GetClusterIndices() + info.Index, indices->data()[i].indices.data(), info.Size * sizeof(int));

					// index 갱신
					index += info.Size;
				}

				// Labels 작성
				memcpy(dst.GetLabel(), labels.data(), labels.size() * sizeof(unsigned char));
			}
			return ret;
		}

		void Cluster2XYZIndices(SHI::Data::StCluster* src, SHI::PointCloud &outCloud, SHI::PCLIndicesVector &outClusterIndices, std::vector<unsigned char>& labels)
		{
			// 포인트 클라우드 변환
			outCloud.clear();
			outCloud.resize(src->GetXYZSize());
			memcpy(outCloud.points.data(), src->GetXYZ(), src->GetXYZSize() * sizeof(SHI::Data::StXYZ));

			// Labels 변환
			labels.clear();
			labels.resize(src->GetLabelSize());
			memcpy(labels.data(), src->GetLabel(), src->GetLabelSize() * sizeof(unsigned char));

			// 클러스터 인덱스 리스트 변환
			outClusterIndices.clear();
			outClusterIndices.reserve(src->GetClusterInfoSize());
			for (size_t idxCluster = 0; idxCluster < src->GetClusterInfoSize(); idxCluster++)
			{
				unsigned int index = src->GetClusterInfo()[idxCluster].Index;
				unsigned int size = src->GetClusterInfo()[idxCluster].Size;

				SHI::PCLIndices indices;
				indices.indices.resize(size);
				memcpy(indices.indices.data(), &src->GetClusterIndices()[index], size * sizeof(int));
				outClusterIndices.push_back(indices);
			}
		}

		bool Distance2DataDistance(SHI::Data::StDistance& dst, const SHI::PointCloudPtr cloud, const SHI::PCLIndicesVectorPtr indices, const std::vector<unsigned char>& labels,
			const SHI::DistanceInfoVectorPtr & distance, const std::vector<unsigned char>& distanceLabels, const SHI::CraneAttitudePtr& attitude)
		{
			bool ret = true;
			memset(&dst, 0, sizeof(SHI::Data::StDistance));
			dst.version = SHI::BuildInfo::GetBuildDate();
			dst.attitude = *((SHI::Data::StCraneAttitude*)attitude.get());
			dst.InitBuffer();

			// XYZ 데이터
			ret = ret && dst.AllocXYZ(cloud->size());
			if (ret) memcpy(dst.GetXYZ(), cloud->points.data(), cloud->size() * sizeof(SHI::Data::StXYZ));

			// indices 크기 계산
			unsigned int sizeCluster = 0;
			for (size_t i = 0; i < indices->size(); i++)
			{
				sizeCluster += indices->data()[i].indices.size();
			}

			// 클러스터 데이터
			ret = ret && dst.AllocClusterInfo(indices->size());
			ret = ret && dst.AllocClusterIndices(sizeCluster);
			ret = ret && dst.AllocLabel(labels.size());
			if (ret)
			{
				unsigned int index = 0;
				for (size_t i = 0; i < indices->size(); i++)
				{
					// info 작성
					SHI::Data::StClusterInfo info;
					info.Index = index;
					info.Size = indices->data()[i].indices.size();
					dst.GetClusterInfo()[i] = info;

					// indices 작성
					memcpy(dst.GetClusterIndices() + info.Index, indices->data()[i].indices.data(), info.Size * sizeof(int));

					// index 갱신
					index += info.Size;
				}

				// Labels 작성
				memcpy(dst.GetLabel(), labels.data(), labels.size() * sizeof(unsigned char));
			}

			// 거리 데이터
			ret = ret && dst.AllocDistanceInfo(distance->size());
			ret = ret && dst.AllocDistanceLabel(distanceLabels.size());
			if (ret)
			{
				memcpy(dst.GetDistanceInfo(), distance->data(), distance->size() * sizeof(SHI::Data::StDistanceInfo));
				memcpy(dst.GetDistanceLabel(), distanceLabels.data(), distanceLabels.size() * sizeof(unsigned char));
			}
			return ret;
		}

		void DataDistance2Distance(SHI::Data::StDistance* src, SHI::PointCloud &outCloud, SHI::PCLIndicesVector &outClusterIndices, std::vector<unsigned char>& labels, 
			SHI::DistanceInfoVector& distance, std::vector<unsigned char>& distanceLabels, SHI::CraneAttitude& attitude)
		{
			// 포인트 클라우드 변환
			outCloud.clear();
			outCloud.resize(src->GetXYZSize());
			memcpy(outCloud.points.data(), src->GetXYZ(), src->GetXYZSize() * sizeof(SHI::Data::StXYZ));

			// Labels 변환
			labels.clear();
			labels.resize(src->GetLabelSize());
			memcpy(labels.data(), src->GetLabel(), src->GetLabelSize() * sizeof(unsigned char));

			// 클러스터 인덱스 리스트 변환
			outClusterIndices.clear();
			outClusterIndices.reserve(src->GetClusterInfoSize());
			for (size_t idxCluster = 0; idxCluster < src->GetClusterInfoSize(); idxCluster++)
			{
				unsigned int index = src->GetClusterInfo()[idxCluster].Index;
				unsigned int size = src->GetClusterInfo()[idxCluster].Size;

				SHI::PCLIndices indices;
				indices.indices.resize(size);
				memcpy(indices.indices.data(), &src->GetClusterIndices()[index], size * sizeof(int));
				outClusterIndices.push_back(indices);
			}

			// Distance 변환
			distance.clear();
			distance.resize(src->GetDistanceInfoSize());
			memcpy(distance.data(), src->GetDistanceInfo(), src->GetDistanceInfoSize() * sizeof(SHI::DistanceInfo));

			// Distance Labels 변환
			distanceLabels.clear();
			distanceLabels.resize(src->GetDistanceInfoSize());
			memcpy(distanceLabels.data(), src->GetDistanceLabel(), src->GetDistanceLabelSize() * sizeof(unsigned char));

			// attitude 변환
			memcpy(&attitude, &src->attitude, sizeof(SHI::CraneAttitude));
		}

	}
}