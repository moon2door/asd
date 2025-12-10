#include "../ClusterProcessor.h"

#include <Utility/Filter.h>
#include <Utility/Transform.h>
#include <Config/CraneInfo.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

void CClusterProcessor::ProcessPreClustering(SHI::PointCloud& dst, SHI::PointCloud& dstGround, SHI::PointCloudPtr src)
{
	// 입력 파라미터
	SHI::ClusterParamPtr param = GetClusterParam();
	SHI::CraneAttitudePtr attitude = GetCraneAttitude();

	if (param && attitude)
	{
		// 포인트 클라우드 누적
		m_qCloudInput.push(src);
		while (m_qCloudInput.size() > param->maxAccum) m_qCloudInput.pop();

		// 누적 데이터 가져오기
		SHI::PointCloudPtr cloudAccum(new SHI::PointCloud);
		size_t queueSize = m_qCloudInput.size();
		for (int i = 0; i < queueSize; i++)
		{
			SHI::PointCloudPtr cloud = m_qCloudInput.front();
			cloudAccum->insert(cloudAccum->end(), cloud->begin(), cloud->end());
			m_qCloudInput.pop();
			m_qCloudInput.push(cloud);
		}

		// 포인트 클라우드 다운샘플
		if (cloudAccum->size() > 0)
		{
			SHI::PointCloudPtr cloudFiltered(new SHI::PointCloud);
			pcl::VoxelGrid<pcl::PointXYZ> vg;
			vg.setInputCloud(cloudAccum);
			vg.setLeafSize(param->downsampleSize, param->downsampleSize, param->downsampleSize);
			vg.setDownsampleAllData(true);
			vg.filter(*cloudFiltered);

			// 지면 포인트 분리
			SHI::PointCloudPtr cloudGround(new SHI::PointCloud);
			if (cloudFiltered->size() > 0)
			{
				pcl::PassThrough<pcl::PointXYZ> pass;
				pass.setInputCloud(cloudFiltered);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(attitude->groundHeight + 1.0, param->roi.roi.minZ);
				pass.filter(*cloudGround);

				pass.setFilterLimits(param->roi.roi.minZ, param->roi.roi.maxZ);
				pass.filter(*cloudFiltered);
			}

			// 지면 데이터 필터 처리
			if (cloudGround->size() > 0)
			{
				pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
				outrem.setInputCloud(cloudGround);
				outrem.setRadiusSearch(3.0);
				outrem.setMinNeighborsInRadius(15);
				outrem.filter(dstGround);
			}
			else dstGround = *cloudGround;

			// 남은 포인트 필터 처리
			if (cloudFiltered->size() > 0)
			{
				pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
				sor.setMeanK(10);
				sor.setStddevMulThresh(2.0);
				sor.setInputCloud(cloudFiltered);
				sor.filter(dst);
			}
			else dst = *cloudFiltered;
		}
	}
}

