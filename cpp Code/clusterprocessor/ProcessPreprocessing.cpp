#include "../ClusterProcessor.h"
#include <pcl/filters/voxel_grid.h>
#include <Utility/Convert.h>

bool CClusterProcessor::ProcessPreProcessing(SHI::PointCloud& dst, SHI::Data::StXYZPoints* src)
{
	bool ret = false;
	// 파라미터 입력
	SHI::ClusterParamPtr param = GetClusterParam();

	// 클러스터링 전처리
	if (param)
	{
		// 입력 포인트 변환: XYZ -> 포인트 클라우드
		SHI::PointCloudPtr cloudInput(new SHI::PointCloud);
		SHI::Convert::XYZ2PointCloud(*cloudInput, src);

		// 포인트 클라우드 다운샘플
		SHI::PointCloudPtr cloudFiltered(new SHI::PointCloud);
		if (cloudInput->size() > 0)
		{
			pcl::VoxelGrid<pcl::PointXYZ> vg;
			vg.setInputCloud(cloudInput);
			vg.setLeafSize(param->downsampleSize, param->downsampleSize, param->downsampleSize);
			vg.setDownsampleAllData(true);
			vg.filter(*cloudFiltered);
		}

		// 결과 반환
		dst = *cloudFiltered;
		ret = true;
	}
	return ret;
}