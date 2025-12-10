#include "../ClusterProcessor.h"

#include <Utility/Convert.h>
void CClusterProcessor::ProcessPostProcessing(SHI::Data::StCluster &dst, SHI::PointCloudPtr cloud, SHI::PCLIndicesVectorPtr vIndices, std::vector<unsigned char>& labels, const SHI::CraneAttitudePtr& attitude)
{
	// 결과 데이터 변환
	SHI::Convert::XYZIndices2Cluster(dst, cloud, vIndices, labels, attitude);
}