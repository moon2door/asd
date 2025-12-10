#include "../ClusterProcessor.h"

#include <Utility/Cluster.h>
#include <Utility/Filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <Config/CraneInfo.h>

bool CClusterProcessor::ProcessClustering(SHI::PCLIndicesVector& dstIndices, SHI::PointCloud& dstCloud, SHI::PointCloudPtr& cloudInput)
{
	bool ret = false;
	SHI::ClusterParamPtr param = GetClusterParam();
	if (param)
	{
		// 클러스터 ROI 처리
		SHI::IndicesPtr indices(new SHI::Indices);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloudInput);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(param->roi.roi.minX, param->roi.roi.maxX);
		pass.filter(*indices);

		pass.setFilterFieldName("y");
		pass.setFilterLimits(param->roi.roi.minY, param->roi.roi.maxY);
		pass.setIndices(indices);
		pass.filter(*indices);

		pass.setFilterFieldName("z");
		pass.setFilterLimits(param->roi.roi.minZ, param->roi.roi.maxZ);
		pass.setIndices(indices);
		pass.filter(*indices);

		SHI::PCLIndicesPtr pclindices(new SHI::PCLIndices);
		pclindices->indices = *indices;

		// 클러스터링
		if (cloudInput->size() > 0)
		{
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloudInput);

			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec2;
			ec2.setIndices(pclindices);
			ec2.setClusterTolerance(param->clusterTolerance);
			ec2.setMinClusterSize(param->minClusterSize);
			ec2.setMaxClusterSize(param->maxClusterSize);
			ec2.setSearchMethod(tree);
			ec2.setInputCloud(cloudInput);
			ec2.extract(dstIndices);
			dstCloud = *cloudInput;
			ret = true;
		}
	}
	return ret;
}