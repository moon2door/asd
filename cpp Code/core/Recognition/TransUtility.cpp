
#include "TransUtility.h"
#include <pcl/common/transforms.h>

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>& pSrc, pcl::PointCloud<pcl::PointXYZ>& pDst, double_t centerPos[3], double_t rotation[3], double_t translation[3])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Affine3f trans1 = Eigen::Affine3f::Identity();
	trans1 = Eigen::Affine3f::Identity();
	trans1.translation() += Eigen::Vector3f(-centerPos[0], -centerPos[1], -centerPos[2]);
	pcl::transformPointCloud(pSrc, *cloudTmp, trans1);

	Eigen::Affine3f trans2 = Eigen::Affine3f::Identity();
	trans2.rotate(Eigen::AngleAxisf(rotation[0] * 3.141592 / 180.0, Eigen::Vector3f::UnitX()));
	trans2.rotate(Eigen::AngleAxisf(rotation[1] * 3.141592 / 180.0, Eigen::Vector3f::UnitY()));
	trans2.rotate(Eigen::AngleAxisf(rotation[2] * 3.141592 / 180.0, Eigen::Vector3f::UnitZ()));
	trans2.translation() += Eigen::Vector3f(centerPos[0] + translation[0], centerPos[1] + translation[1], centerPos[2] + translation[2]);
	pcl::transformPointCloud(*cloudTmp, pDst, trans2);
}