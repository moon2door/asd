#include "../DistanceProcessor.h"
#include <Utility/Filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Config/CraneInfo.h>

void CDistanceProcessor::ProcessHookDistance(SHI::DistanceInfoVectorPtr& distance, const SHI::PointCloudPtr& clusterPoints, const SHI::PCLIndicesVectorPtr& clusterIndices, const std::vector<unsigned char> &labels, const SHI::DistanceParamPtr& param)
{
	distance->clear();

	if (labels.size() == clusterIndices->size())
	{
		// 물체 포인트 클라우드
		SHI::PointCloudPtr objectPoints(new SHI::PointCloud);
		SHI::Filter::FilterClusterLabel(clusterPoints, *objectPoints, clusterIndices, labels, SHI::LABEL_OBJECT);

		// 후크 포인트 클라우드
		SHI::PointCloudPtr hookPoints(new SHI::PointCloud);
		SHI::Filter::FilterClusterLabel(clusterPoints, *hookPoints, clusterIndices, labels, SHI::LABEL_HOOK);

		// 후크 ROI
		SHI::StROI hookRoi(hookPoints);
		hookRoi.roi.minX -= param->maxHookDistance;
		hookRoi.roi.minY -= param->maxHookDistance;
		hookRoi.roi.minZ -= param->maxHookDistance;
		hookRoi.roi.maxX += param->maxHookDistance;
		hookRoi.roi.maxY += param->maxHookDistance;
		hookRoi.roi.maxZ += param->maxHookDistance;

		// 후크 ROI 내의 물체 포인트
		SHI::PointCloudPtr objectInliers(new SHI::PointCloud);
		SHI::Filter::FilterROI(objectPoints, *objectInliers, hookRoi);

		if (objectInliers->size() > 0)
		{
			// kdtree
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(objectInliers);

			// 최근방 포인트 탐색
			float minDistanceSqr = FLT_MAX;
			SHI::Point3D pointCluster;
			SHI::Point3D pointHook;
			for (unsigned int i = 0; i < hookPoints->size(); i++)
			{
				std::vector<int> indices;
				std::vector<float> distancesSqr;
				if (kdtree.nearestKSearch(hookPoints->points.at(i), 1, indices, distancesSqr))
				{
					if (indices.size() > 0 && distancesSqr.size() > 0)
					{
						if (minDistanceSqr > distancesSqr[0])
						{
							minDistanceSqr = distancesSqr[0];
							pointHook = hookPoints->points.at(i);
							pointCluster = objectInliers->points.at(indices[0]);
						}
					}
				}
			}

			// 결과 출력
			if (param->minDistance * param->minDistance < minDistanceSqr
				&& minDistanceSqr < param->maxHookDistance * param->maxHookDistance)
			{
				SHI::DistanceInfo distanceInfo;
				distanceInfo.CraneIndex = 200;
				distanceInfo.ClusterIndex = 0;
				distanceInfo.Distance = sqrt(minDistanceSqr);
				distanceInfo.PosCluster.X = pointCluster.x;
				distanceInfo.PosCluster.Y = pointCluster.y;
				distanceInfo.PosCluster.Z = pointCluster.z;
				distanceInfo.PosCrane.X = pointHook.x;
				distanceInfo.PosCrane.Y = pointHook.y;
				distanceInfo.PosCrane.Z = pointHook.z;
				distance->push_back(distanceInfo);
			}

		}
	}
}

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

float _DistanceHorizontalPointClouds(const SHI::PointCloudPtr &cloudCompared, const SHI::PointCloudPtr &cloudRef, SHI::Point3D& compared, SHI::Point3D& ref)
{
	float eps = 0.5;
	float maxDistance = 60;
	float minDistance = FLT_MAX;
	SHI::Point3D _ref = SHI::Point3D();
	SHI::Point3D _compared = SHI::Point3D();

	for (unsigned int idxRef = 0; idxRef < cloudRef->size(); idxRef++)
	{
		// Referance point
		SHI::Point3D p = cloudRef->points[idxRef];

		// object points
		SHI::IndicesPtr indices(new SHI::Indices);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloudCompared);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(p.z - eps, p.z + eps);
		pass.filter(*indices);

		// Cals min distance
		for (unsigned int i = 0; i < indices->size(); i++)
		{
			int idx = indices->data()[i];
			float distanceSq =
				(cloudCompared->points[idx].x - p.x)*(cloudCompared->points[idx].x - p.x) +
				(cloudCompared->points[idx].y - p.y)*(cloudCompared->points[idx].y - p.y);

			if (distanceSq < minDistance)
			{
				minDistance = distanceSq;
				_compared = cloudCompared->points[idx];
				_ref = p;
			}
		}
	}

	// output
	compared = _compared;
	ref = _ref;
	return sqrt(minDistance);
}

void _MakeHookModelGCUpper(SHI::DistanceModelPtr& dst, const SHI::CraneAttitudePtr& attitude, int partNum, const SHI::DistanceParamPtr& param)
{
	SHI::PointCloudPtr cloud(new SHI::PointCloud());
	SHI::PointCloudPtr cloudDown(new SHI::PointCloud());
	SHI::DistanceModelPtr hookModel(new SHI::DistanceModel(partNum, cloud, cloudDown));

	int numHook = attitude->numHook;
	for (unsigned int i = 0; i < numHook; i++)
	{
		if (i == SHI::PierHan::GC_HOOK_XP || i == SHI::PierHan::GC_HOOK_XN)
		{
			SHI::StPoint hookPoint = attitude->hookPoint[i];
			float minHeight = hookPoint.z - 20.0;
			float maxHeight = attitude->hookRoi[i].roi.maxZ;

			minHeight = (std::max)(minHeight, param->minHookCollisionHeight);
			maxHeight = (std::min)(maxHeight, param->maxHookCollisionHeight);

			// Make hook model
			for (float z = minHeight; z < maxHeight; z += param->dHookInterpoint)
			{
				SHI::Point3D p(hookPoint.x, hookPoint.y, z);
				hookModel->cloud->points.push_back(p);
			}

			for (float z = minHeight; z < maxHeight; z += param->dHookInterpointDown)
			{
				SHI::Point3D p(hookPoint.x, hookPoint.y, z);
				hookModel->cloudDownsampled->points.push_back(p);
			}
		}
	}
	dst = hookModel;
}

void _MakeHookModelGCLower(SHI::DistanceModelPtr& dst, const SHI::CraneAttitudePtr& attitude, int partNum, const SHI::DistanceParamPtr& param)
{
	SHI::PointCloudPtr cloud(new SHI::PointCloud());
	SHI::PointCloudPtr cloudDown(new SHI::PointCloud());
	SHI::DistanceModelPtr hookModel(new SHI::DistanceModel(partNum, cloud, cloudDown));

	int numHook = attitude->numHook;
	for (unsigned int i = 0; i < numHook; i++)
	{
		if (i == SHI::PierHan::GC_HOOK_C)
		{
			SHI::StPoint hookPoint = attitude->hookPoint[i];
			float minHeight = hookPoint.z - 20.0;
			float maxHeight = attitude->hookRoi[i].roi.maxZ;

			minHeight = (std::max)(minHeight, param->minHookCollisionHeight);
			maxHeight = (std::min)(maxHeight, param->maxHookCollisionHeight);

			// Make hook model
			for (float z = minHeight; z < maxHeight; z += param->dHookInterpoint)
			{
				SHI::Point3D p(hookPoint.x, hookPoint.y, z);
				hookModel->cloud->points.push_back(p);
			}

			for (float z = minHeight; z < maxHeight; z += param->dHookInterpointDown)
			{
				SHI::Point3D p(hookPoint.x, hookPoint.y, z);
				hookModel->cloudDownsampled->points.push_back(p);
			}
		}
	}
	dst = hookModel;
}

void _MakeHookModel(SHI::DistanceModelPtr& dst, const SHI::CraneAttitudePtr& attitude, int partNum, const SHI::DistanceParamPtr& param)
{
	SHI::PointCloudPtr cloud(new SHI::PointCloud());
	SHI::PointCloudPtr cloudDown(new SHI::PointCloud());
	SHI::DistanceModelPtr hookModel(new SHI::DistanceModel(partNum, cloud, cloudDown));

	int numHook = attitude->numHook;
	for (unsigned int i = 0; i < numHook; i++)
	{
		SHI::StPoint hookPoint = attitude->hookPoint[i];
		float minHeight = hookPoint.z - 7.0;
		float maxHeight = attitude->hookRoi[i].roi.maxZ;

		minHeight = (std::max)(minHeight, param->minHookCollisionHeight);
		maxHeight = (std::min)(maxHeight, param->maxHookCollisionHeight);

		// Make hook model
		for (float z = minHeight; z < maxHeight; z += param->dHookInterpoint)
		{
			SHI::Point3D p(hookPoint.x, hookPoint.y, z);
			hookModel->cloud->points.push_back(p);
		}

		for (float z = minHeight; z < maxHeight; z += param->dHookInterpointDown)
		{
			SHI::Point3D p(hookPoint.x, hookPoint.y, z);
			hookModel->cloudDownsampled->points.push_back(p);
		}
	}
	dst = hookModel;
}

void CDistanceProcessor::ProcessHookCollisionDistance(SHI::DistanceInfoVectorPtr& distance, const SHI::PointCloudPtr& clusterPoints, const SHI::PCLIndicesVectorPtr& clusterIndices, const std::vector<unsigned char> &labels, const SHI::CraneAttitudePtr& attitude, const SHI::DistanceParamPtr& param)
{
	distance->clear();

	if (labels.size() == clusterIndices->size())
	{
		if (attitude->IsExact(SHI::PIERHAN, SHI::PierHan::GC1) || attitude->IsExact(SHI::PIERHAN, SHI::PierHan::GC2))
		{
			// 물체 포인트 클라우드
			SHI::PointCloudPtr objectPoints(new SHI::PointCloud);
			SHI::PointCloudPtr objectPoints2(new SHI::PointCloud);
			SHI::Filter::FilterClusterLabel(clusterPoints, *objectPoints, clusterIndices, labels, SHI::LABEL_OBJECT);
			SHI::Filter::FilterClusterLabel(clusterPoints, *objectPoints2, clusterIndices, labels, SHI::LABEL_HOOK);
			objectPoints->insert(objectPoints->end(), objectPoints2->begin(), objectPoints2->end());

			// 물체 포인트에서 크레인 포인트 추출
			if (!m_attitude->IsNull())
			{
				for (unsigned int iCrane = 0; iCrane < SHI::GetNumCrane(m_attitude->pierId); iCrane++)
				{
					int id = SHI::GetCraneId(m_attitude->pierId, iCrane);
					SHI::Data::StCraneMiniInfo otherInfo = GetCraneMiniInfo(id);
					SHI::Data::StCraneAttitude otherAttitude = otherInfo.attitude;

					if (!otherAttitude.IsNull())
					{
						SHI::PointCloud temp;
						SHI::Transform::TransformPointCloudGps(objectPoints, *objectPoints, *m_attitude);
						SHI::Transform::TransformPointCloudGpsInv(objectPoints, *objectPoints, otherAttitude);

						SHI::Filter::FilterCrane(*objectPoints, temp, objectPoints, otherAttitude);

						SHI::Transform::TransformPointCloudGps(objectPoints, *objectPoints, otherAttitude);
						SHI::Transform::TransformPointCloudGpsInv(objectPoints, *objectPoints, *m_attitude);
					}
				}
			}
			else
			{
				objectPoints->clear();
			}

			// 후크 포인트 클라우드
			SHI::DistanceModelPtr hookModelUpper;
			SHI::DistanceModelPtr hookModelLower;
			_MakeHookModelGCUpper(hookModelUpper, attitude, SHI::PierHan::GC_UPPER_TROLLY_HOOK, param);
			_MakeHookModelGCLower(hookModelLower, attitude, SHI::PierHan::GC_LOWER_TROLLY_HOOK, param);
			if (hookModelLower && hookModelUpper)
			{
				if (hookModelLower->cloudDownsampled->size() > 0 && 
					hookModelUpper->cloudDownsampled->size() > 0 &&
					objectPoints->size() > 0)
				{
					// 수평 거리 계산
					SHI::Point3D otherPosLower, myPosLower;
					SHI::Point3D otherPosUpper, myPosUpper;
					float hookDistanceLower = _DistanceHorizontalPointClouds(objectPoints, hookModelLower->cloudDownsampled, otherPosLower, myPosLower);
					float hookDistanceUpper = _DistanceHorizontalPointClouds(objectPoints, hookModelUpper->cloudDownsampled, otherPosUpper, myPosUpper);

					SHI::StROI myRoiLower(
						myPosLower.x - param->dHookInterpointDown,
						myPosLower.x + param->dHookInterpointDown,
						myPosLower.y - param->dHookInterpointDown,
						myPosLower.y + param->dHookInterpointDown,
						myPosLower.z - param->dHookInterpointDown,
						myPosLower.z + param->dHookInterpointDown);

					SHI::StROI otherRoiLower(
						otherPosLower.x - param->dHookInterpointDown,
						otherPosLower.x + param->dHookInterpointDown,
						otherPosLower.y - param->dHookInterpointDown,
						otherPosLower.y + param->dHookInterpointDown,
						otherPosLower.z - param->dHookInterpointDown,
						otherPosLower.z + param->dHookInterpointDown);

					SHI::StROI myRoiUpper(
						myPosUpper.x - param->dHookInterpointDown,
						myPosUpper.x + param->dHookInterpointDown,
						myPosUpper.y - param->dHookInterpointDown,
						myPosUpper.y + param->dHookInterpointDown,
						myPosUpper.z - param->dHookInterpointDown,
						myPosUpper.z + param->dHookInterpointDown);

					SHI::StROI otherRoiUpper(
						otherPosUpper.x - param->dHookInterpointDown,
						otherPosUpper.x + param->dHookInterpointDown,
						otherPosUpper.y - param->dHookInterpointDown,
						otherPosUpper.y + param->dHookInterpointDown,
						otherPosUpper.z - param->dHookInterpointDown,
						otherPosUpper.z + param->dHookInterpointDown);

					SHI::Filter::FilterROI(hookModelLower->cloud, *hookModelLower->cloud, myRoiLower);
					SHI::Filter::FilterROI(objectPoints, *objectPoints, otherRoiLower);

					SHI::Filter::FilterROI(hookModelUpper->cloud, *hookModelUpper->cloud, myRoiUpper);
					SHI::Filter::FilterROI(objectPoints, *objectPoints, otherRoiUpper);

					if (objectPoints->size() && hookModelLower->cloud->size() && hookModelUpper->cloud->size())
					{
						hookDistanceLower = _DistanceHorizontalPointClouds(objectPoints, hookModelLower->cloud, otherPosLower, myPosLower);
						hookDistanceUpper = _DistanceHorizontalPointClouds(objectPoints, hookModelUpper->cloud, otherPosUpper, myPosUpper);
					}

					if (1 < otherPosLower.y && otherPosLower.y < 159) // 자기 자신의 다리와는 절대로 계산 안되도록 예외 처리
					{
						// 결과 출력
						if (param->minDistance < hookDistanceLower
							&& hookDistanceLower < param->maxHookDistance)
						{
							SHI::DistanceInfo distanceInfo;
							distanceInfo.CraneIndex = SHI::PierHan::GC_LOWER_TROLLY_HOOK;
							distanceInfo.ClusterIndex = 0;
							distanceInfo.Distance = hookDistanceLower;
							distanceInfo.PosCluster.X = otherPosLower.x;
							distanceInfo.PosCluster.Y = otherPosLower.y;
							distanceInfo.PosCluster.Z = otherPosLower.z;
							distanceInfo.PosCrane.X = myPosLower.x;
							distanceInfo.PosCrane.Y = myPosLower.y;
							distanceInfo.PosCrane.Z = myPosLower.z;
							distance->push_back(distanceInfo);
						}

						if (param->minDistance < hookDistanceUpper
							&& hookDistanceUpper < param->maxHookDistance)
						{
							SHI::DistanceInfo distanceInfo;
							distanceInfo.CraneIndex = SHI::PierHan::GC_UPPER_TROLLY_HOOK;
							distanceInfo.ClusterIndex = 0;
							distanceInfo.Distance = hookDistanceUpper;
							distanceInfo.PosCluster.X = otherPosUpper.x;
							distanceInfo.PosCluster.Y = otherPosUpper.y;
							distanceInfo.PosCluster.Z = otherPosUpper.z;
							distanceInfo.PosCrane.X = myPosUpper.x;
							distanceInfo.PosCrane.Y = myPosUpper.y;
							distanceInfo.PosCrane.Z = myPosUpper.z;
							distance->push_back(distanceInfo);
						}
					}
				}
			}
		}
		else if (attitude->IsExact(SHI::PIERHAN, SHI::PierHan::TTC4))
		{
			// 물체 포인트 클라우드
			SHI::PointCloudPtr objectPoints(new SHI::PointCloud);
			SHI::Filter::FilterClusterLabel(clusterPoints, *objectPoints, clusterIndices, labels, SHI::LABEL_OBJECT);

			// 후크 포인트 클라우드
			SHI::DistanceModelPtr hookModel;
			_MakeHookModel(hookModel, attitude, SHI::PierHan::TTC_TROLLY, param);
			if (hookModel)
			{
				if (hookModel->cloudDownsampled->size() > 0 && objectPoints->size() > 0)
				{
					// 수평 거리 계산
					SHI::Point3D otherPos, myPos;
					float hookDistance = _DistanceHorizontalPointClouds(objectPoints, hookModel->cloudDownsampled, otherPos, myPos);

					SHI::StROI myRoi(
						myPos.x - param->dHookInterpointDown,
						myPos.x + param->dHookInterpointDown,
						myPos.y - param->dHookInterpointDown,
						myPos.y + param->dHookInterpointDown,
						myPos.z - param->dHookInterpointDown,
						myPos.z + param->dHookInterpointDown);

					SHI::StROI otherRoi(
						otherPos.x - param->dHookInterpointDown,
						otherPos.x + param->dHookInterpointDown,
						otherPos.y - param->dHookInterpointDown,
						otherPos.y + param->dHookInterpointDown,
						otherPos.z - param->dHookInterpointDown,
						otherPos.z + param->dHookInterpointDown);

					SHI::Filter::FilterROI(hookModel->cloud, *hookModel->cloud, myRoi);
					SHI::Filter::FilterROI(objectPoints, *objectPoints, otherRoi);

					if (objectPoints->size() && hookModel->cloud->size())
					{
						hookDistance = _DistanceHorizontalPointClouds(objectPoints, hookModel->cloud, otherPos, myPos);
					}

					// 결과 출력
					if (param->minDistance < hookDistance
						&& hookDistance < param->maxHookDistance)
					{
						SHI::DistanceInfo distanceInfo;
						distanceInfo.CraneIndex = SHI::PierHan::TTC_TROLLY;
						distanceInfo.ClusterIndex = 0;
						distanceInfo.Distance = hookDistance;
						distanceInfo.PosCluster.X = otherPos.x;
						distanceInfo.PosCluster.Y = otherPos.y;
						distanceInfo.PosCluster.Z = otherPos.z;
						distanceInfo.PosCrane.X = myPos.x;
						distanceInfo.PosCrane.Y = myPos.y;
						distanceInfo.PosCrane.Z = myPos.z;
						distance->push_back(distanceInfo);
					}

				}
			}
		}
		else
		{
			// 물체 포인트 클라우드
			SHI::PointCloudPtr objectPoints(new SHI::PointCloud);
			SHI::Filter::FilterClusterLabel(clusterPoints, *objectPoints, clusterIndices, labels, SHI::LABEL_OBJECT);

			// 후크 포인트 클라우드
			SHI::DistanceModelPtr hookModel;
			_MakeHookModel(hookModel, attitude, SHI::PierHan::GC_LOWER_TROLLY_HOOK, param);
			if (hookModel)
			{
				if (hookModel->cloudDownsampled->size() > 0 && objectPoints->size() > 0)
				{
					// 수평 거리 계산
					SHI::Point3D otherPos, myPos;
					float hookDistance = _DistanceHorizontalPointClouds(objectPoints, hookModel->cloudDownsampled, otherPos, myPos);

					SHI::StROI myRoi(
						myPos.x - param->dHookInterpointDown,
						myPos.x + param->dHookInterpointDown,
						myPos.y - param->dHookInterpointDown,
						myPos.y + param->dHookInterpointDown,
						myPos.z - param->dHookInterpointDown,
						myPos.z + param->dHookInterpointDown);

					SHI::StROI otherRoi(
						otherPos.x - param->dHookInterpointDown,
						otherPos.x + param->dHookInterpointDown,
						otherPos.y - param->dHookInterpointDown,
						otherPos.y + param->dHookInterpointDown,
						otherPos.z - param->dHookInterpointDown,
						otherPos.z + param->dHookInterpointDown);

					SHI::Filter::FilterROI(hookModel->cloud, *hookModel->cloud, myRoi);
					SHI::Filter::FilterROI(objectPoints, *objectPoints, otherRoi);

					if (objectPoints->size() && hookModel->cloud->size())
					{
						hookDistance = _DistanceHorizontalPointClouds(objectPoints, hookModel->cloud, otherPos, myPos);
					}

					// 결과 출력
					if (param->minDistance < hookDistance
						&& hookDistance < param->maxHookDistance)
					{
						SHI::DistanceInfo distanceInfo;
						distanceInfo.CraneIndex = SHI::PierHan::GC_LOWER_TROLLY_HOOK;
						distanceInfo.ClusterIndex = 0;
						distanceInfo.Distance = hookDistance;
						distanceInfo.PosCluster.X = otherPos.x;
						distanceInfo.PosCluster.Y = otherPos.y;
						distanceInfo.PosCluster.Z = otherPos.z;
						distanceInfo.PosCrane.X = myPos.x;
						distanceInfo.PosCrane.Y = myPos.y;
						distanceInfo.PosCrane.Z = myPos.z;
						distance->push_back(distanceInfo);
					}

				}
			}
		}
	}
}

