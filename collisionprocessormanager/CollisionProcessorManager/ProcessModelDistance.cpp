#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"

#include <Utility/UTM.h>
#include <Utility/Transform.h>
#include <Utility/filter.h>
#include <Utility/types.h>
#include <Data/StDistanceInfo.h>

float_t _DistancePointClouds(const SHI::PointCloudPtr &cloudCompared, const SHI::PointCloudPtr &cloudRef, SHI::Point3D& compared, SHI::Point3D& ref);
float_t _DistanceHorizontalPointClouds(const SHI::PointCloudPtr &cloudCompared, const SHI::PointCloudPtr &cloudRef, SHI::Point3D& compared, SHI::Point3D& ref);
void _MakeHookModel(SHI::DistanceModelPtr& dst, const SHI::CraneAttitudePtr& attitude, int32_t partNum, float_t interpointDistance);

bool CCollisionProcessorManagerDlg::ProcessModelDistance(SHI::Data::StModelDistanceSocket& distance, const SHI::CraneAttitudePtr& myAttitude, const SHI::CraneAttitudePtr& otherAttitude)
{
	Routine::CElapseTimer e;
	bool ret = false;

	if (myAttitude)
	{
		EnsureDistanceResources(myAttitude->pierId, myAttitude->craneId);
	}
	if (otherAttitude)
	{
		EnsureDistanceResources(otherAttitude->pierId, otherAttitude->craneId);
	}

	// Load Parameters
	SHI::DistanceModelVectorPtr myModelRef;
	if (m_refPartModelsAll.find(myAttitude->craneId) != m_refPartModelsAll.end())
	{
		myModelRef = m_refPartModelsAll[myAttitude->craneId];
	}

	SHI::DistanceModelVectorPtr otherModelRef;
	if (m_refPartModelsAll.find(otherAttitude->craneId) != m_refPartModelsAll.end())
	{
		otherModelRef = m_refPartModelsAll[otherAttitude->craneId];
	}

	SHI::DistanceParamPtr myParam;
	if (m_distanceParamAll.find(myAttitude->craneId) != m_distanceParamAll.end())
	{
		myParam = m_distanceParamAll[myAttitude->craneId];
	}

	SHI::DistanceParamPtr otherParam;
	if (m_distanceParamAll.find(otherAttitude->craneId) != m_distanceParamAll.end())
	{
		otherParam = m_distanceParamAll[otherAttitude->craneId];
	}

	if (myAttitude && otherAttitude && myModelRef && otherModelRef && myParam && otherParam)
	{
		// Set attitude to my model, downsampled model
		SHI::PointCloudPtr myModel(new SHI::PointCloud);
		SHI::PointCloudPtr myModelDown(new SHI::PointCloud);
		for (uint32_t i = 0; i < myModelRef->size(); i++)
		{
			int32_t iPart = myModelRef->at(i).part;
			SHI::PointCloudPtr _myModel(new SHI::PointCloud);
			SHI::PointCloudPtr _myModelDown(new SHI::PointCloud);
			SHI::Transform::TransformPointCloud(myModelRef->at(i).cloud, *_myModel, myAttitude->jointInfo[iPart], myAttitude->pose[iPart]);
			SHI::Transform::TransformPointCloud(myModelRef->at(i).cloudDownsampled, *_myModelDown, myAttitude->jointInfo[iPart], myAttitude->pose[iPart]);

			myModel->insert(myModel->end(), _myModel->begin(), _myModel->end());
			myModelDown->insert(myModelDown->end(), _myModelDown->begin(), _myModelDown->end());
		}

		// Set attitude to other model, downsampled model
		SHI::PointCloudPtr otherModel(new SHI::PointCloud);
		SHI::PointCloudPtr otherModelDown(new SHI::PointCloud);
		for (uint32_t i = 0; i < otherModelRef->size(); i++)
		{
			int32_t iPart = otherModelRef->at(i).part;
			SHI::PointCloudPtr _otherModel(new SHI::PointCloud);
			SHI::PointCloudPtr _otherModelDown(new SHI::PointCloud);
			SHI::Transform::TransformPointCloud(otherModelRef->at(i).cloud, *_otherModel, otherAttitude->jointInfo[iPart], otherAttitude->pose[iPart]);
			SHI::Transform::TransformPointCloud(otherModelRef->at(i).cloudDownsampled, *_otherModelDown, otherAttitude->jointInfo[iPart], otherAttitude->pose[iPart]);
			SHI::Transform::TransformPointCloudGps(_otherModel, *_otherModel, *otherAttitude);
			SHI::Transform::TransformPointCloudGps(_otherModelDown, *_otherModelDown, *otherAttitude);
			SHI::Transform::TransformPointCloudGpsInv(_otherModel, *_otherModel, *myAttitude);
			SHI::Transform::TransformPointCloudGpsInv(_otherModelDown, *_otherModelDown, *myAttitude);

			otherModel->insert(otherModel->end(), _otherModel->begin(), _otherModel->end());
			otherModelDown->insert(otherModelDown->end(), _otherModelDown->begin(), _otherModelDown->end());
		}

		if (otherParam->bProcessHookRoiDistance)
		{
			// Make other hook model
			SHI::DistanceModelPtr otherHook;
			_MakeHookModel(otherHook, otherAttitude, 10, 0.5);
			SHI::Transform::TransformPointCloudGps(otherHook->cloud, *otherHook->cloud, *otherAttitude);
			SHI::Transform::TransformPointCloudGps(otherHook->cloudDownsampled, *otherHook->cloudDownsampled, *otherAttitude);
			SHI::Transform::TransformPointCloudGpsInv(otherHook->cloud, *otherHook->cloud, *myAttitude);
			SHI::Transform::TransformPointCloudGpsInv(otherHook->cloudDownsampled, *otherHook->cloudDownsampled, *myAttitude);
			otherModel->insert(otherModel->end(), otherHook->cloud->begin(), otherHook->cloud->end());
			otherModelDown->insert(otherModelDown->end(), otherHook->cloudDownsampled->begin(), otherHook->cloudDownsampled->end());
		}

		// Calc hook distance
		SHI::Point3D otherHookPos, myHookPos;
		float_t hookDistance = FLT_MAX;
		if (myParam->bProcessHookRoiDistance)
		{
			// Make my hook model
			SHI::DistanceModelPtr myHook;
			_MakeHookModel(myHook, myAttitude, 10, 0.3);

			// Calc downsampled hook distance
			hookDistance = _DistanceHorizontalPointClouds(otherModelDown, myHook->cloudDownsampled, otherHookPos, myHookPos);

			SHI::StROI myRoiHook(
				myHookPos.x - 5.0,
				myHookPos.x + 5.0,
				myHookPos.y - 5.0,
				myHookPos.y + 5.0,
				myHookPos.z - 5.0,
				myHookPos.z + 5.0);

			SHI::StROI otherRoiHook(
				otherHookPos.x - 5.0,
				otherHookPos.x + 5.0,
				otherHookPos.y - 5.0,
				otherHookPos.y + 5.0,
				otherHookPos.z - 5.0,
				otherHookPos.z + 5.0);

			SHI::PointCloudPtr otherTemp(new SHI::PointCloud);
			SHI::Filter::FilterROI(myHook->cloud, *myHook->cloud, myRoiHook);
			SHI::Filter::FilterROI(otherModel, *otherTemp, otherRoiHook);

			if (otherTemp->size() && myHook->cloud->size())
			{
				hookDistance = _DistanceHorizontalPointClouds(otherTemp, myHook->cloud, otherHookPos, myHookPos);
			}
		}

		// Calc downsampled model distance
		SHI::Point3D otherPos, myPos;
		float_t modelDistance = FLT_MAX;
		if (myModelDown->points.size() > 0 && otherModelDown->points.size() > 0)
		{
			modelDistance = _DistancePointClouds(myModelDown, otherModelDown, myPos, otherPos);
		}

		// model distance roi
		SHI::StROI myRoi(
			myPos.x - 10,
			myPos.x + 10,
			myPos.y - 10,
			myPos.y + 10,
			myPos.z - 10,
			myPos.z + 10);

		SHI::StROI otherRoi(
			otherPos.x - 10,
			otherPos.x + 10,
			otherPos.y - 10,
			otherPos.y + 10,
			otherPos.z - 10,
			otherPos.z + 10);

		SHI::Filter::FilterROI(myModel, *myModel, myRoi);
		SHI::Filter::FilterROI(otherModel, *otherModel, otherRoi);

		// Calc model distance
		if (myModel->points.size() > 0 && otherModel->points.size() > 0)
		{
			modelDistance = _DistancePointClouds(myModel, otherModel, myPos, otherPos);
		}

		// min distance
		float_t d = modelDistance;
		if (modelDistance > hookDistance)
		{
			d = hookDistance;
			myPos = myHookPos;
			otherPos = otherHookPos;
		}

		// Push result
		if (0 < d && d < 500)
		{
			SHI::Data::StDistanceInfo info;
			info.CraneIndex = 100;
			info.ClusterIndex = 100;
			info.Distance = d;
			info.PosCluster.X = otherPos.x;
			info.PosCluster.Y = otherPos.y;
			info.PosCluster.Z = otherPos.z;
			info.PosCrane.X = myPos.x;
			info.PosCrane.Y = myPos.y;
			info.PosCrane.Z = myPos.z;

			distance.distanceInfo = info;
			memcpy(&distance.myAttitude, myAttitude.get(), sizeof(SHI::Data::StCraneAttitude));
			memcpy(&distance.otherAttitude, otherAttitude.get(), sizeof(SHI::Data::StCraneAttitude));
			ret = true;
		}
	}

	printf("Hook distance %.1lfm, elapsed %d ms\n", distance.distanceInfo.Distance, e.GetElapseTime() / 1000);
	return ret;
}


float_t _DistancePointClouds(const SHI::PointCloudPtr &cloudCompared, const SHI::PointCloudPtr &cloudRef, SHI::Point3D& compared, SHI::Point3D& ref)
{
	float_t minDistance = FLT_MAX;
	SHI::Point3D _ref, _compared;

	// make kdtree
	SHI::KdTree tree;
	tree.setInputCloud(cloudRef);

	// find nearest point pairs
	for (uint32_t i = 0; i < cloudCompared->size(); i++)
	{
		std::vector<int32_t> pointIdxNKNSearch;
		std::vector<float_t> pointNKNSquaredDistance;
		pcl::PointXYZ searchPoint = cloudCompared->points[i];
		if (tree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			float_t distanceSq = pointNKNSquaredDistance[0];
			int32_t index = pointIdxNKNSearch[0];
			if (distanceSq < minDistance)
			{
				minDistance = distanceSq;
				_compared = searchPoint;
				_ref = tree.getInputCloud()->at(index);
			}
		}
	}

	// output
	compared = _compared;
	ref = _ref;
	return sqrt(minDistance);
}

float_t _DistanceHorizontalPointClouds(const SHI::PointCloudPtr &cloudCompared, const SHI::PointCloudPtr &cloudRef, SHI::Point3D& compared, SHI::Point3D& ref)
{
	float_t eps = 0.5;
	float_t maxDistance = 50;
	float_t minDistance = FLT_MAX;
	SHI::Point3D _ref = SHI::Point3D();
	SHI::Point3D _compared = SHI::Point3D();

	for (uint32_t idxRef = 0; idxRef < cloudRef->size(); idxRef++)
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
		for (uint32_t i = 0; i < indices->size(); i++)
		{
			int32_t idx = indices->data()[i];
			float_t distanceSq =
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

void _MakeHookModel(SHI::DistanceModelPtr& dst, const SHI::CraneAttitudePtr& attitude, int32_t partNum, float_t interpointDistance)
{
	SHI::PointCloudPtr cloud(new SHI::PointCloud());
	SHI::PointCloudPtr cloudDown(new SHI::PointCloud());
	SHI::DistanceModelPtr hookModel(new SHI::DistanceModel(partNum, cloud, cloudDown));

	int32_t numHook = attitude->numHook;
	for (uint32_t i = 0; i < numHook; i++)
	{
		SHI::StPoint hookPoint = attitude->hookPoint[i];
		float_t maxHeight = attitude->hookRoi[i].roi.maxZ;

		// Make hook model
		for (float_t z = hookPoint.z; z < maxHeight; z += interpointDistance)
		{
			SHI::Point3D p(hookPoint.x, hookPoint.y, z);
			hookModel->cloud->points.push_back(p);
			hookModel->cloudDownsampled->points.push_back(p);
		}
	}
	dst = hookModel;
}
