#include "../DistanceProcessor.h"

#include <Utility/Transform.h>
#include <Utility/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <Utility/Types.h>

#include <Routine/include/Base/CElapseTimer.h>

float_t DistancePointClouds(const SHI::PointCloudPtr &cloudCompared, const SHI::PointCloudPtr &cloudRef, SHI::Point3D& compared, SHI::Point3D& ref)
{
	float_t minDistance = FLT_MAX;
	SHI::Point3D _ref, _compared;

	// make kdtree
	SHI::KdTree tree;
	tree.setInputCloud(cloudRef);

	// find nearest point pairs
	for (uint32_t i=0; i<cloudCompared->size(); i++)
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

bool CDistanceProcessor::ProcessDistance(SHI::DistanceInfoVectorPtr& distance, const SHI::PointCloudPtr& clusterPoints, const SHI::PCLIndicesVectorPtr& clusterIndices, const std::vector<unsigned char> &labels, const SHI::DistanceModelVectorPtr& models, const SHI::CraneAttitudePtr& attitude, const SHI::DistanceParamPtr& param)
{
	bool ret = false;

	if (distance && clusterPoints && clusterIndices && attitude && param)
	{
		float_t downsampleSize = 2.0;
		float_t seedLeafSize = 10.0;
		float_t maxDistance = param->maxDistance;
		float_t minDistance = param->minDistance;
		float_t iterationThreshold = 0.1;
		uint32_t maxIteration = 100;
		float_t searchSize = 8.0;

		SHI::StROI searchBoundary(-searchSize, searchSize, -searchSize, searchSize, -searchSize, searchSize);
		SHI::StROI modelDownBoundary(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0);
		SHI::StROI clusterDownBoundary(-downsampleSize, downsampleSize, -downsampleSize, downsampleSize, -downsampleSize, downsampleSize);

		if (labels.size() == clusterIndices->size())
		{
			distance->clear();

			// set attitude to model
			SHI::DistanceModelVectorPtr vModels(new SHI::DistanceModelVector);
			for (uint32_t i = 0; i < models->size(); i++)
			{
				SHI::DistanceModel _model(models->at(i).part, SHI::PointCloudPtr(new SHI::PointCloud), SHI::PointCloudPtr(new SHI::PointCloud));
				SHI::Transform::TransformPointCloud(models->at(i).cloud, *_model.cloud, attitude->jointInfo[_model.part], attitude->pose[_model.part]);
				SHI::Transform::TransformPointCloud(models->at(i).cloudDownsampled, *_model.cloudDownsampled, attitude->jointInfo[_model.part], attitude->pose[_model.part]);
				_model.tree->setInputCloud(_model.cloudDownsampled);
				vModels->push_back(_model);
			}

			// gather target indices list
			pcl::PointIndices::Ptr indices(new pcl::PointIndices);
			for (uint32_t i=0; i<clusterIndices->size(); i++)
			{
				if (labels.at(i) == SHI::LABEL_OBJECT)
					indices->indices.insert(indices->indices.end(), clusterIndices->at(i).indices.begin(), clusterIndices->at(i).indices.end());
			}

			// extract target points
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(clusterPoints);
			extract.setIndices(indices);
			extract.filter(*cloudCluster);

			// downsample cluster
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointDownsampled(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::VoxelGrid<pcl::PointXYZ> vg;
			vg.setInputCloud(clusterPoints);
			vg.setIndices(indices);
			vg.setLeafSize(downsampleSize, downsampleSize, downsampleSize);
			vg.filter(*pointDownsampled);
			
			// initial seed points
			pcl::PointCloud<pcl::PointXYZ>::Ptr pointSeed(new pcl::PointCloud<pcl::PointXYZ>);
			vg.setLeafSize(seedLeafSize, seedLeafSize, seedLeafSize);
			vg.filter(*pointSeed);

			for (uint32_t iModel = 0; iModel < vModels->size(); iModel++)
			{
				SHI::DistanceInfoVectorPtr vDistanceTmp(new SHI::DistanceInfoVector);
				SHI::KdTreePtr tree = vModels->at(iModel).tree;

				// iterative search for each seed points
				for (uint32_t iSeed = 0; iSeed < pointSeed->size(); iSeed++)
				{
					Routine::CElapseTimer e;
					// initial seed point
					pcl::PointXYZ seed = pointSeed->points[iSeed];

					// check distance seed to model
					std::vector<int32_t> _pointIdxNKNSearch;
					std::vector<float_t> _pointNKNSquaredDistance;
					float_t _distanceSq = FLT_MAX;
					if (tree->nearestKSearch(seed, 1, _pointIdxNKNSearch, _pointNKNSquaredDistance) > 0)
					{
						_distanceSq = _pointNKNSquaredDistance[0];
					}
					if (_distanceSq > maxDistance * maxDistance) continue;

					// search nearest point roughly
					float_t nearestDistanceSq = FLT_MAX;
					pcl::PointXYZ posCluster, posCrane;
					uint32_t count = 0;
					while (count < maxIteration)
					{
						// seed roi
						pcl::PointCloud<pcl::PointXYZ>::Ptr pointRoi(new pcl::PointCloud<pcl::PointXYZ>);
						SHI::StROI roi = searchBoundary + seed;
						SHI::Filter::FilterROI(pointDownsampled, *pointRoi, roi);

						// find next seed point
						for (uint32_t i = 0; i < pointRoi->size(); i++)
						{
							std::vector<int32_t> pointIdxNKNSearch;
							std::vector<float_t> pointNKNSquaredDistance;
							pcl::PointXYZ searchPoint = pointRoi->points[i];
							if (tree->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
							{
								float_t distanceSq = pointNKNSquaredDistance[0];
								int32_t index = pointIdxNKNSearch[0];
								if (distanceSq < nearestDistanceSq)
								{
									nearestDistanceSq = distanceSq;
									posCluster = searchPoint;
									posCrane = tree->getInputCloud()->at(index);
								}
							}
						}

						// check execution condition
						float_t diff = squaredEuclideanDistance(seed, posCluster);
						seed = posCluster;
						count++;
						if (diff < iterationThreshold*iterationThreshold)
						{
							break;
						}
					}

					// queue final seed point
					float_t nearestDistance = sqrt(nearestDistanceSq);
					if (nearestDistance < maxDistance)
					{
						// search nearest point exactly
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModel(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudObject(new pcl::PointCloud<pcl::PointXYZ>);
						SHI::Filter::FilterROI(vModels->at(iModel).cloud, *cloudModel, modelDownBoundary + posCrane);
						SHI::Filter::FilterROI(cloudCluster, *cloudObject, clusterDownBoundary + posCluster);
						pcl::PointXYZ _posCluster, _posCrane;
						float_t distanceExact = DistancePointClouds(cloudObject, cloudModel, _posCluster, _posCrane);

						posCluster = _posCluster;
						posCrane = _posCrane;
						nearestDistance = distanceExact;

						// output
						SHI::DistanceInfo d;
						d.PosCluster.X = posCluster.x;
						d.PosCluster.Y = posCluster.y;
						d.PosCluster.Z = posCluster.z;
						d.PosCrane.X = posCrane.x;
						d.PosCrane.Y = posCrane.y;
						d.PosCrane.Z = posCrane.z;
						d.ClusterIndex = 0;
						d.CraneIndex = vModels->at(iModel).part;
						d.Distance = nearestDistance;
						vDistanceTmp->push_back(d);
					}
				}

				// 중복 체크
				std::vector<float_t> vList(vDistanceTmp->size());
				for (uint32_t i = 0; i < vDistanceTmp->size(); i++)
				{
					// 중복 리스트 조회
					bool bOnList = false;
					for (uint32_t j = 0; j < vList.size() && bOnList == false; j++)
					{
						if (abs(vList[j] - vDistanceTmp->at(i).Distance) < iterationThreshold) bOnList = true;
					}

					// 중복되지 않는 거리만 반환함
					if (bOnList == false)
					{
						vList.push_back(vDistanceTmp->at(i).Distance);
						distance->push_back(vDistanceTmp->at(i));
					}
				}
			}

		}
	}
	return ret;
}