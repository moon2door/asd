#include "../ClusterProcessor.h"

#include <Utility/Filter.h>
#include <Utility/Transform.h>
#include <Config/CraneInfo.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

bool CClusterProcessor::ProcessRemoveCranePoints(SHI::PointCloud& dst, SHI::PointCloud& dstOutlier, SHI::PointCloud& dstExceptionRoi, SHI::PointCloudPtr cloudInput)
{
	bool ret = false;

	// 파라미터 입력
	SHI::CraneAttitudePtr attitude = GetCraneAttitude();
	std::shared_ptr<std::vector<SHI::PointCloudPtr>> vRefPartPoints = GetRefPartPoints();
	SHI::ClusterParamPtr param = GetClusterParam();

	if(attitude && vRefPartPoints && param)
	{
		if (vRefPartPoints->size() == attitude->numPart)
		{
			SHI::PointCloudPtr cloudCandidate(new SHI::PointCloud);
			SHI::PointCloudPtr cloudCrane(new SHI::PointCloud);
			SHI::PointCloudPtr cloudExceptionRoi(new SHI::PointCloud);
			SHI::PointCloudPtr cloudOthers(new SHI::PointCloud);

			// 크레인 포인트 ROI 처리: 
			SHI::Filter::FilterCrane(*cloudCandidate, *cloudOthers, cloudInput, *attitude);

			// 누적 모델 생성 업데이트
			ProcessAccumCloudModel(cloudCandidate);

			// 모델 포인트 생성
			SHI::PointCloudPtr cloudModel(new SHI::PointCloud);
			for (unsigned int i = 0; i < vRefPartPoints->size(); i++)
			{
				SHI::PointCloudPtr cloudTmp(new SHI::PointCloud);
				// vRefPartPoints->at(i)의 포인트를 attitude->jointInfo[i]와 attitude->pose[i]를 이용하여 변환하여 cloudTmp에 저장.
				SHI::Transform::TransformPointCloud(vRefPartPoints->at(i), *cloudTmp, attitude->jointInfo[i], attitude->pose[i]);
				// 
				*cloudModel += *cloudTmp;
			}

			// 크레인 필터 처리
			// 임계치 이상 떨어진 포인트만 분리하여 cloudOthers에 저장
			float threshold = param->clusterTolerance2;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloudModel);
			for (unsigned int i = 0; i < cloudCandidate->size(); i++)
			{
				pcl::PointXYZ p = cloudCandidate->at(i);
				std::vector<int> indices;
				std::vector<float> distancesSq;

				// 크레인 포인트 후보 탐색
				bool bExist = false;
				indices.resize(1);
				distancesSq.resize(1);
				int numFound = tree->nearestKSearch(p, 1, indices, distancesSq);
				if (numFound > 0)
				{
					for (unsigned int iFound = 0; iFound < numFound; iFound++)
					{
						if (distancesSq[iFound] < threshold)
						{
							bExist = true;
							break;
						}
					}
				}

				// 근거리 포인트가 존재하지 않는 포인트만 분리
				if (bExist)
				{
					cloudCrane->push_back(p);
				}
				else
				{
					cloudOthers->push_back(p);
				}
			}

			if (param->roiException)
			{
				// ROI 처리: exception의 roi 즉 예외 영역의 inlier, cloudTmp를 cloudExceptionRoi에 저장
				for (unsigned int i = 0; i < param->roiException->vRoi.size(); i++)
				{
					SHI::PointCloudPtr cloudTmp(new SHI::PointCloud);
					SHI::Filter::FilterROI(cloudOthers, *cloudTmp, *cloudOthers, param->roiException->vRoi[i]);
					cloudExceptionRoi->insert(cloudExceptionRoi->end(), cloudTmp->begin(), cloudTmp->end());
				}
				// 파트 ROI 처리: 각 파트의 예외 영역의 inlier, cloudInlier를 cloudExceptionRoi에 저장
				for (unsigned int part = 0; part < attitude->numPart; part++)
				{
					// 회전/이동 적용
					SHI::PointCloudPtr cloudOthersTrans(new SHI::PointCloud);
					SHI::Transform::TransformPointCloudInv(cloudOthers, *cloudOthersTrans, *attitude, part);


					SHI::PointCloudPtr cloudInlier(new SHI::PointCloud);
					for (unsigned int iFilter = 0; iFilter < param->roiException->part[part].size(); iFilter++)
					{
						// ROI 처리
						SHI::PointCloudPtr cloudTmp(new SHI::PointCloud);
						SHI::Filter::FilterROI(cloudOthersTrans, *cloudTmp, *cloudOthersTrans, param->roiException->part[part].at(iFilter));
						cloudInlier->insert(cloudInlier->end(), cloudTmp->begin(), cloudTmp->end());
					}

					// 회전/이동 역변환
					SHI::Transform::TransformPointCloud(cloudOthersTrans, *cloudOthersTrans, *attitude, part);
					SHI::Transform::TransformPointCloud(cloudInlier, *cloudInlier, *attitude, part);

					// 분류 결과
					cloudExceptionRoi->insert(cloudExceptionRoi->end(), cloudInlier->begin(), cloudInlier->end());
					cloudOthers = cloudOthersTrans;
				}

				// 삼각 ROI 처리
				for (unsigned int i = 0; i < param->roiException->triangular.size(); i++)
				{
					SHI::PointCloudPtr cloudTmp(new SHI::PointCloud);
					SHI::Filter::FilterROI(cloudOthers, *cloudTmp, *cloudOthers, param->roiException->triangular[i], attitude);
					cloudExceptionRoi->insert(cloudExceptionRoi->end(), cloudTmp->begin(), cloudTmp->end());
				}
			}
			dst = *cloudOthers;
			dstOutlier = *cloudCrane;
			dstExceptionRoi = *cloudExceptionRoi;
			ret = true;
		}
	}
	return ret;
}
