
#include "Filter.h"
#include "Transform.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

namespace SHI
{
	namespace Filter
	{
		//get inlier and outlier points from point cloud.
		void FilterROI(const SHI::PointCloudPtr pSrc, SHI::PointCloud& inlier, SHI::PointCloud& outlier, SHI::StROI roi)
		{
			SHI::IndicesPtr indices(new SHI::Indices);

			// Pass through filter로 ROI 내의 Indices 추출
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(pSrc);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(roi.roi.minX, roi.roi.maxX);
			pass.filter(*indices);

			pass.setIndices(indices);
			pass.setFilterFieldName("y");
			pass.setFilterLimits(roi.roi.minY, roi.roi.maxY);
			pass.filter(*indices);

			pass.setIndices(indices);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(roi.roi.minZ, roi.roi.maxZ);
			pass.filter(*indices);

			// Indices 포인트 추출
			pcl::ExtractIndices<pcl::PointXYZ> extraceSub;
			extraceSub.setInputCloud(pSrc);
			extraceSub.setIndices(indices);
			extraceSub.filter(inlier);

			// get outlier
			extraceSub.setNegative(true);
			extraceSub.filter(outlier);
		}
		// get inlier points from point cloud.
		void FilterROI(const SHI::PointCloudPtr pSrc, SHI::PointCloud& inlier, SHI::StROI roi)
		{
			SHI::IndicesPtr indices(new SHI::Indices);

			// Pass through filter로 ROI 내의 Indices 추출
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(pSrc);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(roi.roi.minX, roi.roi.maxX);
			pass.filter(*indices);

			pass.setIndices(indices);
			pass.setFilterFieldName("y");
			pass.setFilterLimits(roi.roi.minY, roi.roi.maxY);
			pass.filter(*indices);

			pass.setIndices(indices);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(roi.roi.minZ, roi.roi.maxZ);
			pass.filter(*indices);

			// Indices 포인트 추출
			pcl::ExtractIndices<pcl::PointXYZ> extraceSub;
			extraceSub.setInputCloud(pSrc);
			extraceSub.setIndices(indices);
			extraceSub.filter(inlier);
		}
		// get inlier points from 2d point cloud.
		void FilterROI(const SHI::PointCloud2dPtr pSrc, SHI::PointCloud2d& inlier, SHI::StROI roi)
		{
			inlier.clear();
			for (unsigned int i=0; i<pSrc->size(); i++)
			{
				Point2D& p = pSrc->at(i);
				if (roi.roi.minX <= p.x && p.x <= roi.roi.minX &&
					roi.roi.minY <= p.y && p.y <= roi.roi.minY)
				{
					inlier.push_back(p);
				}
			}
		}
		// get 
		void FilterROI(const SHI::PointCloudPtr &pSrc, SHI::PointCloud& inlier, SHI::PointCloud& outlier, SHI::StTriangularROI roi, const SHI::CraneAttitudePtr &attitude)
		{
			SHI::PointCloudPtr _inlier(new SHI::PointCloud);
			SHI::PointCloudPtr _outlier(new SHI::PointCloud);
			if (pSrc && attitude)
			{
				SHI::Point3D high = roi.GetHighPos();
				SHI::Point3D low = roi.GetLowPos();
				SHI::Point3D pole = roi.GetPolePos(attitude);

				// 후보 포인트 추출
				SHI::StROI candidateRoi(roi.roi.minX, roi.roi.maxX, (std::min)(high.y, low.y), pole.y, (std::min)(low.z, pole.z), (std::max)(high.z, pole.z));
				SHI::PointCloudPtr candidates(new SHI::PointCloud);
				// get new candidate points and outlier points from candidate roi points.
				FilterROI(pSrc, *candidates, *_outlier, candidateRoi);

				// ROI 처리 : 전체를 비교하지 않고 후보 포인트들 만 imlier인지 비교하여 ROI 내의 포인트 추출.
				for (unsigned int i = 0; i < candidates->size(); i++)
				{
					SHI::Point3D p = candidates->at(i);
					if (roi.IsInlier(p, attitude))
					{
						_inlier->push_back(p);
					}
					else
					{
						_outlier->push_back(p);
					}
				}
			}
			inlier.swap(*_inlier);
			outlier.swap(*_outlier);
		}
		// src에서 attitude를 적용하여 inlier는 dstCrane에 outlier는 dstOthers에 저장.
		void FilterCrane(SHI::PointCloud& dstCrane, SHI::PointCloud& dstOthers, SHI::PointCloudPtr src, const SHI::CraneAttitude & attitude)
		{
			dstCrane.clear();
			dstOthers.clear();

			SHI::PointCloudPtr input = src;
			SHI::PointCloudPtr inputTrans(new SHI::PointCloud);
			SHI::PointCloudPtr cloudTmpIn(new SHI::PointCloud);
			SHI::PointCloudPtr cloudTmpOut(new SHI::PointCloud);
			for (unsigned int i = 0; i < attitude.numPart; i++)
			{
				// 해당 파트 좌표계 역변환
				SHI::Transform::TransformPointCloudInv(input, *inputTrans, attitude, i);

				// 파트 ROI 처리
				SHI::Filter::FilterROI(inputTrans, *cloudTmpIn, *cloudTmpOut, attitude.craneRoi[i]);
				
				// 해당 파트 좌표계 변환
				SHI::Transform::TransformPointCloud(cloudTmpIn, *cloudTmpIn, attitude, i);
				SHI::Transform::TransformPointCloud(cloudTmpOut, *cloudTmpOut, attitude, i);

				// inlier는 크레인 포인트에 추가
				dstCrane.insert(dstCrane.begin(), cloudTmpIn->begin(), cloudTmpIn->end());

				// outlier는 다음 입력으로 사용
				input = cloudTmpOut;
			}

			// 최종 outlier 반환
			dstOthers.insert(dstOthers.begin(), cloudTmpOut->begin(), cloudTmpOut->end());
		}

		void FilterIndices(SHI::PointCloudPtr pSrc, SHI::PointCloud& inlier, const SHI::IndicesPtr& indices)
		{
			pcl::ExtractIndices<pcl::PointXYZ> extractor;
			extractor.setInputCloud(pSrc);
			extractor.setIndices(indices);
			extractor.filter(inlier);
		}

		void FilterIndices(SHI::PointCloudPtr pSrc, SHI::PointCloud& inlier, const std::vector<int>& indices)
		{
			SHI::IndicesPtr _indices(new SHI::Indices);
			*_indices = indices;
			FilterIndices(pSrc, inlier, _indices);
		}

		void FilterClusterLabel(SHI::PointCloudPtr pSrc, SHI::PointCloud& inlier, const SHI::PCLIndicesVectorPtr& clusterIndices, const std::vector<unsigned char> &labels, int label)
		{
			SHI::PointCloudPtr _inlier(new SHI::PointCloud);
			for (unsigned int i = 0; i < labels.size(); i++)
			{
				if (labels.at(i) == label)
				{
					SHI::PointCloudPtr tmp(new SHI::PointCloud);
					SHI::Filter::FilterIndices(pSrc, *tmp, clusterIndices->at(i).indices);
					*_inlier += *tmp;
				}
			}
			inlier = *_inlier;
		}
	}
}