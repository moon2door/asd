
#include "EstimateLLCAzimuth.h"
#include "../Utility/Filter.h"
#include "../Utility/Transform.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/angles.h>
#include <stdio.h>

////pjh
//#ifdef _DEBUG
//#include <pcl/visualization/pcl_visualizer.h>
//#endif
////

namespace SHI
{
	namespace Recognition
	{
//		//pjh
//#ifdef _DEBUG
//		pcl::visualization::PCLVisualizer::Ptr mm_viewer;
//#endif
//		//

		CEstimateLLCAzimuth::CEstimateLLCAzimuth()
			: CAbstractEstimator(0.1, 0.7, 0.001, 0.8, 0.2, 5, true)
		{

		}

		CEstimateLLCAzimuth::~CEstimateLLCAzimuth()
		{

		}

		void CEstimateLLCAzimuth::Initialize(float_t dAngle, const StJointInfo& joint, StROI roi, const std::vector<SHI::StROI>& vInliers, int32_t minCount)
		{
			m_joint = joint;
			m_dtheta = dAngle;
			m_minCount = minCount;

			// 포인트 roi는 조인트 기준 좌표
			// 내부에서 ROI처리를 원점 기준 좌표에서 처리함
			// 따라서 원점 기준 ROI로 변환
			m_roi.roi.minX = roi.roi.minX + joint.cx;
			m_roi.roi.maxX = roi.roi.maxX + joint.cx;
			m_roi.roi.minY = roi.roi.minY + joint.cy;
			m_roi.roi.maxY = roi.roi.maxY + joint.cy;
			m_roi.roi.minZ = roi.roi.minZ + joint.cz;
			m_roi.roi.maxZ = roi.roi.maxZ + joint.cz;

			// 나머지 roi는 원점 기준 좌표
			// 내부에서 조인트 기준으로 처리함
			// 따라서 조인트 기준 ROI로 변환
			m_vInliers.clear();
			for (uint32_t i=0; i<vInliers.size(); i++)
			{
				SHI::StROI roi;
				roi.roi.minX = vInliers.at(i).roi.minX - joint.cx;
				roi.roi.maxX = vInliers.at(i).roi.maxX - joint.cx;
				roi.roi.minY = vInliers.at(i).roi.minY - joint.cy;
				roi.roi.maxY = vInliers.at(i).roi.maxY - joint.cy;
				roi.roi.minZ = vInliers.at(i).roi.minZ - joint.cz;
				roi.roi.maxZ = vInliers.at(i).roi.maxZ - joint.cz;
				m_vInliers.push_back(roi);
			}
		}
		
		bool CEstimateLLCAzimuth::Run(PointCloudPtr cloud)
		{
			bool ret = false;

			// ROI 처리
			PointCloudPtr cloudRoi(new PointCloud);
			SHI::Filter::FilterROI(cloud, *cloudRoi, m_roi);

			// 오프셋 변환
			// 원점 좌표계에서 조인트 기준 좌표계로 변환
			PointCloudPtr cloudOrigin(new PointCloud);
			for (uint32_t i = 0; i < cloudRoi->size(); i++)
			{
				Point3D point = cloudRoi->points.at(i);
				point.x = point.x - m_joint.cx;
				point.y = point.y - m_joint.cy;
				point.z = point.z - m_joint.cz;
				cloudOrigin->push_back(point);
			}

//			//pjh
//#ifdef _DEBUG
//			mm_viewer.reset(new pcl::visualization::PCLVisualizer("Before ROI viewer"));
//			mm_viewer->addPointCloud(cloud);
//
//			while (!mm_viewer->wasStopped())
//			{
//				mm_viewer->spinOnce();
//			}
//			mm_viewer = nullptr;
//
//			mm_viewer.reset(new pcl::visualization::PCLVisualizer("After ROI viewer"));
//			mm_viewer->addPointCloud(cloudRoi);
//			while (!mm_viewer->wasStopped())
//			{
//				mm_viewer->spinOnce();
//			}
//			mm_viewer = nullptr;
//#endif
//			//

			// 각 각도별 평가값 계산 후 최대값 추출
			std::shared_ptr<std::vector< PointCloudPtr >> vPointsMax(new std::vector< PointCloudPtr >);
			uint32_t maxCount = 0;
			float_t maxTheta = 0;
			for (float_t f = 0; f<360; f += m_dtheta)
			{
				std::shared_ptr<std::vector< PointCloudPtr >> tmp(new std::vector< PointCloudPtr >);
				float_t s = sin(pcl::deg2rad(f));
				float_t c = cos(pcl::deg2rad(f));

				// 각 각도만큼 포인트(cloudOrigin)를 회전하여 pointsRot에 저장
				PointCloudPtr pointsRot(new PointCloud);
				pointsRot->reserve(cloudOrigin->size());
				for (uint32_t i = 0; i < cloudOrigin->size(); i++)
				{
					Point3D p;
					p.x = cloudOrigin->at(i).x * c - cloudOrigin->at(i).y * s;
					p.y = cloudOrigin->at(i).x * s + cloudOrigin->at(i).y * c;
					p.z = cloudOrigin->at(i).z;
					pointsRot->push_back(p);
				}

				// ROI별 평가값 합산
				uint32_t count = 0;
				for (uint32_t iRoi = 0; iRoi < m_vInliers.size(); iRoi++)
				{
					// ROI 포인트 추출
					PointCloudPtr pointsRoi(new PointCloud);
					SHI::Filter::FilterROI(pointsRot, *pointsRoi, m_vInliers[iRoi]);

					count += pointsRoi->size();
					tmp->push_back(pointsRoi);
				}

				// 최대값 비교
				if (maxCount <= count)
				{
					vPointsMax = tmp;
					maxCount = count;
					maxTheta = f;
				}
			}

			// 결과 반환
			if (m_minCount < maxCount)
			{
				ret = true;
				maxTheta = 180.0f-maxTheta;
				while (maxTheta > 360.0f) maxTheta = maxTheta - 360.0f;
				while (maxTheta < 0.0f) maxTheta = maxTheta + 360.0f;
				UpdateResult(maxTheta);
			}
			m_cloudOrigin = cloudOrigin;
			m_vPointsMax = vPointsMax;
			return ret;
		}

		inline int32_t GetPosX(float_t px, float_t ratio, int32_t cx) { return cx + int32_t(px * ratio); }
		inline int32_t GetPosY(float_t py, float_t ratio, int32_t cy) { return cy - int32_t(py * ratio); }

		bool CEstimateLLCAzimuth::GetResultImage(Routine::Graphics::CImage& dst)
		{
			bool ret = false;
			const Routine::Graphics::CImageColor red(255, 20, 20);
			const Routine::Graphics::CImageColor black(0, 0, 0);

			const float_t ratio = 50;
			float_t w = (m_roi.roi.maxX - m_roi.roi.minX);
			float_t h = (m_roi.roi.maxY - m_roi.roi.minY);
			int32_t width = (int32_t)w*ratio;
			int32_t height = (int32_t)h*ratio;
			int32_t cy = height / 2;
			int32_t cx = width / 2;

			if (width > 0 && height > 0)
			{
				// 이미지 생성
				Routine::Graphics::CImage img;
				img.Create(width, height);
				img.Clear(255);

				// 중심선 그리기
				img.DrawRect(0, 0, width-1, height-1, black);
				img.DrawLine(cx, 0, cx, height, black);
				img.DrawLine(0, cy, width, cy, black);

				// 원점 그리기
				int32_t originX = GetPosX(-m_joint.cy, ratio, cx);
				int32_t originY = GetPosY(-m_joint.cz, ratio, cy);
				img.DrawFillCircle(originX, originY, 2, black);

				// 입력 포인트 그리기
				if (m_cloudOrigin)
				{
					for (uint32_t i = 0; i < m_cloudOrigin->size(); i++)
					{
						int32_t u = GetPosX(m_cloudOrigin->at(i).x, ratio, cx);
						int32_t v = GetPosY(m_cloudOrigin->at(i).y, ratio, cy);
						int32_t s = 1;
						if (s <= u && u < img.GetWidth() - s && s <= v && v < img.GetHeight() - s)
						{
							img.DrawFillCircle(u, v, s, black);
						}
					}
				}

				// ROI 그리기
				float_t s = sin(pcl::deg2rad(-(180-GetResult())));
				float_t c = cos(pcl::deg2rad(-(180-GetResult())));
				if (m_vPointsMax)
				{
					for (uint32_t iRoi = 0; iRoi < m_vPointsMax->size(); iRoi++)
					{
						// ROI 포인트 그리기
						Routine::Graphics::CImageColor color(rand() % 255, rand() % 255, rand() % 255);
						for (uint32_t i = 0; i < m_vPointsMax->at(iRoi)->size(); i++)
						{
							float_t x = m_vPointsMax->at(iRoi)->at(i).x;
							float_t y = m_vPointsMax->at(iRoi)->at(i).y;
							float_t rx = x * c - y * s;
							float_t ry = x * s + y * c;
							int32_t u = GetPosX(rx, ratio, cx);
							int32_t v = GetPosY(ry, ratio, cy);
							int32_t s = 2;
							if (s <= u && u < img.GetWidth() - s && s <= v && v < img.GetHeight() - s)
							{
								img.DrawFillCircle(u, v, s, color);
							}
						}

						// ROI 박스 그리기
						float_t x1 = m_vInliers[iRoi].roi.minX;
						float_t y1 = m_vInliers[iRoi].roi.minY;
						float_t x2 = m_vInliers[iRoi].roi.maxX;
						float_t y2 = m_vInliers[iRoi].roi.minY;
						float_t x3 = m_vInliers[iRoi].roi.minX;
						float_t y3 = m_vInliers[iRoi].roi.maxY;
						float_t x4 = m_vInliers[iRoi].roi.maxX;
						float_t y4 = m_vInliers[iRoi].roi.maxY;

						float_t rx1 = x1 * c - y1 * s;
						float_t ry1 = x1 * s + y1 * c;
						float_t rx2 = x2 * c - y2 * s;
						float_t ry2 = x2 * s + y2 * c;
						float_t rx3 = x3 * c - y3 * s;
						float_t ry3 = x3 * s + y3 * c;
						float_t rx4 = x4 * c - y4 * s;
						float_t ry4 = x4 * s + y4 * c;

						int32_t u1 = GetPosX(rx1, ratio, cx);
						int32_t v1 = GetPosY(ry1, ratio, cy);
						int32_t u2 = GetPosX(rx2, ratio, cx);
						int32_t v2 = GetPosY(ry2, ratio, cy);
						int32_t u3 = GetPosX(rx3, ratio, cx);
						int32_t v3 = GetPosY(ry3, ratio, cy);
						int32_t u4 = GetPosX(rx4, ratio, cx);
						int32_t v4 = GetPosY(ry4, ratio, cy);

						img.DrawLine(u1, v1, u2, v2, color);
						img.DrawLine(u2, v2, u4, v4, color);
						img.DrawLine(u4, v4, u3, v3, color);
						img.DrawLine(u3, v3, u1, v1, color);
					}
				}				

				// 결과 반환
				dst = img;
				ret = true;
			}
			return ret;
		}

	}

}
