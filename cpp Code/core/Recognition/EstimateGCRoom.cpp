
#include "EstimateGCRoom.h"
#include "../Utility/filter.h"
#include <Routine/include/Visualization/CImageViewer.h>

#ifdef _DEBUG
#include <pcl/visualization/pcl_visualizer.h>
#endif

namespace SHI
{
	namespace Recognition
	{
#ifdef _DEBUG
		pcl::visualization::PCLVisualizer::Ptr m_viewer;
#endif

		CEstimateGCRoom::CEstimateGCRoom()
			: CAbstractEstimator(0.1, 0.7, 0.05, 0.9, 0.2, 0.5, false)
		{

		}

		CEstimateGCRoom::~CEstimateGCRoom()
		{

		}

		void CEstimateGCRoom::Initialize(float_t dMin, float_t dMax, float_t dRes, StROI roiPoint, std::vector<SHI::StROI>& vRoiInlier, int32_t minCount)
		{
			m_dMin = dMin;
			m_dMax = dMax;
			m_dRes = dRes;
			m_roiPoint = roiPoint;
			m_vRoiInlier = vRoiInlier;
			m_minCount = minCount;
		}
		 
		bool CEstimateGCRoom::Run(PointCloudPtr cloud)
		{
			bool ret = false;

			// 입력 포인트 ROI 처리
			PointCloudPtr pointRoi(new PointCloud);
			SHI::Filter::FilterROI(cloud, *pointRoi, m_roiPoint);

			// 각도별 평가값 계산
			uint32_t maxCount = 0;
			float_t distance = 0;
			PointCloudPtr pointMax;

			//pjh
			/*m_viewer.reset(new pcl::visualization::PCLVisualizer("Before ROI viewer"));
					m_viewer->addPointCloud(cloud);
					while (!m_viewer->wasStopped())
					{
						m_viewer->spinOnce();
					}
			m_viewer = nullptr;

			m_viewer.reset(new pcl::visualization::PCLVisualizer("After ROI viewer"));
			m_viewer->addPointCloud(pointRoi);
			while (!m_viewer->wasStopped())
			{
				m_viewer->spinOnce();
			}
			m_viewer = nullptr;*/
			//

			for (float_t d=m_dMin; d<m_dMax; d+=m_dRes)
			{
				// ROI 합 계산
				PointCloudPtr pointIn(new PointCloud);
				for (uint32_t i=0; i<m_vRoiInlier.size(); i++)
				{
					StROI roiCur = m_vRoiInlier[i];
					roiCur.roi.minY += d;
					roiCur.roi.maxY += d;
					PointCloudPtr pointTmp(new PointCloud);
					SHI::Filter::FilterROI(pointRoi, *pointTmp, roiCur);

					// ROI 포인트 병합
					*pointIn += *pointTmp;
				}
				
				// 평가값 비교
				if (maxCount < pointIn->size())
				{
					maxCount = pointIn->size();
					distance = d;
					pointMax = pointIn;
				}
			}

			if (m_minCount < maxCount)
			{
				UpdateResult(distance);
				ret = true;
			}
			//pjh
			/*m_viewer.reset(new pcl::visualization::PCLVisualizer("After ROI viewer"));
			m_viewer->addPointCloud(pointMax);
			while (!m_viewer->wasStopped())
			{
				m_viewer->spinOnce();
			}
			m_viewer = nullptr;*/
			//

			m_pointRoi = pointRoi;
			m_pointMax = pointMax;
			return ret;
		}

		inline int32_t GetPosX(float_t px, float_t ratio, int32_t cx) { return cx + int32_t(px * ratio); }
		inline int32_t GetPosY(float_t py, float_t ratio, int32_t cy) { return cy - int32_t(py * ratio); }

		bool CEstimateGCRoom::GetResultImage(Routine::Graphics::CImage& dst)
		{
			bool ret = false;
			const Routine::Graphics::CImageColor green(20, 230, 20);
			const Routine::Graphics::CImageColor red(255, 20, 20);
			const Routine::Graphics::CImageColor black(0, 0, 0);
			const float_t ratio = 5;
			float_t w = (m_roiPoint.roi.maxX - m_roiPoint.roi.minX);
			float_t h = (m_roiPoint.roi.maxY - m_roiPoint.roi.minY);
			int32_t width = (int32_t)w*ratio;
			int32_t height = (int32_t)h*ratio;
			int32_t cy = height;
			int32_t cx = width / 2;

			if (width > 0 && height > 0 && m_pointRoi)
			{
				// 이미지 생성
				Routine::Graphics::CImage img;
				img.Create(width, height);
				img.Clear(255);

				// 중심선 그리기
				img.DrawRect(0, 0, width - 1, height - 1, black);
				img.DrawLine(cx, 0, cx, height, black);


				// 입력 포인트 그리기
				if (m_pointRoi)
				{
					for (uint32_t i = 0; i < m_pointRoi->size(); i++)
					{
						int32_t u = GetPosX(m_pointRoi->at(i).x, ratio, cx);
						int32_t v = GetPosY(m_pointRoi->at(i).y, ratio, cy);
						int32_t s = 1;
						if (s <= u && u < img.GetWidth() - s && s <= v && v < img.GetHeight() - s)
						{
							img.DrawFillCircle(u, v, s, black);
						}
					}
				}

				// ROI 포인트 그리기
				if (m_pointMax)
				{
					for (uint32_t i = 0; i < m_pointMax->size(); i++)
					{
						float_t x = m_pointMax->at(i).x;
						float_t y = m_pointMax->at(i).y;
						int32_t u = GetPosX(x, ratio, cx);
						int32_t v = GetPosY(y, ratio, cy);
						int32_t s = 2;
						if (s <= u && u < img.GetWidth() - s && s <= v && v < img.GetHeight() - s)
						{
							img.DrawFillCircle(u, v, s, green);
						}
					}
				}

				// ROI 박스 그리기
				for (uint32_t i = 0; i < m_vRoiInlier.size(); i++)
				{
					int32_t sx = GetPosX(m_vRoiInlier[i].roi.minX, ratio, cx);
					int32_t sy = GetPosY(m_vRoiInlier[i].roi.minY + GetResult(), ratio, cy);
					int32_t ex = GetPosX(m_vRoiInlier[i].roi.maxX, ratio, cx);
					int32_t ey = GetPosY(m_vRoiInlier[i].roi.maxY + GetResult(), ratio, cy);
					img.DrawRect(sx, ey, abs(ex - sx), abs(ey - sy), red);
				}

				// 결과 반환
				ret = true;
				dst = img;
			}
			return ret;
		}

	}
}