
#include "EstimateLLCJib.h"
#include "../Utility/filter.h"
#include "../3rd/Routine/include/Base/RoutineUtility.h"

#include <pcl/common/angles.h>
#include <vector>
#include <math.h>

namespace SHI
{
	namespace Recognition
	{
		CEstimateLLCJib::CEstimateLLCJib()
			: CAbstractEstimator(0.1, 0.8, 0.1, 0.9, 0.5, 0.2, true)
		{
		}

		CEstimateLLCJib::~CEstimateLLCJib()
		{
		}

		void CEstimateLLCJib::Initialize(float_t minAngle, float_t maxAngle, float_t dAngle, const StJointInfo& joint, StROI roi, int32_t minCount, float_t dJibUpperLineNegative, float_t dr)
		{
			m_joint = joint;
			m_dtheta = dAngle;
			m_thetaMin = minAngle - 90;
			m_thetaMax = maxAngle - 90;
			m_minCount = minCount;
			m_dJibUpperLine = dJibUpperLineNegative;
			m_dr = dr;

			m_roi.roi.minX = roi.roi.minX + joint.cx;
			m_roi.roi.maxX = roi.roi.maxX + joint.cx;
			m_roi.roi.minY = roi.roi.minY + joint.cy;
			m_roi.roi.maxY = roi.roi.maxY + joint.cy;
			m_roi.roi.minZ = roi.roi.minZ + joint.cz;
			m_roi.roi.maxZ = roi.roi.maxZ + joint.cz;
		}

		bool CEstimateLLCJib::Run(PointCloudPtr cloud)
		{
			bool ret = false;
			float_t dthetaInv = 1 / m_dtheta;

			// ROI 처리
			PointCloud cloudRoi;
			SHI::Filter::FilterROI(cloud, cloudRoi, m_roi);

			// 입력 포인트를 2D 포인트로 변환
			std::shared_ptr<std::vector<Point2D>> points2d(new std::vector<Point2D>);
			for (uint32_t i = 0; i < cloudRoi.size(); i++)
			{
				Point3D point3d = cloudRoi.points.at(i);
				Point2D point2d;
				point2d.x = point3d.y - m_joint.cy;
				point2d.y = point3d.z - m_joint.cz;
				points2d->push_back(point2d);
			}

			// Hough 변환
			int32_t width = (m_thetaMax - m_thetaMin) * dthetaInv;
			uint32_t *arCount = new uint32_t[width];
			memset(arCount, 0, sizeof(uint32_t) * width);

			for (uint32_t i = 0; i < points2d->size(); i++)
			{
				for (float_t theta = m_thetaMin; theta <= m_thetaMax; theta += m_dtheta)
				{
					float_t r = points2d->at(i).x * cos(pcl::deg2rad(theta)) + points2d->at(i).y * sin(pcl::deg2rad(theta));
					int32_t pos = (theta - m_thetaMin) * dthetaInv;

					if (0 <= pos && pos < width )
					{
						if (m_dJibUpperLine - m_dr <= r && r <= m_dJibUpperLine + m_dr)
						{
							arCount[pos] ++;
						}
					}
				}
			}

			// 최대값 추출
			uint32_t maxCount = 0;
			uint32_t maxIdx = 0;
			for (uint32_t i = 0; i < width; i++)
			{
				if (maxCount <= arCount[i])
				{
					maxCount = arCount[i];
					maxIdx = i;
				}
			}
			delete arCount;

			// 결과
			float_t thetaResult = (maxIdx * m_dtheta + m_thetaMin) + 90 + Routine::GetConfigDouble("Offset", "Jib", 0, "ClusterProcessor.json");;
			m_points2d = points2d;

			if (maxCount > m_minCount)
			{
				UpdateResult(thetaResult);
				ret = true;
			}
			return ret;

			// cms
			/*uint32_t maxCount1 = 0;
			uint32_t maxCount2 = 0;
			uint32_t maxIdx1 = 0;
			uint32_t maxIdx2 = 0;
			for (uint32_t i = 0; i < width; i++)
			{
				if (maxCount1 <= arCount[i])
				{
					maxCount2 = maxCount1; maxIdx2 = maxIdx1;
					maxCount1 = arCount[i];
					maxIdx1 = i;
				}
				else if (arCount[i] > maxCount2 && arCount[i] != maxCount1) {
					maxCount2 = arCount[i];
					maxIdx2 = i;
				}
			}*/
			
			//delete arCount;
			//// 결과
			//float_t thetaResult = (((maxIdx1 + maxIdx2) / 2) * m_dtheta + m_thetaMin) + 90 + Routine::GetConfigDouble("Offset", "Jib", 10, "ClusterProcessor.json");
			//m_points2d = points2d;

			//if (maxCount2 > m_minCount)
			//{
			//	UpdateResult(thetaResult);
			//	ret = true;
			//}
			//return ret;
			//~cms
		}

		inline int32_t GetPosX(float_t px, float_t ratio, int32_t cx) { return cx + int32_t(px * ratio); }
		inline int32_t GetPosY(float_t py, float_t ratio, int32_t cy) { return cy - int32_t(py * ratio); }

		bool CEstimateLLCJib::GetResultImage(Routine::Graphics::CImage& dst)
		{
			const Routine::Graphics::CImageColor red(255, 20, 20);
			const Routine::Graphics::CImageColor black(0, 0, 0);
			const Routine::Graphics::CImageColor blue(25, 20, 175);
			std::shared_ptr<std::vector<Point2D>> points2d = m_points2d;
			bool ret = false;

			const float_t ratio = 10;
			float_t w = (m_roi.roi.maxY - m_roi.roi.minY);
			float_t h = (m_roi.roi.maxZ - m_roi.roi.minZ);

			int32_t width = (int32_t)w*ratio;
			int32_t height = (int32_t)h*ratio;
			int32_t cy = height / 2;
			int32_t cx = 0;

			if (width > 0 && height > 0)
			{
				// 이미지 생성
				Routine::Graphics::CImage img ;
				img.Create(width, height);
				img.Clear(255);

				// 중심선 그리기
				img.DrawRect(0, 0, width-1, height-1, black);
				img.DrawLine(cx, cy, width, cy, black);

				// 원점 그리기
				int32_t originX = GetPosX(-m_joint.cy, ratio, cx);
				int32_t originY = GetPosY(-m_joint.cz, ratio, cy);
				img.DrawFillCircle(originX, originY, 4, black);

				// 상하한값 표시
				int32_t x1 = GetPosX(cos(pcl::deg2rad(m_thetaMin + 90)) * 100, ratio, cx);
				int32_t y1 = GetPosY(sin(pcl::deg2rad(m_thetaMin + 90)) * 100, ratio, cy);
				int32_t x2 = GetPosX(cos(pcl::deg2rad(m_thetaMax + 90)) * 100, ratio, cx);
				int32_t y2 = GetPosY(sin(pcl::deg2rad(m_thetaMax + 90)) * 100, ratio, cy);
				img.DrawLine(0, cy, x1, y1, blue);
				img.DrawLine(0, cy, x2, y2, blue);

				// 입력 포인트 그리기
				if (points2d)
				{
					for (uint32_t i = 0; i < points2d->size(); i++)
					{
						float_t theta = GetResult() - 90;
						float_t dthetaInv = 1.0f / m_dtheta;
						float_t r = points2d->at(i).x * cos(pcl::deg2rad(theta)) + points2d->at(i).y * sin(pcl::deg2rad(theta));
						int32_t pos = (theta - m_thetaMin) * dthetaInv;

						int32_t u = GetPosX(points2d->at(i).x, ratio, cx);
						int32_t v = GetPosY(points2d->at(i).y, ratio, cy);
						int32_t s = 1;
						if (s <= u && u < img.GetWidth() - s && s <= v && v < img.GetHeight() - s)
						{
							if (m_dJibUpperLine - m_dr <= r && r <= m_dJibUpperLine + m_dr)
							{
								img.DrawFillCircle(u, v, s+1, red);
							}
							else
							{
								img.DrawFillCircle(u, v, s, black);
							}
						}
					}
				}

				// 결과 각도 그리기
				float_t _x = w;
				float_t _y = _x  * tan(pcl::deg2rad(GetResult()));
				int32_t sx = cx;
				int32_t sy = cy;
				int32_t ex = GetPosX(_x, ratio, cx);
				int32_t ey = GetPosY(_y, ratio, cy);
				img.DrawLine(sx, sy, ex, ey, red);

				// 결과 반환
				dst = img;
				ret = true;
			}
			return ret;
		}

	}
}