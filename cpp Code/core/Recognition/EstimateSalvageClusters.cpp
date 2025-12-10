#include "EstimateSalvageClusters.h"
#include <pcl/common/distances.h>

namespace SHI
{
	namespace Recognition
	{
		inline float_t squaredEuclideanDistance(const SHI::StPoint& p1, const SHI::StPoint& p2)
		{
			float_t diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
			return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
		}

		CEstimateSalvageClusters::CEstimateSalvageClusters(void)
			: m_maxCoworkDistanceSq(100), m_marginROI(20), m_roi(-FLT_MAX, -FLT_MAX, -FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX)
		{
			m_hookStatus[0] = false;
			m_hookStatus[1] = false;
			m_hookStatus[2] = false;
		}

		CEstimateSalvageClusters::~CEstimateSalvageClusters(void)
		{
		}

		void CEstimateSalvageClusters::Initialize(float_t maxCoworkDistance, float_t minHeight)
		{
			m_maxCoworkDistanceSq = maxCoworkDistance*maxCoworkDistance;
			m_minHeight = minHeight;
		}

		void CEstimateSalvageClusters::PushHookPositions(const SHI::StPoint& pos1, const SHI::StPoint& pos2, const SHI::StPoint& pos3)
		{
			m_hookPos[0] = pos1;
			m_hookPos[1] = pos2;
			m_hookPos[2] = pos3;
		}

		void CEstimateSalvageClusters::PushHookStatus(bool stat1, bool stat2, bool stat3)
		{
			m_hookStatus[0] = stat1;
			m_hookStatus[1] = stat2;
			m_hookStatus[2] = stat3;
		}

		void CEstimateSalvageClusters::PushClusters(const SHI::PointCloudPtr& points, const SHI::PCLIndicesVectorPtr& clusters)
		{
			m_points = points;
			m_clusters = clusters;
		}

		void CEstimateSalvageClusters::PushROI(const SHI::StROI& roi)
		{
			m_roi = roi;
		}

		void CEstimateSalvageClusters::Run(SHI::PCLIndicesVectorPtr& salvageClusters, SHI::PCLIndicesVectorPtr& others)
		{
			std::vector<uint32_t> vSalvageClusters;
			salvageClusters->clear();
			others->clear();

			// 협업 상태 판단하여 관심영역 생성
			ProcessROI(m_vRoi);

			// 관심 영역의 Min/Max 제한
			for (uint32_t i = 0; i < m_vRoi.size(); i++)
			{
				m_vRoi.at(i).roi.minX = std::min(std::max(m_vRoi.at(i).roi.minX, m_roi.roi.minX), m_roi.roi.maxX);
				m_vRoi.at(i).roi.minY = std::min(std::max(m_vRoi.at(i).roi.minY, m_roi.roi.minY), m_roi.roi.maxY);
				m_vRoi.at(i).roi.minZ = std::min(std::max(m_vRoi.at(i).roi.minZ, m_roi.roi.minZ), m_roi.roi.maxZ);
				m_vRoi.at(i).roi.maxX = std::min(std::max(m_vRoi.at(i).roi.maxX, m_roi.roi.minX), m_roi.roi.maxX);
				m_vRoi.at(i).roi.maxY = std::min(std::max(m_vRoi.at(i).roi.maxY, m_roi.roi.minY), m_roi.roi.maxY);
				m_vRoi.at(i).roi.maxZ = std::min(std::max(m_vRoi.at(i).roi.maxZ, m_roi.roi.minZ), m_roi.roi.maxZ);
			}

			for (uint32_t iCluster = 0; iCluster < m_clusters->size(); iCluster++)
			{
				PCLIndices othersPoints;
				PCLIndices hookPoints;
				for (uint32_t iPoint = 0; iPoint < m_clusters->at(iCluster).indices.size(); iPoint++)
				{
					int32_t index = m_clusters->at(iCluster).indices[iPoint];
					SHI::Point3D point = m_points->points[index];

					bool bInlier = false;
					for (uint32_t iRoi = 0; iRoi < m_vRoi.size(); iRoi++)
					{
						StROI roi = m_vRoi.at(iRoi);
						if (roi.IsInlier(point))
						{
							bInlier = true;
							break;
						}
					}

					if (bInlier)
					{
						hookPoints.indices.push_back(index);
					}
					else
					{
						othersPoints.indices.push_back(index);
					}
				}
				if (hookPoints.indices.size()) salvageClusters->push_back(hookPoints);
				if (othersPoints.indices.size()) others->push_back(othersPoints);
			}
		}

		void CEstimateSalvageClusters::GetCurrentROI(std::vector<SHI::StROI>& vroi)
		{
			vroi = m_vRoi;
		}

		void CEstimateSalvageClusters::ProcessROI(std::vector<SHI::StROI>& vRoi)
		{
			std::vector<SHI::StROI> tmp;
			if (m_hookStatus[0] && m_hookStatus[1] && m_hookStatus[2])
			{
				float_t d1 = squaredEuclideanDistance(m_hookPos[0], m_hookPos[1]);
				float_t d2 = squaredEuclideanDistance(m_hookPos[1], m_hookPos[2]);
				float_t d3 = squaredEuclideanDistance(m_hookPos[2], m_hookPos[0]);
				if (d1 < m_maxCoworkDistanceSq &&
					d2 < m_maxCoworkDistanceSq &&
					d3 < m_maxCoworkDistanceSq)
				{
					// d1, d2, d3 coworking
					float_t minX = std::min(m_hookPos[0].x - m_marginROI, m_hookPos[1].x - m_marginROI);
					float_t minY = std::min(m_hookPos[0].y - m_marginROI, m_hookPos[1].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[0].x + m_marginROI, m_hookPos[1].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[0].y + m_marginROI, m_hookPos[1].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[0].z, m_hookPos[1].z);
					minX = std::min(minX, m_hookPos[2].x - m_marginROI);
					minY = std::min(minY, m_hookPos[2].y - m_marginROI);
					maxX = std::max(maxX, m_hookPos[2].x + m_marginROI);
					maxY = std::max(maxY, m_hookPos[2].y + m_marginROI);
					maxZ = std::max(maxZ, m_hookPos[2].z);
					SHI::StROI roi(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi);
				}
				else if (d1 >= m_maxCoworkDistanceSq &&
					d2 < m_maxCoworkDistanceSq &&
					d3 < m_maxCoworkDistanceSq)
				{
					// d1 working and d2, d3 coworking
					StROI roi1(
						m_hookPos[0].x - m_marginROI, m_hookPos[0].x + m_marginROI,
						m_hookPos[0].y - m_marginROI, m_hookPos[0].y + m_marginROI,
						m_minHeight, m_hookPos[0].z);
					tmp.push_back(roi1);

					float_t minX = std::min(m_hookPos[1].x - m_marginROI, m_hookPos[2].x - m_marginROI);
					float_t minY = std::min(m_hookPos[1].y - m_marginROI, m_hookPos[2].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[1].x + m_marginROI, m_hookPos[2].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[1].y + m_marginROI, m_hookPos[2].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[1].z, m_hookPos[2].z);
					SHI::StROI roi2(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi2);
				}
				else if (d1 < m_maxCoworkDistanceSq &&
					d2 >= m_maxCoworkDistanceSq &&
					d3 < m_maxCoworkDistanceSq)
				{
					// d2 working and d1, d3 coworking
					SHI::StROI roi1(
						m_hookPos[1].x - m_marginROI, m_hookPos[1].x + m_marginROI,
						m_hookPos[1].y - m_marginROI, m_hookPos[1].y + m_marginROI,
						m_minHeight, m_hookPos[1].z);
					tmp.push_back(roi1);

					float_t minX = std::min(m_hookPos[0].x - m_marginROI, m_hookPos[2].x - m_marginROI);
					float_t minY = std::min(m_hookPos[0].y - m_marginROI, m_hookPos[2].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[0].x + m_marginROI, m_hookPos[2].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[0].y + m_marginROI, m_hookPos[2].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[0].z, m_hookPos[2].z);
					SHI::StROI roi2(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi2);
				}
				else if (d1 < m_maxCoworkDistanceSq &&
					d2 < m_maxCoworkDistanceSq &&
					d3 >= m_maxCoworkDistanceSq)
				{
					// d3 working and d1, d2 coworking
					SHI::StROI roi1(
						m_hookPos[2].x - m_marginROI, m_hookPos[2].x + m_marginROI,
						m_hookPos[2].y - m_marginROI, m_hookPos[2].y + m_marginROI,
						m_minHeight, m_hookPos[2].z);
					tmp.push_back(roi1);

					float_t minX = std::min(m_hookPos[0].x - m_marginROI, m_hookPos[1].x - m_marginROI);
					float_t minY = std::min(m_hookPos[0].y - m_marginROI, m_hookPos[1].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[0].x + m_marginROI, m_hookPos[1].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[0].y + m_marginROI, m_hookPos[1].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[0].z, m_hookPos[1].z);
					StROI roi2(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi2);
				}
				else
				{
					// d1, d2, d3 not coworking
					SHI::StROI roi1(
						m_hookPos[0].x - m_marginROI, m_hookPos[0].x + m_marginROI,
						m_hookPos[0].y - m_marginROI, m_hookPos[0].y + m_marginROI,
						m_minHeight, m_hookPos[0].z);
					tmp.push_back(roi1);

					SHI::StROI roi2(
						m_hookPos[1].x - m_marginROI, m_hookPos[1].x + m_marginROI,
						m_hookPos[1].y - m_marginROI, m_hookPos[1].y + m_marginROI,
						m_minHeight, m_hookPos[1].z);
					tmp.push_back(roi2);

					SHI::StROI roi3(
						m_hookPos[2].x - m_marginROI, m_hookPos[2].x + m_marginROI,
						m_hookPos[2].y - m_marginROI, m_hookPos[2].y + m_marginROI,
						m_minHeight, m_hookPos[2].z);
					tmp.push_back(roi3);
				}
			}
			else if (m_hookStatus[0] && m_hookStatus[1])
			{
				float_t d = squaredEuclideanDistance(m_hookPos[0], m_hookPos[1]);
				if (d < m_maxCoworkDistanceSq)
				{
					// coworking
					float_t minX = std::min(m_hookPos[0].x - m_marginROI, m_hookPos[1].x - m_marginROI);
					float_t minY = std::min(m_hookPos[0].y - m_marginROI, m_hookPos[1].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[0].x + m_marginROI, m_hookPos[1].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[0].y + m_marginROI, m_hookPos[1].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[0].z, m_hookPos[1].z);
					SHI::StROI roi(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi);

				}
				else
				{
					// not coworking
					SHI::StROI roi1(
						m_hookPos[0].x - m_marginROI, m_hookPos[0].x + m_marginROI,
						m_hookPos[0].y - m_marginROI, m_hookPos[0].y + m_marginROI,
						m_minHeight, m_hookPos[0].z);
					tmp.push_back(roi1);

					SHI::StROI roi2(
						m_hookPos[1].x - m_marginROI, m_hookPos[1].x + m_marginROI,
						m_hookPos[1].y - m_marginROI, m_hookPos[1].y + m_marginROI,
						m_minHeight, m_hookPos[1].z);
					tmp.push_back(roi2);
				}

			}
			else if (m_hookStatus[1] && m_hookStatus[2])
			{
				float_t d = squaredEuclideanDistance(m_hookPos[1], m_hookPos[2]);
				if (d < m_maxCoworkDistanceSq)
				{
					// coworking
					float_t minX = std::min(m_hookPos[1].x - m_marginROI, m_hookPos[2].x - m_marginROI);
					float_t minY = std::min(m_hookPos[1].y - m_marginROI, m_hookPos[2].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[1].x + m_marginROI, m_hookPos[2].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[1].y + m_marginROI, m_hookPos[2].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[1].z, m_hookPos[2].z);
					SHI::StROI roi(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi);
				}
				else
				{
					// not coworking
					SHI::StROI roi1(
						m_hookPos[1].x - m_marginROI, m_hookPos[1].x + m_marginROI,
						m_hookPos[1].y - m_marginROI, m_hookPos[1].y + m_marginROI,
						m_minHeight, m_hookPos[1].z);

					SHI::StROI roi2(
						m_hookPos[2].x - m_marginROI, m_hookPos[2].x + m_marginROI,
						m_hookPos[2].y - m_marginROI, m_hookPos[2].y + m_marginROI,
						m_minHeight, m_hookPos[2].z);
					tmp.push_back(roi2);
				}
			}
			else if (m_hookStatus[2] && m_hookStatus[0])
			{
				float_t d = squaredEuclideanDistance(m_hookPos[2], m_hookPos[0]);
				if (d < m_maxCoworkDistanceSq)
				{
					// coworking
					float_t minX = std::min(m_hookPos[0].x - m_marginROI, m_hookPos[2].x - m_marginROI);
					float_t minY = std::min(m_hookPos[0].y - m_marginROI, m_hookPos[2].y - m_marginROI);
					float_t minZ = m_minHeight;
					float_t maxX = std::max(m_hookPos[0].x + m_marginROI, m_hookPos[2].x + m_marginROI);
					float_t maxY = std::max(m_hookPos[0].y + m_marginROI, m_hookPos[2].y + m_marginROI);
					float_t maxZ = std::max(m_hookPos[0].z, m_hookPos[2].z);
					SHI::StROI roi(minX, maxX, minY, maxY, minZ, maxZ);
					tmp.push_back(roi);
				}
				else
				{
					// not coworking
					SHI::StROI roi1(
						m_hookPos[0].x - m_marginROI, m_hookPos[0].x + m_marginROI,
						m_hookPos[0].y - m_marginROI, m_hookPos[0].y + m_marginROI,
						m_minHeight, m_hookPos[0].z);
					tmp.push_back(roi1);

					SHI::StROI roi2(
						m_hookPos[2].x - m_marginROI, m_hookPos[2].x + m_marginROI,
						m_hookPos[2].y - m_marginROI, m_hookPos[2].y + m_marginROI,
						m_minHeight, m_hookPos[2].z);
					tmp.push_back(roi2);

				}
			}
			else if (m_hookStatus[0])
			{
				SHI::StROI roi(
					m_hookPos[0].x - m_marginROI, m_hookPos[0].x + m_marginROI,
					m_hookPos[0].y - m_marginROI, m_hookPos[0].y + m_marginROI,
					m_minHeight, m_hookPos[0].z);
				tmp.push_back(roi);
			}
			else if (m_hookStatus[1])
			{
				SHI::StROI roi(
					m_hookPos[1].x - m_marginROI, m_hookPos[1].x + m_marginROI,
					m_hookPos[1].y - m_marginROI, m_hookPos[1].y + m_marginROI,
					m_minHeight, m_hookPos[1].z);
				tmp.push_back(roi);
			}
			else if (m_hookStatus[2])
			{
				SHI::StROI roi(
					m_hookPos[2].x - m_marginROI, m_hookPos[2].x + m_marginROI,
					m_hookPos[2].y - m_marginROI, m_hookPos[2].y + m_marginROI,
					m_minHeight, m_hookPos[2].z);
				tmp.push_back(roi);
			}
			else
			{
				// Crane is not on work.
			}

			vRoi = tmp;
		}

		void CEstimateSalvageClusters::ExtractClustersInROI(SHI::StROI roi, std::vector<int32_t> &clusters)
		{
			clusters.clear();

			// ROI에 걸치는 클러스터
			for (uint32_t iCluster = 0; iCluster < m_clusters->size(); iCluster++)
			{
				SHI::PCLIndices clusterIndices = m_clusters->at(iCluster);
				bool bClusterInROI = false;

				for (uint32_t iClusterPoint = 0; iClusterPoint < clusterIndices.indices.size(); iClusterPoint++)
				{
					uint32_t idx = clusterIndices.indices.at(iClusterPoint);
					pcl::PointXYZ point = m_points->at(idx);

					bool bInROI = roi.IsInlier(point);
					if (bInROI)
					{
						bClusterInROI = true;
						break;
					}
				}

				if (bClusterInROI)
				{
					clusters.push_back(iCluster);
				}
			}
		}

		int32_t CEstimateSalvageClusters::GetHeighestCluster(std::vector<int32_t>& clusters)
		{
			float_t maxHeight = -FLT_MAX;
			uint32_t idxMaxCluster = clusters.at(0);

			for (uint32_t i = 0; i < clusters.size(); i++)
			{
				// 해당 클러스터에서 가장 높은 점의 높이
				float_t curMaxHeight = -FLT_MAX;

				SHI::PCLIndices clusterIndices = m_clusters->at(clusters.at(i));
				for (uint32_t iClusterPoint = 0; iClusterPoint < clusterIndices.indices.size(); iClusterPoint++)
				{
					uint32_t idx = clusterIndices.indices.at(iClusterPoint);
					pcl::PointXYZ point = m_points->at(idx);
					curMaxHeight = std::max(curMaxHeight, point.z);
				}

				// 전체 클러스터의 최대점 여부 검사
				if (curMaxHeight > maxHeight)
				{
					maxHeight = curMaxHeight;
					idxMaxCluster = clusters.at(i);
				}
			}

			return idxMaxCluster;
		}

	}
}