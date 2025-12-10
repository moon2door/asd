#include "EstimateHookCluster.h"
#include "../Utility/filter.h"
#include "../Utility/Roi.h"
#include <pcl/segmentation/extract_clusters.h>

namespace SHI
{
	namespace Recognition
	{
		bool CEstimateHookCluster::Run(const SHI::PointCloudPtr cloud, const SHI::PCLIndicesVectorPtr& clusterIndicesInput, const SHI::CraneAttitudePtr& attitude)
		{
			bool ret = true;

			// 후크 후보 클러스터 분류
			SHI::PCLIndicesVector vHookCandidates;
			SHI::PCLIndicesVector clusterIndicesOthers;
			ExreactHookCandidates(cloud, clusterIndicesInput, attitude, vHookCandidates, clusterIndicesOthers);

			// 후크 클러스터 추출
			SHI::PCLIndicesVector clusterIndicesHook;
			ExtractHook(cloud, vHookCandidates, attitude, clusterIndicesHook, clusterIndicesOthers);

			// 출력
			m_indicesObject.clear();
			m_indicesHook.clear();
			m_indicesObject.insert(m_indicesObject.end(), clusterIndicesOthers.begin(), clusterIndicesOthers.end());
			m_indicesHook.insert(m_indicesHook.end(), clusterIndicesHook.begin(), clusterIndicesHook.end());
			return ret;
		}

		void ExreactHookCandidates(const SHI::PointCloudPtr cloud, const SHI::PCLIndicesVectorPtr& clusterIndices, const SHI::CraneAttitudePtr& attitude,
			SHI::PCLIndicesVector &inliers, SHI::PCLIndicesVector &outliers)
		{
			for (uint32_t iCluster = 0; iCluster < clusterIndices->size(); iCluster++)
			{
				// 해당 클러스터 포인트가 Hook Roi에 걸치는지 검사
				bool bInHookRange = SHI::Roi::IsOverlappedIn(cloud, clusterIndices->at(iCluster), attitude->hookRoi, attitude->numHook);
				if (bInHookRange)
				{
					// hook Range에 걸치는 클러스터를 후보에 추가
					inliers.push_back(clusterIndices->at(iCluster));
				}
				else
				{
					// bound range에 클러스터가 완전히 포함되는지 검사
					// hook range에 걸치지 않더라도, bound range에 완전히 포함되는 클러스터는 후보에 포함한다.
					bool bIncluded = SHI::Roi::IsIncludedIn(cloud, clusterIndices->at(iCluster), attitude->hookBoundRoi, attitude->numHook);
					if (bIncluded)
					{
						inliers.push_back(clusterIndices->at(iCluster));
					}
					else
					{
						outliers.push_back(clusterIndices->at(iCluster));
					}
				}
			}
		}

		void ExtractHook(const SHI::PointCloudPtr cloud, const SHI::PCLIndicesVector& candidates, const SHI::CraneAttitudePtr& attitude,
			SHI::PCLIndicesVector &inliers, SHI::PCLIndicesVector &outliers)
		{
			SHI::PCLIndices hookIndices;
			// bound Range 이내의 포인트만 사용사여 클러스터 다시 묶음
			for (uint32_t iCandidate = 0; iCandidate < candidates.size(); iCandidate++)
			{
				std::shared_ptr<std::vector<int>> indicesObject(new std::vector<int>);

				for (uint32_t iPoint = 0; iPoint < candidates.at(iCandidate).indices.size(); iPoint++)
				{
					// Cluster의 각 point가 bounding box에 포함되는지 검사
					uint32_t index = candidates.at(iCandidate).indices.at(iPoint);
					bool bInlier = SHI::Roi::IsInlier(cloud->at(index), attitude->hookBoundRoi, attitude->numHook);

					// 포함 여부에 따라 point별로 분류
					if (bInlier) hookIndices.indices.push_back(index); 
					else indicesObject->push_back(index);
				}

				// 미포함 point 처리
				// 후크를 빼고 남은 데이터가 서로 멀리 떨어지는 경우가 있기 때문에, 다시 클러스터링을 해서 분리해야 한다.
				// 분리된 클러스터를 물체 클러스터에 넣는다.
				if (indicesObject->size() > 0)
				{
					std::vector<pcl::PointIndices> clusterNew;
					pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
					ec.setClusterTolerance(1.0);
					ec.setMinClusterSize(10);
					ec.setMaxClusterSize(indicesObject->size());
					ec.setInputCloud(cloud);
					ec.setIndices(indicesObject);
					ec.extract(clusterNew);

					for (uint32_t i = 0; i < clusterNew.size(); i++)
					{
						SHI::PCLIndices indicesTmp;
						indicesTmp.indices.insert(indicesTmp.indices.end(), clusterNew.at(i).indices.begin(), clusterNew.at(i).indices.end());
						outliers.push_back(indicesTmp);
					}
				}
			}
			inliers.push_back(hookIndices);
		}
	}
}