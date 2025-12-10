#include "EstimateHookClusterSalvage.h"
#include "EstimateHookCluster.h"
#include <pcl/segmentation/extract_clusters.h>

namespace SHI
{
	namespace Recognition
	{

		void CEstimateHookClusterSalvageGC::Initialize(const SHI::StROI& salvageRoi, const SHI::CraneAttitude& attitude)
		{
			m_salvageRoi = salvageRoi;
			for (uint32_t i=0; i<attitude.numHook; i++)
			{
				m_stat[i].Initialize(attitude.hookWightThreshold[i], 10, 30);
			}
			m_salvage.Initialize(25, attitude.groundHeight);
		}

		bool CEstimateHookClusterSalvageGC::Run(const SHI::PointCloudPtr cloud, const SHI::PCLIndicesVectorPtr& clusterIndicesInput, const SHI::CraneAttitudePtr& attitude)
		{
			bool ret = true;

			// 후크 후보 클러스터 분류
			SHI::PCLIndicesVectorPtr vHookCandidates(new SHI::PCLIndicesVector);
			SHI::PCLIndicesVectorPtr clusterIndicesOthers(new SHI::PCLIndicesVector);
			ExreactHookCandidates(cloud, clusterIndicesInput, attitude, *vHookCandidates, *clusterIndicesOthers);

			// 후크 클러스터 추출
			SHI::PCLIndicesVectorPtr clusterIndicesHook(new SHI::PCLIndicesVector);
			ExtractHook(cloud, *vHookCandidates, attitude, *clusterIndicesHook, *clusterIndicesOthers);

			// 인양물 상태 갱신
			for (uint32_t i=0; i<attitude->numHook; i++)
			{
				m_stat[i].UpdateWeight(attitude->hookWeight[i]);
			}
			//////////////////////////////////////////////////////////////////////////
			// 인양물 인식
			SHI::PCLIndicesVectorPtr salvageClusters(new SHI::PCLIndicesVector);
			SHI::PCLIndicesVectorPtr others(new SHI::PCLIndicesVector);
			m_salvage.PushClusters(cloud, clusterIndicesOthers);
			m_salvage.PushHookPositions(attitude->hookPoint[0], attitude->hookPoint[1], attitude->hookPoint[2]);
			m_salvage.PushHookStatus(m_stat[0].GetResult(), m_stat[1].GetResult(), m_stat[2].GetResult());
			m_salvage.PushROI(m_salvageRoi);
			m_salvage.Run(salvageClusters, others);
			
			//////////////////////////////////////////////////////////////////////////
			// 결과 반환
			clusterIndicesHook->insert(clusterIndicesHook->end(), salvageClusters->begin(), salvageClusters->end());
			m_indicesObject = *others;
			m_indicesHook = *clusterIndicesHook;
			
			return ret;
		}
		
		SHI::SalvageInfo CEstimateHookClusterSalvageGC::GetSalvageInfo()
		{
			SHI::SalvageInfo info;

			// 인양물 Roi 불러오기
			std::vector<SHI::StROI> roi;
			m_salvage.GetCurrentROI(roi);

			// 인양물 Roi 갱신
			info.numSalvageRoi = roi.size();
			for (uint32_t i=0; i<roi.size() && i < 5; i++)
			{
				info.salvageRoi[i] = roi[i];
			}

			// 인양물 상태 갱신
			for (uint32_t i = 0; i < 5; i++)
			{
				info.salvageStatus[i] = m_stat[i].GetResult();
			}
			return info;
		}

	}
}