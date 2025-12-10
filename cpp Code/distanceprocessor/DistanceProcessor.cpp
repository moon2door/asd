
#include "DistanceProcessor.h"
#include "Utility/mathematics.h"

#include <Routine/Include/Base/CElapseTimer.h>

CDistanceProcessor::CDistanceProcessor()
{
	// 초기화
	if (ProcessInitialize())
	{
		if (m_attitude)
		{
			int32_t pier = m_attitude->pierId;
			int32_t id = m_attitude->craneId;
			printf("Crane pier : %s, Type : %s \n", SHI::ConvPierDisplayStr(pier).c_str(), SHI::ConvCraneStr(pier, id).c_str());
		}
		else
		{
			printf("Crane pier : ??, Type : ?? \n");
		}
	}
	else
	{
		printf("ProcessInitialize failed. \n");
	}
}

CDistanceProcessor::~CDistanceProcessor()
{
}

void CDistanceProcessor::OnCluster(SHI::Data::StCluster* pData)
{
	Routine::CElapseTimer elapsed, elapsedStep;
	printf("On update Cluster : %d clusters, %d points\n", pData->GetClusterInfoSize(), pData->GetXYZSize());
	Routine::CElapseTimer elapsedTotal;
	
	if (m_attitude && m_distanceParam && m_vRefPartModels)
	{
		// 수신 데이터
		SHI::PointCloudPtr clusterPoints(new SHI::PointCloud);
		SHI::PCLIndicesVectorPtr clusterIndices(new SHI::PCLIndicesVector);
		std::vector<unsigned char> labels;
		SHI::Convert::Cluster2XYZIndices(pData, *clusterPoints, *clusterIndices, labels);
		memcpy(m_attitude.get(), &pData->attitude, sizeof(SHI::CraneAttitude));

		// 거리 계산
		SHI::DistanceInfoVectorPtr distanceNormal(new SHI::DistanceInfoVector);
		ProcessDistance(distanceNormal, clusterPoints, clusterIndices, labels, m_vRefPartModels, m_attitude, m_distanceParam);
		printf(" > ProcessDistance : %d distances, elapsed  %dms \n", distanceNormal->size(), static_cast<int32_t>(elapsedStep.GetElapseTime() * 0.001));

		// 후크 클러스터 거리 계산
		SHI::DistanceInfoVectorPtr distanceHook(new SHI::DistanceInfoVector);
		if (m_distanceParam->bProcessHookRoiDistance)
		{
			ProcessHookCollisionDistance(distanceHook, clusterPoints, clusterIndices, labels, m_attitude, m_distanceParam);
			printf(" > ProcessHookRoiDistance : %d hook distances, elapsed  %dms \n", distanceHook->size(), static_cast<int32_t>(elapsedStep.GetElapseTime() * 0.001));
		}
		else
		{
			ProcessHookDistance(distanceHook, clusterPoints, clusterIndices, labels, m_distanceParam);
			printf(" > ProcessHookDistance : %d hook distances, elapsed  %dms \n", distanceHook->size(), static_cast<int32_t>(elapsedStep.GetElapseTime() * 0.001));
		}

		// 거리 데이터 트래킹
		SHI::DistanceInfoVectorPtr distanceTracked(new SHI::DistanceInfoVector);
		SHI::DistanceInfoVectorPtr distanceTrackedHook(new SHI::DistanceInfoVector);
		ProcessTrackDistance(*distanceTracked, distanceNormal, *distanceTrackedHook, distanceHook);
		printf(" > ProcessTrackDistance : %d distances, elapsed  %dms \n", distanceTracked->size(), static_cast<int32_t>(elapsedStep.GetElapseTime() * 0.001));

		// 거리 데이터 후처리
		std::vector<unsigned char> distanceLabels;
		ProcessPostDistance(*distanceTracked, *distanceTrackedHook, distanceLabels);

		// 거리데이터 예외 처리
		ProcessExceptionDistance(distanceLabels, distanceTracked, m_distanceParam, m_attitude);
		printf(" > ProcessPostDistance : %d distances, elapsed  %dms \n", distanceTracked->size(), static_cast<int32_t>(elapsedStep.GetElapseTime() * 0.001));

		// 송신 데이터 작성
		SHI::Data::StDistance *distance = new SHI::Data::StDistance;
		SHI::Convert::Distance2DataDistance(*distance, clusterPoints, clusterIndices, labels, distanceTracked, distanceLabels, m_attitude);
		distance->timeStamp = pData->timeStamp;
		distance->elapsedTimeDistance = (uint32_t)elapsedTotal.GetElapseTime()*0.001;

		// 송신
		GetStubDistance()->WriteData(distance);
		GetStubDebugDistance()->WriteData(distance);
		printf(" > Write distance : %d clusters, %d points, %d distances, total elapsed  %dms \n",
			distance->GetClusterInfoSize(), distance->GetXYZSize(), distance->GetDistanceInfoSize(), distance->elapsedTimeDistance);
		delete[] distance;
	}
}

void CDistanceProcessor::OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info)
{
	int32_t pier = m_attitude->pierId;
	int32_t crane = m_attitude->craneId;

	if (!info->attitude.IsNull())
	{
		UpdateMiniInfo(info);
	}
}
