#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"
#include <Config/CraneInfo.h>

/// <summary>
/// DistanceProcessor에게 받은 Data
/// </summary>
/// <param name="pData"></param>
void CCollisionProcessorManagerDlg::OnDistance(SHI::Data::StDistance* pData)
{
	LockDistanceSocket();

	// 충돌감시 전송
	GetStubDistance()->WriteData(pData);

	// 데이터 초기화
	memset(GetDistanceSocket(), 0, sizeof(SHI::Data::StDistanceSocket));
	GetDistanceSocket()->InitBuffer();
	GetDistanceSocket()->AllocXYZ(pData->GetXYZSize());
	GetDistanceSocket()->AllocClusterIndices(pData->GetClusterIndicesSize());
	GetDistanceSocket()->AllocClusterInfo(pData->GetClusterInfoSize());
	GetDistanceSocket()->AllocLabel(pData->GetLabelSize());
	GetDistanceSocket()->AllocDistanceInfo(pData->GetDistanceInfoSize());
	GetDistanceSocket()->AllocDistanceLabel(pData->GetDistanceLabelSize());
	

	SHI::Data::StXYZSocket* pSrcXYZ = GetDistanceSocket()->GetXYZ();
	SHI::Data::StXYZ* pXYZ = pData->GetXYZ();
	uint32_t nXYZ = pData->GetXYZSize();

	GetDistanceSocket()->timeStamp = pData->timeStamp;
	GetDistanceSocket()->elapsedTimeClustering = pData->elapsedTimeClustering;
	GetDistanceSocket()->elapsedTimeDistance = pData->elapsedTimeDistance;
	GetDistanceSocket()->attitude = pData->attitude;
	
	// GPS 정보
	//pjh 240509
	//GetDistanceSocket()->gpsLat = m_dbConnector->GetLat();
	//GetDistanceSocket()->gpsLon = m_dbConnector->GetLon();
	GetDistanceSocket()->gpsLat = pData->attitude.gpsLatitude;
	GetDistanceSocket()->gpsLon = pData->attitude.gpsLongitude;
	//

	// 가변버퍼 데이터 복사
	for (uint32_t i = 0; i < nXYZ; i++)
	{
		pSrcXYZ[i].X = short(pXYZ[i].X * 100.f);
		pSrcXYZ[i].Y = short(pXYZ[i].Y * 100.f);
		pSrcXYZ[i].Z = short(pXYZ[i].Z * 100.f);
	}
	memcpy(GetDistanceSocket()->GetClusterIndices(), pData->GetClusterIndices(), sizeof(uint32_t) * pData->GetClusterIndicesSize());
	memcpy(GetDistanceSocket()->GetClusterInfo(), pData->GetClusterInfo(), sizeof(SHI::Data::StClusterInfo) * pData->GetClusterInfoSize());
	memcpy(GetDistanceSocket()->GetLabel(), pData->GetLabel(), sizeof(unsigned char) * pData->GetLabelSize());
	memcpy(GetDistanceSocket()->GetDistanceInfo(), pData->GetDistanceInfo(), sizeof(SHI::Data::StDistanceInfo) * pData->GetDistanceInfoSize());
	memcpy(GetDistanceSocket()->GetDistanceLabel(), pData->GetDistanceLabel(), sizeof(unsigned char) * pData->GetDistanceLabelSize());	

	UnLockDistanceSocket();


	// 버전 정보
	GetSystemStatus().Version.Distance = pData->version;

	CString str;
	str.Format("%lf", pData->timeStamp);
	SetDlgItemText(IDC_TIMESTAMP_DISTANCE, str);

	uint32_t eTime = m_eTimerDistance.GetElapseTime();
	str.Format("%.3lf ms", eTime * 0.001);
	SetDlgItemText(IDC_TIME_DISTANCE, str);
}
