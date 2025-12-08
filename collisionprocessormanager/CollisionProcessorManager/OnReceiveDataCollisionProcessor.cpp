#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"
#include <Data/StDistanceSocketCompressed.h>
#include <Utility/Types.h>
#include <Config/CraneInfo.h>
#include <Routine/Include/Base/CElapseTimer.h>
#include <Routine/Include/Base/RoutineUtility.h>
#include <stdio.h>
#include <Routine/Include/Routines/Compressor/CCompressor.h>
#include <Utility/mathematics.h>

int32_t Collision2Plc(int32_t collisionLevel)
{
	int32_t level = 3 - collisionLevel;
	if (level < 0 || level > 3)
	{
		level = SHI::Data::PLC_DECEL_NONE;
	}
	return level;
}

/// <summary>
/// CollisionProcessor Data
/// </summary>
/// <param name="pData"></param>
void CCollisionProcessorManagerDlg::OnCollisionInformation(SHI::Data::StCollisionInformation* pData)
{
	// 버전 정보
	GetSystemStatus().Version.Collision = pData->version;

	CString str;
	str.Format("%lf", pData->timeStamp);
	SetDlgItemText(IDC_TIMESTAMP_COLLISIONINFO, str);

	uint32_t eTime = m_eTimerCollisionInfo.GetElapseTime();
	str.Format("%.3lf ms", eTime * 0.001);
	SetDlgItemText(IDC_TIME_COLLISION, str);

	if (GetDistanceSocket()->timeStamp != pData->timeStamp)
	{
		printf("Error => Data Throw because Timestamp Diff \n");
		return;
	}

	bool bCompression = Routine::GetConfigInt("DistanceSocket", "bCompression", 1) == 0 ? false : true;

	SHI::Data::StDistanceSocket *src = new (std::nothrow)SHI::Data::StDistanceSocket;
	if (src)
	{	
		LockDistanceSocket();
		GetDistanceSocket()->AllocCollisionInfo(pData->GetCollisionInfoSize());
		GetDistanceSocket()->AllocReasonInfo(pData->GetReasonInfoSize());
		GetDistanceSocket()->collisionTotal = pData->collisionTotal;
		memcpy(GetDistanceSocket()->GetCollisionInfo(), pData->GetCollisionInfo(), pData->GetCollisionInfoSize());
		memcpy(GetDistanceSocket()->GetReasonInfo(), pData->GetReasonInfo(), pData->GetReasonInfoSize());
		memset(src, 0, sizeof(SHI::Data::StDistanceSocket));
		memcpy(src, GetDistanceSocket(), GetDistanceSocket()->GetSize());
		UnLockDistanceSocket();
		if (!bCompression)
		{
			m_craneMonitoringInterface->SendDistance(src);
		}
		else
		{
			SHI::Data::StDistanceSocketCompressed *compressed = new (std::nothrow)SHI::Data::StDistanceSocketCompressed;
			if (compressed)
			{
				if (m_compressor->Encode(*compressed, *src))
				{
					m_craneMonitoringInterface->SendDistanceCompressed(compressed);
				}
				delete compressed;
			}
		}
		delete src;
	}

	// MiniInfo 전송
	SHI::Data::StCraneMiniInfo *info = new (std::nothrow)SHI::Data::StCraneMiniInfo;
	if (info)
	{
		memset(info, 0, sizeof(SHI::Data::StCraneMiniInfo));
		LockDistanceSocket();
		info->attitude = GetDistanceSocket()->attitude;
		info->collisionTotal = GetDistanceSocket()->collisionTotal;

		float minDistance = FLT_MAX; // 현재 Frame의 최소 거리	
		for (uint32_t i = 0; i < GetDistanceSocket()->GetDistanceInfoSize(); i++)
		{
			if (GetDistanceSocket()->GetDistanceLabel()[i] == SHI::DISTANCE_NORMAL)
			{
				minDistance = (std::min)(minDistance, GetDistanceSocket()->GetDistanceInfo()[i].Distance);
			}
		}
		info->minDistance = minDistance;
		UnLockDistanceSocket();

		m_craneMonitoringInterface->SendMiniInfo(info);

		UpdateMiniInfo(info);
		delete info;
	}

	// 운전자 정보 전송
	m_craneMonitoringInterface->SendReportMonitoringUserInfo(&m_userInfo);

	
}