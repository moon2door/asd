#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"
#include <direct.h>
#include <ctime>

void CCollisionProcessorManagerDlg::OnConnectedMonitoring(const std::string& ip, int32_t port)
{
	//if (ip ==  "192.168.207.207")
	//{
		SetDlgItemText(IDC_TXT_MONITOR_CONNECTION, "Connected");
	//}
}

void CCollisionProcessorManagerDlg::OnDisconnectedMonitoring(const std::string& ip, int32_t port)
{
	//if (ip == "192.168.207.207")
	//{
		SetDlgItemText(IDC_TXT_MONITOR_CONNECTION, "Disconnected");
	//}
}

void CCollisionProcessorManagerDlg::OnCooperationMode(SHI::Data::StCooperationMode* pCooperationMode)
{
	unsigned char curMode = GetSystemStatus().StatusCooperationMode;

	if (pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_NONE ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_REQUEST ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_REQUEST_ACCEPTED ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_ON)
	{
		GetSystemStatus().StatusCooperationMode = pCooperationMode->CooperationMode;
	}

	m_craneMonitoringInterface->SendCooperationMode(pCooperationMode);
}

void CCollisionProcessorManagerDlg::OnAlarmUse(const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse)
{
	GetSystemStatus().bUseAlarm = pAlarmUse->bAlarmUse;

	Routine::CTime time;
	char fname[256] = "";
	
	_mkdir("./AlarmControlLog");
	sprintf_s(fname, "./AlarmControlLog/%04d-%02d-%02d.txt", time.Year(), time.Month(), time.Day(), time.Day());

	FILE* fp = fopen(fname, "at");
	if(fp)
	{

		//pjh sprintf_s(m_address, sizeof(m_address), "%d.%d.%d.%d", pAlarmUse->address[0], pAlarmUse->address[1], pAlarmUse->address[2], pAlarmUse->address[3]);
		if(pAlarmUse->bAlarmUse == 1)
		{
			fprintf(fp, "%d:%d:%d: Alarm \"Use\", from ip %d.%d.%d.%d\n", time.Hour(), time.Min(), time.Second(), pAlarmUse->address[0], pAlarmUse->address[1], pAlarmUse->address[2], pAlarmUse->address[3]);//ip.c_str());//pjh
		}
		else
		{
			fprintf(fp, "%d:%d:%d: Alarm \"Not use\", from ip %d.%d.%d.%d\n", time.Hour(), time.Min(), time.Second(), pAlarmUse->address[0], pAlarmUse->address[1], pAlarmUse->address[2], pAlarmUse->address[3]);//ip.c_str());//pjh
		}
		fclose(fp);
	}
}

void CCollisionProcessorManagerDlg::OnCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZone)
{
	GetStubCollisionZoneLength()->WriteData(pCollisionZone);
	UpdateCollisionZoneLength(pCollisionZone);
}

void CCollisionProcessorManagerDlg::OnCollisionZoneDecelerationRate(SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
{
	GetStubCollisionZoneDecelerationRate()->WriteData(pCollisionZoneDecelerationRate);
	UpdateCollisionZoneDecelerationRate(pCollisionZoneDecelerationRate);
}

void CCollisionProcessorManagerDlg::OnRequestCollisionZoneLength(SHI::Data::StCollisionRequestZone* pCollisionRequestZone)
{
	m_craneMonitoringInterface->SendResponseCollisionZoneLength(&GetCollisionZoneLength(pCollisionRequestZone->nZone));
}

void CCollisionProcessorManagerDlg::OnRequestCollisionZoneDecelerationRate()
{
	m_craneMonitoringInterface->SendResponseCollisionZonepDecelerationRate(&GetCollisionZoneDecelerationRate());
}

void CCollisionProcessorManagerDlg::OnRotorParameter(SHI::Data::StRotorParameter* pRotorParameter)
{
	m_objSensorManager->SendRotorParameter(pRotorParameter);
}

void CCollisionProcessorManagerDlg::OnRotorControl(SHI::Data::StRotorControl* pRotorControl)
{
#ifdef TEST
	printf("%d:%s\n", pRotorControl->SensorNumber, pRotorControl->ControlMotor.bStart ? "True" : "False");
#endif
	m_objSensorManager->SendRotorControl(pRotorControl);
}

void CCollisionProcessorManagerDlg::OnRequestRotorParameter()
{
	//m_objMonitoringSendResponseRotorParameter(&GetRotorStatus().RotorParameter);
}
#ifdef TEMP_KSG
void CCollisionProcessorManagerDlg::OnRequestSetNewParameter(SHI::Data::StRotorParameter* pRotorParameter)
{
	//char* pTempData = new char[1500];
	//memcpy(pTempData, pRotorParameter, sizeof(pRotorParameter));
	//UpdateRotorParameter(pRotorParameter);
	//GetStubRotorParameter()->WriteData(pRotorParameter);
	m_objSensorManager->SendRotorParameter(pRotorParameter);
}
#endif
void CCollisionProcessorManagerDlg::OnMonitoringUserInfo(SHI::Data::StMonitoringUserInfo* pUserInfo)
{
	if (pUserInfo->Authority == 1)
	{
		m_bAuthor = pUserInfo->Authority;
		CString text;
		text.Format("%s(%s)", pUserInfo->ID, pUserInfo->Name);
		SetDlgItemText(IDC_TXT_MONITOR_USERINFO, text.GetString());
		m_userInfo = *pUserInfo;
	}
}

void CCollisionProcessorManagerDlg::OnWindInfo(SHI::Data::StWindInfo* info)
{
	m_windInfo = *info;
}

void CCollisionProcessorManagerDlg::OnPlcControlInfo(SHI::Data::StPlcControlInfo* pControl)
{
	m_bControlPlc = pControl->bPlcControl;

	int32_t ctrl = 0;
	if (m_bControlPlc) ctrl = 1;
	Routine::SetConfigInt("PLC", "bControlPlc", ctrl);
}

void CCollisionProcessorManagerDlg::OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info)
{
	routerFrames()++;

	printf("OnMonitoringMiniInfo (%d, %d)\n", info->attitude.pierId, info->attitude.craneId);

	GetStubMonitoringMiniInfo()->WriteData(info);

	if (!info->attitude.IsNull())
	{
		UpdateMiniInfo(info);

		// Calc model distance
		if (info->attitude.pierId == m_pier && info->attitude.craneId == m_crane)
		{
			// skip
		}
		else
		{
			int32_t id = SHI::GetCraneId(m_pier, m_crane);
			SHI::CraneAttitudePtr myAttitude(new SHI::CraneAttitude(GetCraneMiniInfo(id).attitude));
			SHI::CraneAttitudePtr otherAttitude(new SHI::CraneAttitude(info->attitude));

			if (!myAttitude->IsNull())
			{
				SHI::Data::StModelDistanceSocket distance;
				std::vector<unsigned char> distanceModelLabels;
				//if (ProcessModelDistance(distance, myAttitude, otherAttitude))
				{
					m_craneMonitoringInterface->SendModelDistanceSocket(&distance);
				}
			}
		}
	}
}

void CCollisionProcessorManagerDlg::OnMaxDistance(SHI::Data::StMaxDistance* pMaxDistance)
{
	Routine::SetConfigDouble("Viewer Distance", "Max(m)", pMaxDistance->maxDistance);
	printf("OnMaxDistance = %f \n", pMaxDistance->maxDistance);
}

