#include "stdafx.h"
#include "SensorProcessorManager.h"
#include "SensorProcessorManagerDlg.h"

#ifdef _DEBUG
#include <cmath>
#endif
void CSensorProcessorManagerDlg::OnXYZPoint(SHI::Data::StXYZPoints* pData)
{
	// 클러스터 처리기에 데이터 전송
	auto *db = new SHI::Data::StDBInfo;
	memset(db, 0, sizeof(SHI::Data::StDBInfo));
	if (GetProxyDBInfo()->ReadBuffer(db))
	{
#ifdef _DEBUG
		char test_str[256];
		float Latitude;
		float Lat_decimalPart = std::modf(db->latitude, &Latitude);
		Lat_decimalPart *= 60;
		float Longitude;
		float Long_decimalPart = std::modf(db->logitude, &Longitude);
		Long_decimalPart *= 60;
		//sprintf(test_str, "la:%d.%d, lo:%d.%d, hgt:%d\n", (int)Latitude, (int)std::round(Lat_decimalPart), (int)Longitude, (int)std::round(Long_decimalPart), (int)db->hook.gps.height);
		//sprintf(test_str, "lat:%.2f, lon:%.2f, hgt:%.1f\n", db->latitude, db->logitude, db->hook.gps.height);
		
		//pjh
		//sprintf(test_str, "la:%.06f,lo:%.06f,az:%.02f\n", db->latitude, db->logitude, db->hook.gps.azimuth);
		sprintf(test_str, "latitude:%.06f, logitude:%.06f, azimuth:%.02f, quality:%d\n", db->latitude, db->logitude, db->hook.gps.azimuth, db->hook.gps.quality);
		//

		SetDlgItemText(IDC_STATIC_DEBUG, test_str);

#endif
		GetStubDBInfo()->WriteData(db);
	}
	delete db;

	GetStubXYZPoint()->WriteData(pData);

	CString str;
	str.Format("%lf", pData->timeStamp);
	SetDlgItemText(IDC_TXT_PROXY_XYZPOINT, str);

	uint32_t eTime = m_eTimerInterface.GetElapseTime();
	str.Format("%.3lf ms", eTime * 0.001);
	SetDlgItemText(IDC_ELAPSED_TICK, str);
}

void CSensorProcessorManagerDlg::OnRotorStatus(SHI::Data::StRotorStatus* pData)
{
	pData->VersionInterfacecRotor = SHI::BuildInfo::GetBuildDate();
	int32_t ret = m_objCollisionManager->SendRotorStatus(pData);
}

void CSensorProcessorManagerDlg::OnRotorControl(SHI::Data::StRotorControl* pData)
{
//#ifndef TEMP_KSG
	//printf("---------- [%d]구동기 구동 명령 = %s\n", pData->SensorNumber, pData->ControlMotor.bStart > 0 ? "시작" : "정지");
	int32_t ret = m_objCollisionManager->SendRotorControlSocket(pData);
//#endif
}
