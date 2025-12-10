#include "stdafx.h"
#include "SensorProcessorManager.h"
#include "SensorProcessorManagerDlg.h"

// #define TEMP_YUN
#ifdef TEMP_YUN
void CSensorProcessorManagerDlg::OnCluster(SHI::Data::StCluster* pData)
{
	// 클러스터 데이터 수신
	GetProxyCluster()->ReadBuffer(pData);

	CString str;
	str.Format("%lf", pData->timeStamp);
	SetDlgItemText(IDC_TXT_PROXY_CLUSTER, str);

	uint32_t eTime = m_eTimerCluster.GetElapseTime();
	str.Format("%.3lf ms", eTime * 0.001);
	SetDlgItemText(IDC_ELAPSED_AVG, str);
}

void CSensorProcessorManagerDlg::OnLogSyncTime(SHI::Data::StLogSyncTime* pData)
{
	// LogSyncTime 데이터 수신
	GetProxyLogSyncTime()->ReadBuffer(pData);

}
#else
void CSensorProcessorManagerDlg::OnCluster(SHI::Data::StCluster* pData)
{
	// 클러스터 데이터 전송
	m_objCollisionManager->SendCluster(pData);

	CString str;
	str.Format("%lf", pData->timeStamp);
	SetDlgItemText(IDC_TXT_PROXY_CLUSTER, str);

	uint32_t eTime = m_eTimerCluster.GetElapseTime();
	str.Format("%.3lf ms", eTime * 0.001);
	SetDlgItemText(IDC_ELAPSED_AVG, str);
}

void CSensorProcessorManagerDlg::OnLogSyncTime(SHI::Data::StLogSyncTime* pData)
{
}

#endif