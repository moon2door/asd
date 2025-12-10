#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"

void CCollisionProcessorManagerDlg::OnConnectedSensorProcessorManager(const std::string& ip, int32_t port)
{
	SetDlgItemText(IDC_TXT_SENSOR_CONNECTION, "Connected");
}

void CCollisionProcessorManagerDlg::OnDisconnectedSensorProcessorManager(const std::string& ip, int32_t port)
{
	SetDlgItemText(IDC_TXT_SENSOR_CONNECTION, "Disconnected");
}

void CCollisionProcessorManagerDlg::OnRotorStatus(SHI::Data::StRotorStatus* pRotorStatus)
{
	UpdateRotorStatus(pRotorStatus);
	SHI::Data::StRotorParameter* data = new SHI::Data::StRotorParameter;
	for (int nLidar = 0; nLidar < pRotorStatus->NumSensor; nLidar++)
	{
		for (int nSize = 0; nSize < 16; nSize++)
		{
			data->Sensor[nLidar].intrinsicsMatrix[nSize] = pRotorStatus->RotorParameter[nLidar].intrinsicsMatrix[nSize];
			data->Sensor[nLidar].extrinsicsMatrix[nSize] = pRotorStatus->RotorParameter[nLidar].extrinsicsMatrix[nSize];
		}
	}	
 	m_craneMonitoringInterface->SendCollisionResponseRotorParameter(data);
	delete data;
}

void CCollisionProcessorManagerDlg::OnCluster(SHI::Data::StCluster* pClusterData)
{
	// 거리계산 처리기 에 전송
	GetStubCluster()->WriteData(pClusterData);

	// 버전 정보 업데이트 
	GetSystemStatus().Version.Cluster = pClusterData->version;

	CString str;
	str.Format("%lf", pClusterData->timeStamp);
	SetDlgItemText(IDC_TIMESTAMP_CLUSTER, str);
	
	uint32_t eTime = m_eTimerCluster.GetElapseTime();
	str.Format("%.3lf ms", eTime * 0.001);
	SetDlgItemText(IDC_TIME_CLUSTER, str);
	
}

void CCollisionProcessorManagerDlg::OnRotorControlSocket(SHI::Data::StRotorControl* pRotorControl)
{
	//printf("---------- [%d]구동기 구동 명령 = %s\n", pRotorControl->SensorNumber, pRotorControl ->ControlMotor.bStart> 0 ? "시작" : "정지");//pjh
	printf("---------- [%d]InterfaceRotor Order = %s\n", pRotorControl->SensorNumber, pRotorControl->ControlMotor.bStart > 0 ? "Start" : "Stop");//pjh
	m_craneMonitoringInterface->SendRotorControlSocket(pRotorControl);
}

