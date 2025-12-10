#include "stdafx.h"
#include "SensorProcessorManager.h"
#include "SensorProcessorManagerDlg.h"
#include <Data/StLogDBInfo.h>

void CSensorProcessorManagerDlg::OnConnectedSensorProcessorManager(const std::string& ip, int32_t port)
{
	SetDlgItemText(IDC_TXT_COLLISION_CONNECTION, "Connected");
}

void CSensorProcessorManagerDlg::OnDisconnectedSensorProcessorManager(const std::string& ip, int32_t port)
{
	SetDlgItemText(IDC_TXT_COLLISION_CONNECTION, "Disconnected");
}

void CSensorProcessorManagerDlg::OnRotorControlSocket(SHI::Data::StRotorControl* pRotorControl)
{
	// 회전체 컨트롤 전송
	GetStubRotorControl()->WriteData(pRotorControl);
#ifdef _DEBUG
	CString str;
	str.Format("%d:%s", pRotorControl->SensorNumber, pRotorControl->ControlMotor.bStart ? "True" : "False");
	SetDlgItemText(IDC_STATIC_DEBUG, str);
#endif
}

void CSensorProcessorManagerDlg::OnRotorParameter(SHI::Data::StRotorParameter* pRotorParameter)
{
	// 회전 변환 파라미터 전송
	GetStubRotorParameter()->WriteData(pRotorParameter);
}

void CSensorProcessorManagerDlg::OnDBInfo(SHI::Data::StDBInfo* pData)
{	
	// 동기화 시간 가져오기
	auto syncTime = new SHI::Data::StLogSyncTime;
	GetProxyLogSyncTime()->ReadBuffer(syncTime);
	m_log->Write(SHI::Data::StLogDBInfo(*syncTime, *pData));
	delete syncTime;
}

#ifdef TEMP_KSG
void CSensorProcessorManagerDlg::OnConnectedInterfaceRotor(const std::string& ip, int32_t port)
{
	printf("OnInterfaceConnected : %s(%d)\n", ip.c_str(), port);
}

void CSensorProcessorManagerDlg::OnDisconnectedInterfaceRotor(const std::string& ip, int32_t port)
{
	printf("OnInterfaceDisConnected : %s(%d)\n", ip.c_str(), port);
}


void CSensorProcessorManagerDlg::OnInterfaceRotorControl(SHI::Data::StRotorControl* pRotorControl)
{
	//printf("---------- [%d]구동기 구동 명령 = %s\n", pRotorControl->SensorNumber, pRotorControl->ControlMotor.bStart > 0 ? "시작" : "정지");//pjh
	printf("---------- [%d]InterfaceRotor Order = %s\n", pRotorControl->SensorNumber, pRotorControl->ControlMotor.bStart > 0 ? "Start" : "Stop");//pjh
	m_objCollisionManager->SendRotorControlSocket(pRotorControl);
}
#endif