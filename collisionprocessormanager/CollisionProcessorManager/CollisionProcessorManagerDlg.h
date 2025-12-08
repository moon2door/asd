
// CollisionProcessorManagerDlg.h : 헤더 파일
//

#pragma once
// 
// Interface
#include <Interface/Cluster/Stub/StubCluster.h>
#include <Interface/Distance/Proxy/ProxyDistance.h>
#include <Interface/CraneModel/Stub/StubCraneModel.h>
#include <Interface/Distance/Stub/StubDistance.h>
#include <Interface/DBInfo/Stub/StubDBInfo.h>
#include <Interface/CollisionInformation/Proxy/ProxyCollisionInformation.h>
#include <interface/CollisionZoneLength/Stub/StubCollisionZoneLength.h>
#include <interface/CollisionZoneDecelerationRate/Stub/StubCollisionZoneDecelerationRate.h>
#include <Interface/CollisionInformation/Stub/StubCollisionInformation.h>
#include <Interface/SensorProcessorManager/Server/ServerSensorProcessorManager.h>
#include <Interface/Monitoring/Server/ServerMonitoring.h>
#include <Interface/PLCInfo/Server/ServerPLCInfo.h>
#include <Interface/PLCInfo/Stub/StubPlcInfo.h>
#include <Interface/MonitoringMiniInfo/Stub/StubMonitoringMiniInfo.h>
#ifdef TEMP_KSG
#include <Interface/RotorParameter/Stub/StubRotorParameter.h>
#endif
#include <Data/StDistanceSocket.h>
#include <Utility/Compressor.h>
#include <Utility/Types.h>

// Status
#ifdef TEMP_KSG
#include "ModelRotorParameter.h"
#endif
#include "ModelCooperationMode.h"
#include "ModelSystemStatus.h"
#include "ModelCollisionZone.h"
//pjh#include "ModelDBMSConnector.h"
#include "ModelDistanceSocket.h"
#include "ModelCraneMiniInfo.h"


// DevLib
#include <Routine/Include/Base/RoutineUtility.h>
#include <Utility/BuildInfo.h>
#include <Routine/include/Routines/Compressor/CCompressor.h>
#include <Routine/Include/Base/CElapseTimer.h>
#include <Utility/mathematics.h>

// Device
//#include <device/myahrs_plus/InterfaceAhrsPlus.h>
//#include <device/SEC225/InterfaceSEC225.h>
#include <device/EncoderGXM7S/EncoderGXM7S.h>
#include <device/GpsTdr2000/InterfaceGpsTdr2000.h>

// CCollisionProcessorManagerDlg 대화 상자
class CCollisionProcessorManagerDlg 
	: public CDialogEx
	//////////////////////////////////////////////////////////////////////////
	// 거리계산 
	, public SHI::Interface::Cluster::Stub::CStubCluster
	, public SHI::Interface::Distance::Proxy::CProxyDistance
	, public SHI::Interface::DBInfo::Stub::CStubDBInfo
	, public SHI::Interface::MonitoringMiniInfo::Stub::CStubMonitoringMiniInfo
	//////////////////////////////////////////////////////////////////////////
	// 충돌감지
	, public SHI::Interface::Distance::Stub::CStubDistance
	, public SHI::Interface::CollisionInformation::Proxy::CProxyCollisionInformation
	, public SHI::Interface::CollisionZoneLength::Stub::CStubCollisionZoneLength
	, public SHI::Interface::CollisionZoneDecelerationRate::Stub::CStubCollisionZoneDecelerationRate
	//////////////////////////////////////////////////////////////////////////
	// PLC 제어기
	, public SHI::Interface::PlcInfo::Server::CObserverPLCInfo
	, public SHI::Interface::PlcInfo::Stub::CStubPlcInfo
	//////////////////////////////////////////////////////////////////////////
	// 센서처리
	, public SHI::Interface::SensorProcessorManager::Server::CObserverServerSensorProcessorManager
	//////////////////////////////////////////////////////////////////////////
	// 모니터링
	, public SHI::Interface::Monitoring::Server::CObserverServerMonitoring
	//////////////////////////////////////////////////////////////////////////
	// Status
	, public CModelCooperationMode
	, public CModelSystemStatus
	, public CModelCollisionZone
	//pjh, public CModelDBMSConnector
	, public CModelDistanceSocket
	, public CModelCraneMiniInfo
#ifdef TEMP_KSG
	, public CModelRotorParameter
	//////////////////////////////////////////////////////////////////////////
	, public SHI::Interface::RotorParameter::Stub::CStubRotorParameter
#endif
	//////////////////////////////////////////////////////////////////////////
	// AHRS 센서
	//, public SHI::Device::MyAhrsPlus::CObserverAhrsPlus
	//, public Device::SEC225::CObserverSEC225
	, public Device::EncoderGXM7S::CObserverEncoderGXM7S
	, public SHI::Device::TDR2000::CObserverGpsTdr2000
{
// 생성입니다.
public:
	CCollisionProcessorManagerDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_COLLISIONPROCESSORMANAGER_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


	//////////////////////////////////////////////////////////////////////////
	// 센서처리 Manager 수신 : Socket
	SHI::Interface::SensorProcessorManager::Server::CServerSensorProcessorManger*	m_objSensorManager;
	void OnConnectedSensorProcessorManager(const std::string& ip, int32_t port) override;	// 연결 알림
	void OnDisconnectedSensorProcessorManager(const std::string& ip, int32_t port) override; // 연결 끊김 알림
	void OnRotorStatus(SHI::Data::StRotorStatus* pRotorStatus) override;	// Rotor Status 수신
	void OnCluster(SHI::Data::StCluster* pClusterData) override;	// Cluster 수신
	void OnRotorControlSocket(SHI::Data::StRotorControl* pRotorControl) override; // 구동 정지 시작 수신

	//////////////////////////////////////////////////////////////////////////
	// 모니터링 장치 수신 : Socket
	SHI::Interface::Monitoring::Server::CServerMonitoring* m_craneMonitoringInterface;
	//pjh SHI::Interface::Monitoring::Server::CServerMonitoring* m_objMonitoring;
	/// <summary>
	/// Connect to CraneMonitoringInterface
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	void OnConnectedMonitoring(const std::string& ip, int32_t port) override;
	void OnDisconnectedMonitoring(const std::string& ip, int32_t port)override;
	void OnCooperationMode(SHI::Data::StCooperationMode* pCooperationMode)override;
	void OnAlarmUse(const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse)override;
	void OnCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZone)override;
	void OnCollisionZoneDecelerationRate(SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)override;
	void OnRotorParameter(SHI::Data::StRotorParameter* pRotorParameter)override;
	void OnRotorControl(SHI::Data::StRotorControl* pRotorControl)override;
	void OnRequestRotorParameter()override;
#ifdef TEMP_KSG
	void OnRequestSetNewParameter(SHI::Data::StRotorParameter* pRotorParameter)override;
#endif
	void OnRequestCollisionZoneLength(SHI::Data::StCollisionRequestZone* pCollisionRequestZone)override;
	void OnRequestCollisionZoneDecelerationRate()override;
	void OnMonitoringUserInfo(SHI::Data::StMonitoringUserInfo* pUserInfo)override;
	void OnWindInfo(SHI::Data::StWindInfo* info)override;

	void OnPlcControlInfo(SHI::Data::StPlcControlInfo* pControl)override;
	void OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info)override;
	void OnMaxDistance(SHI::Data::StMaxDistance* pMaxDistance)override;


	//////////////////////////////////////////////////////////////////////////
	// AHRS 센서
	virtual void OnDataEncoderGXM7S(unsigned int count, float angle);

	//////////////////////////////////////////////////////////////////////////
	// 거리계산 처리
	void OnDistance(SHI::Data::StDistance* pData);

	//////////////////////////////////////////////////////////////////////////
	// 충돌감지 처리
	void OnCollisionInformation(SHI::Data::StCollisionInformation* pData);

	//////////////////////////////////////////////////////////////////////////
	// PLC 제어
	virtual void OnConnectedPLCInfo(const std::string& ip, int32_t port);
	virtual void OnDisconnectedPLCInfo(const std::string& ip, int32_t port);
	virtual void OnPlcInfo(SHI::Data::StPlcInfo* info);

	SHI::Interface::PlcInfo::Server::CServerPLCInfo* m_objPlcServer;

	//////////////////////////////////////////////////////////////////////////
	// Timer 시스템 상태
	void OnTimerReportSystemStatus();
	//pjhvoid OnUpdatedDistance();

	//////////////////////////////////////////////////////////////////////////
	// GPS 수신
	virtual void OnGpsData(SHI::Device::TDR2000::StGpsData& data);
	virtual void OnGpsConnected(const std::string& ip, int32_t port);
	virtual void OnGpsDisConnected(const std::string& ip, int32_t port);

	//////////////////////////////////////////////////////////////////////////
	// ProcessorUpdate
	virtual void OnUpdateSensorManagerProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdateInterfaceProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdateClusterProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdateCollisionManagerProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdateDistanceProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdateCollisionProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdatePLCProc(unsigned char* pUpdateData, unsigned int size);
	virtual void OnUpdateMonitoring(unsigned char* pUpdateData, unsigned int size);

	//////////////////////////////////////////////////////////////////////////
	// 압축
	SHI::Compressor::CCompressDistance *m_compressor;

	//////////////////////////////////////////////////////////////////////////
	// 모델 기반 거리 계산
	bool ProcessModelDistance(SHI::Data::StModelDistanceSocket& distance, const SHI::CraneAttitudePtr& myAttitude, const SHI::CraneAttitudePtr& otherAttitude);
	std::map<int, SHI::DistanceModelVectorPtr> m_refPartModelsAll;
	std::map<int, SHI::DistanceParamPtr> m_distanceParamAll;
	int m_pier;
	int m_crane;
	bool EnsureDistanceResources(int32_t pier, int32_t craneId);

private:
	SHELLEXECUTEINFO m_seiRestart;
	SHELLEXECUTEINFO m_seiCollisionManager;
	SHELLEXECUTEINFO m_seiDistance;
	SHELLEXECUTEINFO m_seiCollision;
	SHELLEXECUTEINFO m_seiPLC;
	void RunProcessor(SHELLEXECUTEINFO& sei);
	BOOL SafeTerminateProcess(SHELLEXECUTEINFO& sei);
	void SafeTerminateAllProcess();//pjh

	bool UpdateProcessor(SHELLEXECUTEINFO& sei, unsigned char* pData, unsigned int size);

	//////////////////////////////////////////////////////////////////////////
	// DB 연동	
	//pjh
	/*void OnTimerConnetDBMS();
	void OnTimerUpdateCraneInfo();
	void OnTimerUpdateCooperation();*/
	//

	Routine::CElapseTimer	m_eTimerCluster;
	Routine::CElapseTimer	m_eTimerDistance;
	Routine::CElapseTimer	m_eTimerCollisionInfo;
	Routine::CElapseTimer	m_eTimerDebug;

	// AHRS 센서
	//SHI::Device::MyAhrsPlus::CInterfaceAhrsPlus m_ahrs;
	//Device::SEC225::CInterfaceSEC225 m_ahrs;
	Device::EncoderGXM7S::CInterfaceEncoderGXM7S m_ahrs;

	// GPS
	SHI::Device::TDR2000::CInterfaceGpsTdr2000 m_gps;

	//
	SHI::Data::StMonitoringUserInfo m_userInfo;

	SHI::Data::StWindInfo m_windInfo;

	//status Log
	char m_address[256];

// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton5();
	virtual BOOL DestroyWindow();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedShowDistance();
	afx_msg void OnBnClickedShowCollision();
	afx_msg void OnBnClickedShowPlcprocessor();

	afx_msg void OnBnClickedButton4();

	bool m_zeroSet;
	bool m_bControlPlc;
	bool m_bAuthor;
	bool m_bMonitoring;
	bool m_bGpsConnected;
	
	afx_msg void OnBnClickedShowCollision2();
};
