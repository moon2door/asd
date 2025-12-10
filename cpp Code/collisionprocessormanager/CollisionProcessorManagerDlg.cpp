
// CollisionProcessorManagerDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"
#include "afxdialogex.h"
#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif

#include "Process_HWND_API.h"

#include <Utility/mathematics.h>
#include <Config/DistanceParameter.h>
#include <Utility/LoadCraneModel.h>


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

	// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

														// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CCollisionProcessorManagerDlg 대화 상자


CCollisionProcessorManagerDlg::CCollisionProcessorManagerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_COLLISIONPROCESSORMANAGER_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CCollisionProcessorManagerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CCollisionProcessorManagerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CCollisionProcessorManagerDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CCollisionProcessorManagerDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDOK, &CCollisionProcessorManagerDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_SHOW_DISTANCE, &CCollisionProcessorManagerDlg::OnBnClickedShowDistance)
	ON_BN_CLICKED(IDC_SHOW_COLLISION, &CCollisionProcessorManagerDlg::OnBnClickedShowCollision)
	ON_BN_CLICKED(IDC_BUTTON4, &CCollisionProcessorManagerDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CCollisionProcessorManagerDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_SHOW_PLCPROCESSOR, &CCollisionProcessorManagerDlg::OnBnClickedShowPlcprocessor)
	ON_WM_DESTROY() //pjh
END_MESSAGE_MAP()

// CCollisionProcessorManagerDlg 메시지 처리기
bool CCollisionProcessorManagerDlg::EnsureDistanceResources(int32_t pier, int32_t craneId)
{
	if (pier != m_pier || craneId < 0)
	{
		return false;
	}

	const bool hasModels = (m_refPartModelsAll.find(craneId) != m_refPartModelsAll.end());
	const bool hasParam = (m_distanceParamAll.find(craneId) != m_distanceParamAll.end());
	if (hasModels && hasParam)
	{
		return true;
	}

	SHI::DistanceModelVectorPtr models(new SHI::DistanceModelVector);
	SHI::DistanceParamPtr param(new SHI::DistanceParam);

	if (!SHI::GetDistanceParam(pier, craneId, *param))
	{
		printf("Failed to get distance param (pier=%d, crane=%d)\n", pier, craneId);
		return false;
	}

	if (!SHI::LoadDistanceModel(*models, param))
	{
		printf("Failed to load distance model (pier=%d, crane=%d)\n", pier, craneId);
		return false;
	}

	m_distanceParamAll[craneId] = param;
	m_refPartModelsAll[craneId] = models;
	return true;
}

BOOL CCollisionProcessorManagerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();


	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.

	m_compressor = new SHI::Compressor::CCompressDistance;

	m_zeroSet = false;

	// Init MathBuffer
	//SHI::Math::InitSinCosTable(0.001);

	// AHRS 센서 인터페이스 초기화
	//std::string serialId = Routine::GetConfigString("AHRS", "Port", serialId);
	//m_ahrs.Start(serialId.data());
	//m_ahrs.SetObserver(this);
	
	//pjh
	HANDLE hMutex = CreateMutex(FALSE, 0, "CollisionProcessorManager.exe");

	if (::GetLastError() == ERROR_ALREADY_EXISTS)
	{
		hMutex = INVALID_HANDLE_VALUE;
		AfxMessageBox(_T("이미 프로그램이 실행중입니다."));
		exit(0);
	}
	else
	{
		/*WNDCLASSEX wcex;
		wcex.cbSize = sizeof(WNDCLASSEX);*/

		//~pjh
		// 재시작
		memset(&m_seiRestart, 0, sizeof(SHELLEXECUTEINFO));
		m_seiRestart.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiRestart.lpFile = "cmd.exe";
		m_seiRestart.lpParameters = "/C taskkill /f /im CollisionProcessorManager.exe & start .\\CollisionProcessorManager.exe";
		m_seiRestart.nShow = SW_HIDE;
		m_seiRestart.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiRestart.lpVerb = "open";

		// CollisionProcessorManager
		memset(&m_seiCollisionManager, 0, sizeof(SHELLEXECUTEINFO));
		m_seiCollisionManager.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiCollisionManager.lpFile = "cmd.exe";
		m_seiCollisionManager.lpParameters = "/C taskkill /f /im CollisionProcessorManager.exe & del CollisionProcessorManager.exe & copy _temp_ CollisionProcessorManager.exe & del _temp_ & start .\\CollisionProcessorManager.exe";
		m_seiCollisionManager.nShow = SW_HIDE;
		m_seiCollisionManager.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiCollisionManager.lpVerb = "open";

		// DistanceProcessor
		memset(&m_seiDistance, 0, sizeof(SHELLEXECUTEINFO));
		m_seiDistance.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiDistance.lpFile = ".\\DistanceProcessor.exe";
		m_seiDistance.lpParameters = "";
		m_seiDistance.nShow = SW_HIDE;
		m_seiDistance.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiDistance.lpVerb = "open";

		// CollisionProcessor
		memset(&m_seiCollision, 0, sizeof(SHELLEXECUTEINFO));
		m_seiCollision.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiCollision.lpFile = ".\\CollisionProcessor.exe";
		m_seiCollision.lpParameters = "";
		m_seiCollision.nShow = SW_HIDE;
		m_seiCollision.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiCollision.lpVerb = "open";

		// PLCProcessor
		memset(&m_seiPLC, 0, sizeof(SHELLEXECUTEINFO));
		m_seiPLC.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiPLC.lpFile = ".\\PLCProcessor.exe";
		m_seiPLC.lpParameters = "";
		m_seiPLC.nShow = SW_HIDE;
		m_seiPLC.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiPLC.lpVerb = "open";
	}
	
	//////////////////////////////////////////////////////////////////////////
	// 거리계산 인터페이스 생성
	int32_t idStubCluster = 1;
	bool bCreatedStubCluster = GetStubCluster()->Create(idStubCluster);

	int32_t idProxyDistance = 1;
	bool bCreatedProxyDistance = GetProxyDistance()->Create(idProxyDistance);

	int32_t idStubMiniinfo = 1;
	bool bCreatedStubMiniinfo = GetStubMonitoringMiniInfo()->Create(idStubMiniinfo);
	
	//////////////////////////////////////////////////////////////////////////
	// 충돌감지 인터페이스 생성
	int32_t idStubDistance = 2;
	bool bCreatedStubDistance = GetStubDistance()->Create(idStubDistance);

	int32_t idProxyCollisionInformation = 2;
	bool bCreatedProxyCollisionInformation = GetProxyCollisionInformation()->Create(idProxyCollisionInformation);

	int32_t idStubCollisionZoneLength = 2;
	bool bCreatedStubCollisionZoneLength = GetStubCollisionZoneLength()->Create(idStubCollisionZoneLength);


	//////////////////////////////////////////////////////////////////////////
	// PLC 제어
	int32_t idStubCollisionInformation = 3;
	GetStubPlcInfo()->Create(idStubCollisionInformation);

	//////////////////////////////////////////////////////////////////////////
	// 모니터링 소켓 생성
	//char monitoringIP[32] = "127.0.0.1";pjh
	//char monitoringIPBind[32] = "127.0.0.1";
	//int32_t monitoringTCPPort = 9098;pjh
	//int32_t monitoringUDPPort = 9099;

	//pjh
	const std::string monitoringIP = Routine::GetConfigString("CraneMonitoringInterface", "IP", "127.0.0.1", "IPConfig.ini");
	int32_t monitoringTCPPort = Routine::GetConfigInt("CraneMonitoringInterface", "Port", 9098, "IPConfig.ini");
	//
	
	m_craneMonitoringInterface = new SHI::Interface::Monitoring::Server::CServerMonitoring(this);
	bool bCreatedMonitoring = m_craneMonitoringInterface->Create(monitoringIP, monitoringTCPPort,sizeof(SHI::Data::StCluster));
	
	//////////////////////////////////////////////////////////////////////////
	// PLC 소켓 생성
	int32_t plcPort = 7098;
	m_objPlcServer = new SHI::Interface::PlcInfo::Server::CServerPLCInfo(this);
	bool bCreatedPlcInfoServer = m_objPlcServer->Create(monitoringIP, plcPort, sizeof(SHI::Data::StCluster));

	//////////////////////////////////////////////////////////////////////////
	// 센서처리 매니저 인터페이스 생성
	int32_t idStubHookDB = 1;
	bool bCreatedStubHookDB = GetStubDBInfo()->Create(idStubHookDB);
#ifdef TEMP_KSG
	//////////////////////////////////////////////////////////////////////////
	/// 로터 파라미터 
	int idStubRotorParameter = 1;
	bool bCreatedStubRotorParameter = GetStubRotorParameter()->Create(idStubRotorParameter);
#endif
	//////////////////////////////////////////////////////////////////////////
	// 센서처리 Manager 소켓 생성
	//char sensorProcessorIP[32] = "192.168.100.103";
	//char sensorProcessorIPBind[32] = "192.168.100.104";
	char sensorProcessorIP[32] = "192.168.0.148";
	//char sensorProcessorIPBind[32] = "192.168.100.104";
	int32_t sensorProcessorTCPPort = 8098;
	//int32_t sensorProcessorUDPPort = 8099;

	m_objSensorManager = new SHI::Interface::SensorProcessorManager::Server::CServerSensorProcessorManger(this);
	bool bCreatedCollisionManager = m_objSensorManager->Create(sensorProcessorIP, sensorProcessorTCPPort, sizeof(SHI::Data::StCluster));


	StartTimerReportSystemStatus();
	// 
	//////////////////////////////////////////////////////////////////////////
	//char gpsAddr[256] = "192.168.1.12";

	//pjh
	/*const int32_t gpsPort = Routine::GetConfigInt("GPS", "Port", 5019);
	const std::string gpsAddr = Routine::GetConfigString("GPS", "Address", "192.168.1.12");*/
	const int32_t gpsPort = Routine::GetConfigInt("GPS", "Port", 5019, "IPConfig.ini");
	const std::string gpsAddr = Routine::GetConfigString("GPS", "Address", "192.168.1.12", "IPConfig.ini");
	//
	m_gps.SetObserver(this);
	printf("IP = %s, port = %d \n", gpsAddr.data(), gpsPort);
	m_gps.Create(gpsAddr, gpsPort);

	//////////////////////////////////////////////////////////////////////////
	//// DB 생성
	//pjh
	//int32_t use = Routine::GetConfigInt("DB", "Use", 1);
	//if (use > 0) StartTimerConnetDBMS(); 
	// 
	//TCP Connected to Server
	
	////

	RunProcessor(m_seiDistance);
	RunProcessor(m_seiCollision);

	bool bUsePlc = Routine::GetConfigInt("PLC", "bUsePlc", 0)==0? false:true;
	if (bUsePlc)
	{
		RunProcessor(m_seiPLC);
	}
	else
	{
		GetDlgItem(IDC_BUTTON5)->EnableWindow(false);
		GetDlgItem(IDC_SHOW_PLCPROCESSOR)->EnableWindow(false);
	}
	m_bControlPlc = Routine::GetConfigInt("PLC", "bControlPlc", 0) == 0 ? false : true;
	
	// 타이틀 수정
	/*
	pjhCStringA str;
	str.Format("충돌감시장치 매니저");*/
	wchar_t str[] = L"충돌감시장치 매니저";
	HWND hWnd = GetSafeHwnd();
	SetWindowTextW(hWnd, str);
	//~pjh

	CStringA strBuildDate;
	strBuildDate.Format("Build DateTime : %s %s \n", __DATE__,__TIME__);
	SetDlgItemTextA(IDC_TXT_BUILDDATETIME, strBuildDate);

	// 모델 불러오기
	m_crane = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
	std::string szPier = Routine::GetConfigString("CraneType", "Pier", "Y", "CraneType.json");
	m_pier = SHI::ConvPierInt(szPier);
	m_refPartModelsAll.clear();
	m_distanceParamAll.clear();
	if (!EnsureDistanceResources(m_pier, m_crane))
	{
		printf("Warning: failed to preload distance resources (pier=%d, crane=%d)\n", m_pier, m_crane);
	}
	
	memset(&m_userInfo, 0, m_userInfo.GetSize());

	memset(&m_windInfo, 0, m_windInfo.GetSize());

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CCollisionProcessorManagerDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CCollisionProcessorManagerDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int32_t cxIcon = GetSystemMetrics(SM_CXICON);
		int32_t cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int32_t x = (rect.Width() - cxIcon + 1) / 2;
		int32_t y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CCollisionProcessorManagerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CCollisionProcessorManagerDlg::OnBnClickedButton1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	RunProcessor(m_seiDistance);
}

void CCollisionProcessorManagerDlg::OnBnClickedButton2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	RunProcessor(m_seiCollision);
}

void CCollisionProcessorManagerDlg::OnBnClickedButton5()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	RunProcessor(m_seiPLC);
}

BOOL CCollisionProcessorManagerDlg::DestroyWindow()
{
	//pjh
	/*SafeTerminateProcess(m_seiDistance);
	SafeTerminateProcess(m_seiCollision);
	SafeTerminateProcess(m_seiPLC);*/
	SafeTerminateAllProcess();
	//

	return __super::DestroyWindow();
}

void CCollisionProcessorManagerDlg::OnBnClickedOk()
{
	SafeTerminateProcess(m_seiDistance);
	SafeTerminateProcess(m_seiCollision);
	SafeTerminateProcess(m_seiPLC);

	RunProcessor(m_seiRestart);
}

void CCollisionProcessorManagerDlg::OnBnClickedShowDistance()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (((CButton*)GetDlgItem(IDC_SHOW_DISTANCE))->GetCheck())
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiDistance.hProcess), SW_SHOWNORMAL);
		
	}
	else
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiDistance.hProcess), SW_HIDE);
	}
}

void CCollisionProcessorManagerDlg::OnBnClickedShowCollision()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (((CButton*)GetDlgItem(IDC_SHOW_COLLISION))->GetCheck())
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiCollision.hProcess), SW_SHOWNORMAL);
	}
	else
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiCollision.hProcess), SW_HIDE);
	}
}

void CCollisionProcessorManagerDlg::OnBnClickedShowPlcprocessor()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (((CButton*)GetDlgItem(IDC_SHOW_PLCPROCESSOR))->GetCheck())
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiPLC.hProcess), SW_SHOWNORMAL);
	}
	else
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiPLC.hProcess), SW_HIDE);
	}
}

void CCollisionProcessorManagerDlg::OnBnClickedButton4()
{
	m_zeroSet = true;
}
