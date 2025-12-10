
// SensorProcessorManagerDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "SensorProcessorManager.h"
#include "SensorProcessorManagerDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include "Process_HWND_API.h"

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


// CSensorProcessorManagerDlg 대화 상자



CSensorProcessorManagerDlg::CSensorProcessorManagerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SENSORPROCESSORMANAGER_DIALOG, pParent)
	, m_objCollisionManager(nullptr)
	, m_objInterfaceRotor(nullptr)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	::ZeroMemory(&m_seiRestart, sizeof(SHELLEXECUTEINFO));
	::ZeroMemory(&m_seiSensorManager, sizeof(SHELLEXECUTEINFO));
	::ZeroMemory(&m_seiInterface, sizeof(SHELLEXECUTEINFO));
	::ZeroMemory(&m_seiCluster, sizeof(SHELLEXECUTEINFO));
}

void CSensorProcessorManagerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CSensorProcessorManagerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CSensorProcessorManagerDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CSensorProcessorManagerDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_SHOW_INTERFACE, &CSensorProcessorManagerDlg::OnBnClickedShowInterface)
	ON_BN_CLICKED(IDC_SHOW_CLUSTER, &CSensorProcessorManagerDlg::OnBnClickedShowCluster)
	ON_BN_CLICKED(IDC_RESTART, &CSensorProcessorManagerDlg::OnBnClickedRestart)
	ON_WM_DESTROY()//pjh
END_MESSAGE_MAP()


// CSensorProcessorManagerDlg 메시지 처리기


void CSensorProcessorManagerDlg::OnInterfaceInitSequence()
{
	m_timerInterfaceInitSequence.StopTimer();
	while (SafeTerminateProcess(m_seiInterface) == FALSE) Sleep(100);
	Sleep(3000);
	RunProcessor(m_seiInterface);
}

void CSensorProcessorManagerDlg::OnTimer1sec()
{
	Routine::CTime time; 
	time.UpdateCurrentTime();

	Routine::CTime t = time - m_timerRunningTime;
	

	char text[256] = "";
	sprintf(text, "%d시간 %d분 %d초", t.Hour(), t.Min(), t.Second());
	SetDlgItemText(IDC_TXT_RUNNINGTIME, text);
}


BOOL CSensorProcessorManagerDlg::OnInitDialog()
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
	//DevLib::Utility::Console::ConsoleOpen("디버그");

	m_timerRunningTime.UpdateCurrentTime();
	m_timer1sec.StartTimer(1000, &CSensorProcessorManagerDlg::OnTimer1sec, this);

	HANDLE hMutex = CreateMutex(FALSE, 0, "SensorProcessorManager.exe");
	if (::GetLastError() == ERROR_ALREADY_EXISTS)
	{
		hMutex = INVALID_HANDLE_VALUE;
		AfxMessageBox(_T("이미 프로그램이 실행중입니다."));
		exit(0);
	}
	else
	{
		//// Restart
		memset(&m_seiRestart, 0, sizeof(SHELLEXECUTEINFO));
		m_seiRestart.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiRestart.lpFile = "cmd.exe";
		m_seiRestart.lpParameters = "/C taskkill /f /im SensorProcessorManager.exe & start .\\SensorProcessorManager.exe";
		m_seiRestart.nShow = SW_HIDE;
		m_seiRestart.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiRestart.lpVerb = "open";

		// SensorManager
		memset(&m_seiSensorManager, 0, sizeof(SHELLEXECUTEINFO));
		m_seiSensorManager.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiSensorManager.lpFile = "cmd.exe";
		m_seiSensorManager.lpParameters = "/C taskkill /f /im SensorProcessorManager.exe & del SensorProcessorManager.exe & copy _temp_ SensorProcessorManager.exe & del _temp_ & start .\\SensorProcessorManager.exe";
		m_seiSensorManager.nShow = SW_HIDE;
		m_seiSensorManager.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiSensorManager.lpVerb = "open";

		// InterfaceProcessor
		memset(&m_seiInterface, 0, sizeof(SHELLEXECUTEINFO));
		m_seiInterface.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiInterface.lpFile = ".\\InterfaceRotor.exe";
		m_seiInterface.lpParameters = "";
		m_seiInterface.nShow = SW_HIDE;
		m_seiInterface.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiInterface.lpVerb = "open";

		// ClusterProcessor
		memset(&m_seiCluster, 0, sizeof(SHELLEXECUTEINFO));
		m_seiCluster.cbSize = sizeof(SHELLEXECUTEINFO);
		m_seiCluster.lpFile = ".\\ClusterProcessor.exe";
		m_seiCluster.lpParameters = "";
		m_seiCluster.nShow = SW_HIDE;
		m_seiCluster.fMask = SEE_MASK_NOCLOSEPROCESS;
		m_seiCluster.lpVerb = "open";
	}
	

	// SHM 인터페이스 생성
	int idProxyXYZPoint = 1;//DevLib::Utility::Config::GetConfigInt("Interface", "Proxy_XYZPoint", 1, DevLib::Utility::Config::GetMakeConfigName());
	bool bCreatedProxyXYZPoint = GetProxyXYZPoint()->Create(idProxyXYZPoint);

	int idProxyRotorStatus = 1;//DevLib::Utility::Config::GetConfigInt("Interface", "Proxy_RotorStatus", 1, DevLib::Utility::Config::GetMakeConfigName());
	bool bCreatedProxyRotorStatus = GetProxyRotorStatus()->Create(idProxyRotorStatus);

	int idStubRotorControl = 1;//DevLib::Utility::Config::GetConfigInt("Interface", "Stub_RotorControl", 1, DevLib::Utility::Config::GetMakeConfigName());
	bool bCreatedStubRotorControl = GetStubRotorControl()->Create(idStubRotorControl);

	int idProxyRotorControl = 2;//DevLib::Utility::Config::GetConfigInt("Interface", "Stub_RotorControl", 1, DevLib::Utility::Config::GetMakeConfigName());
	bool bidProxyRotorControl = GetProxyRotorControl()->Create(idProxyRotorControl);
#ifdef TEMP_KSG
	//int idProxyRotorParameter = 11; //DevLib::Utility::Config::GetConfigInt("Interface", "Stub_RotorControl", 1, DevLib::Utility::Config::GetMakeConfigName());
	//bool bCreatedProxyRotorParameter = GetProxyRotorParameter()->Create(idProxyRotorParameter);
#endif
	int idStubRotorParameter = 1;//DevLib::Utility::Config::GetConfigInt("Interface", "Stub_RotorParameter", 1, DevLib::Utility::Config::GetMakeConfigName());
	bool bCreatedStubRotorParameter = GetStubRotorParameter()->Create(idStubRotorParameter);

	int idStubXYZPoint = 2;//DevLib::Utility::Config::GetConfigInt("ClusterProcessor", "Stub_XYZPoint", 2, DevLib::Utility::Config::GetMakeConfigName());
	bool bCreatedStubXYZPoint = GetStubXYZPoint()->Create(idStubXYZPoint);

	int idProxyCluster = 2;//DevLib::Utility::Config::GetConfigInt("ClusterProcessor", "Proxy_Cluster", 2, DevLib::Utility::Config::GetMakeConfigName());
	bool bCreatedProxyCluster = GetProxyCluster()->Create(idProxyCluster);

	int idProxyLogSyncTime = 1;
	bool bCreatedProxyLogSyncTime = GetProxyLogSyncTime()->Create(idProxyLogSyncTime);

	int idProxyHookInfoDB = 1;
	bool bCreatedProxyHookInfoDB = GetProxyDBInfo()->Create(idProxyHookInfoDB);

	int idStubDBInfo = 2;
	bool bCreateStubDBInfo = GetStubDBInfo()->Create(idStubDBInfo);


	// Collision Manager 소켓 생성
	//std::string dstIP = "192.168.100.104";
	//char bindIP[32] = "192.168.100.103";
	//int udpPort = 8099;
	std::string dstIP = Routine::GetConfigString("CollisionManager", "IP_DST", "192.168.100.104");
	int tcpPort = 8098;
#ifdef TEMP_KSG
	std::string srcIP = "127.0.0.1";
	int srcPort = 6098;
	m_objInterfaceRotor = new SHI::Interface::InterfaceRotor::Client::CClientInterfaceRotor(this);
	bool bCreatedClientInterfaceRotor = m_objInterfaceRotor->Create(srcIP, srcPort, sizeof(SHI::Data::StRotorControl));
#endif
	//DevLib::Utility::Config::GetConfigString("CollisionManager", "IP_BIND", bindIP, 32, bindIP, DevLib::Utility::Config::GetMakeConfigName());
	//tcpPort = DevLib::Utility::Config::GetConfigInt("CollisionManager", "TCP_Port", tcpPort, DevLib::Utility::Config::GetMakeConfigName());
	//udpPort = DevLib::Utility::Config::GetConfigInt("CollisionManager", "UDP_Port", udpPort, DevLib::Utility::Config::GetMakeConfigName());

	m_objCollisionManager = new SHI::Interface::SensorProcessorManager::Client::CClientSensorProcessorManager(this);
	bool bCreatedClientCollisionManager = m_objCollisionManager->Create(dstIP, tcpPort, sizeof(SHI::Data::StCluster));

	RunProcessor(m_seiInterface);
	RunProcessor(m_seiCluster);

	//int useRestartInterface = DevLib::Utility::Config::GetConfigInt("Interface", "Restart", 0, DevLib::Utility::Config::GetMakeConfigName());

	//if (useRestartInterface) StartInterfaceInitSequence();

	// 타이틀 수정 
	unsigned int ver = SHI::BuildInfo::GetBuildDate();
	CString str;
	str.Format("센서처리 매니저");
	SetWindowText(str);
	CString strBuildDate;
	strBuildDate.Format("Build DateTime : %s %s \n", __DATE__, __TIME__);
	SetDlgItemText(IDC_TXT_BUILDDATETIME, strBuildDate);

	// DB 로그 기능
	m_log->Initialize("./dblog", "dblog", 1080);

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CSensorProcessorManagerDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CSensorProcessorManagerDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

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
HCURSOR CSensorProcessorManagerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CSensorProcessorManagerDlg::OnBnClickedButton1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	RunProcessor(m_seiInterface);
}

void CSensorProcessorManagerDlg::OnBnClickedButton2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	RunProcessor(m_seiCluster);
}

BOOL CSensorProcessorManagerDlg::DestroyWindow()
{
	//pjh
	/*SafeTerminateProcess(m_seiInterface);
	SafeTerminateProcess(m_seiCluster);*/

	SafeTerminateAllProcess();
	m_log.reset();
	//

	return __super::DestroyWindow();
}


void CSensorProcessorManagerDlg::OnBnClickedShowInterface()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (((CButton*)GetDlgItem(IDC_SHOW_INTERFACE))->GetCheck())
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiInterface.hProcess), SW_SHOWNORMAL);
	}
	else
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiInterface.hProcess), SW_HIDE);
	}
}


void CSensorProcessorManagerDlg::OnBnClickedShowCluster()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (((CButton*)GetDlgItem(IDC_SHOW_CLUSTER))->GetCheck())
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiCluster.hProcess), SW_SHOWNORMAL);
	}
	else
	{
		::ShowWindow(GetHwndFromProcessHandle(m_seiCluster.hProcess), SW_HIDE);
	}
}


void CSensorProcessorManagerDlg::OnBnClickedRestart()
{
	SafeTerminateProcess(m_seiInterface);
	SafeTerminateProcess(m_seiCluster);

	RunProcessor(m_seiRestart);
}