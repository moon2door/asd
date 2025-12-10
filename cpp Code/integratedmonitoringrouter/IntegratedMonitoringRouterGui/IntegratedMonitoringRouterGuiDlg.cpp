
// IntegratedMonitoringRouterGuiDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "IntegratedMonitoringRouterGui.h"
#include "IntegratedMonitoringRouterGuiDlg.h"
#include "afxdialogex.h"

#include <Routine/Include/Base/RoutineUtility.h>
#include <Routine/include/IO/RoutineFileUtility.h>
#include <Routine/Include/Base/CTime.h>
#include <Utility/Types.h>
#include <Config/CraneInfo.h>
#include <Utility/BuildInfo.h>
#include <string>

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif
#define USER_TIMER_1HZ		1000
#define USER_TIMER_10HZ		1001
#define WM_USER_UPDATE_LIST	WM_USER + 100

// CIntegratedMonitoringRouterGuiDlg 대화 상자
CIntegratedMonitoringRouterGuiDlg::CIntegratedMonitoringRouterGuiDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_INTEGRATEDMONITORINGROUTERGUI_DIALOG, pParent)
	, m_bInit(false)
	, m_collisionManager(nullptr)
	, m_craneStatus(nullptr)
	, m_dbConnector(nullptr)
	, m_integratedMonitoringInfo(nullptr)
	, m_integratedmonitoringInterfaceManager(nullptr)
	, m_SchedulerOption(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CIntegratedMonitoringRouterGuiDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_CONNECTIONS, m_listConnection);
}

BEGIN_MESSAGE_MAP(CIntegratedMonitoringRouterGuiDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_MESSAGE(WM_USER_UPDATE_LIST, &CIntegratedMonitoringRouterGuiDlg::OnUpdateListItems)
END_MESSAGE_MAP()


// CIntegratedMonitoringRouterGuiDlg 메시지 처리기

void CIntegratedMonitoringRouterGuiDlg::OnProcessPeoridicThread()
{
	uint32_t count = 0;
	while (m_threadPRocessPeoridic.IsRunThread())
	{
		Sleep(1000);
		//printf("OnTimer\n");
		OnTimer1Sec();

		if (++count % 10 == 0)
		{
			//printf("OnTimer10\n");
			OnTimer10Sec();
		}
	}
}

BOOL CIntegratedMonitoringRouterGuiDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	//

	m_SchedulerOption = Routine::GetConfigInt("Time(0: Daily, 1:Weekly(7 days), 2:Monthly(15th of each month)", "Option", 0, "IntegratedMonitoringRouter.json");
	//pjh
	hour = Routine::GetConfigInt("Time", "Hour", 3, "IntegratedMonitoringRouter.json");
	minute = Routine::GetConfigInt("Time", "Minute", 0, "IntegratedMonitoringRouter.json");
	seconds = Routine::GetConfigInt("Time", "Seconds", 0, "IntegratedMonitoringRouter.json");
	//~pjh

	printf("IntegratedMonitoringRouterGui(ver %d) \n", SHI::BuildInfo::GetBuildDate());
	m_dbConnector = new(std::nothrow) IntegratedRouter::CProcessDBConnector();
	m_craneStatus = new(std::nothrow) CModelCraneStatus();
	m_integratedMonitoringInfo = new(std::nothrow) CModelIntegratedMonitoringInfo();
	
	//m_listConnection.SetExtendedStyle(LVS_EX_CHECKBOXES);
	m_listConnection.InsertColumn(0, "Crane Name", LVCFMT_CENTER, 100);
	m_listConnection.InsertColumn(1, "Crane IP", LVCFMT_CENTER, 150);
	m_listConnection.InsertColumn(2, "Ping", LVCFMT_CENTER, 50);
	m_listConnection.InsertColumn(3, "Connect Status", LVCFMT_CENTER, 80);
	m_listConnection.InsertColumn(4, "Port", LVCFMT_CENTER, 80);
	m_listConnection.InsertColumn(5, "Connect Node", LVCFMT_CENTER, 80);
	//m_listConnection.DeleteAllItems();
	
	// Insert list items
	printf("Insert list items\n");
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);
			std::string strPier = SHI::ConvPierStr(pier);
			std::string strCrane = SHI::ConvCraneStr(pier, crane);
			std::string name = strPier + "_" + strCrane;
			int32_t craneMonitoringPort = 10000 + pier*10 + crane;
			//pjh
			craneMonitoringPort = Routine::GetConfigInt(name.c_str(), "craneMonitoringPort", craneMonitoringPort, "IntegratedMonitoringRouter.json");
			int32_t collisionManagerPort = Routine::GetConfigInt(name.c_str(), "collisionManagerPort", 9098, "IntegratedMonitoringRouter.json");
			std::string collisionManagerIP = Routine::GetConfigString(name.c_str(), "collisionManagerIP", "127.0.0.1", "IntegratedMonitoringRouter.json");
			//

			char _ping[32] = "";
			sprintf(_ping, "X");
			char _craneIp[32] = "";
			sprintf(_craneIp, "%s(%d)", collisionManagerIP.c_str(), collisionManagerPort);
			char _serverPort[32] = "";
			sprintf(_serverPort, "%d", craneMonitoringPort);
			char _connections[32] = "";
			sprintf(_connections, "0");

			m_listConnection.InsertItem(id, "Item");
			m_listConnection.SetItemText(id, 0, name.c_str());
			m_listConnection.SetItemText(id, 1, _craneIp);
			m_listConnection.SetItemText(id, 2, _ping);
			m_listConnection.SetItemText(id, 3, "");
			m_listConnection.SetItemText(id, 4, _serverPort);
			m_listConnection.SetItemText(id, 5, _connections);

			m_updateList[id] = StUpdateItems();
		}
	}

	// DB Initialize
	printf("DB Initialize\n");
	int32_t dbPort = Routine::GetConfigInt("MongoDB", "Port", 27017, "IntegratedMonitoringRouter.json");
	std::string dbIp = Routine::GetConfigString("MongoDB", "IP", "127.0.0.1", "IntegratedMonitoringRouter.json");
	
	char _dbIp[32] = "";
	sprintf(_dbIp, "%s(%d)", dbIp.c_str(), dbPort);
	SetDlgItemText(IDC_TXT_DBIP, _dbIp);

	CString str;
	str.Format("Build DateTime : %s %s \n", __DATE__, __TIME__);
	SetDlgItemText(IDC_TXT_BUILDDATE, str);

	//pjh
	// Create collision manager
	printf("Create collision manager\n");
	m_collisionManager = new (std::nothrow)IntegratedRouter::CCollisionManager(this);
	if (m_collisionManager)
	{
		bool bCreateCollisioManager = m_collisionManager->Create();
		bCreateCollisioManager;
	}
	//

	m_dbConnector->ConnectDB(dbIp, dbPort);
	if (m_dbConnector->IsConnectedDB())
	{
		SetDlgItemText(IDC_TXT_DBCONNECTIOM, "연결됨");
	}
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);

			if (m_dbConnector)
			{
				SHI::Data::StCraneInfo info;
				bool succeed = m_dbConnector->ReadCraneInfo(id, info);
				if (!succeed)
				{
					m_dbConnector->InsertCraneInfo(id);
				}
			}
		}
	}

	// Update initial integratedMonitoringInfo
	printf("Update initial integratedMonitoringInfo\n");
	uint32_t pier = 0;
	auto pierCount = SHI::GetNumPier();

	for (uint32_t pier = 0; pier < pierCount; pier++)
	{
		auto craneCount = SHI::GetNumCrane(pier);
		for (uint32_t crane = 0; crane < craneCount; crane++)
		{
			printf(" - %s %s\n", SHI::ConvPierStr(pier).c_str(), SHI::ConvCraneStr(pier, crane).c_str());
			int32_t id = SHI::GetCraneId(pier, crane);

			if (m_dbConnector)
			{
				Routine::CTime t;
				t.UpdateCurrentTime();
				int32_t year = t.Year();

				SHI::Data::StCollisionHistory collisionHistory;
				bool succeed = m_dbConnector->ReadCollisionHistory(id, year, collisionHistory);
				if (succeed)
				{
					collisionHistory.crane = crane;
					collisionHistory.pier = pier;
					collisionHistory.year = year;
					m_integratedMonitoringInfo->UpdateCollisionHistory(id, year, collisionHistory);
				}
				else
				{
					collisionHistory.crane = crane;
					collisionHistory.pier = pier;
					collisionHistory.year = year;
					memset(collisionHistory.dayly, 0, sizeof(SHI::Data::StCollisionHistoryDaily) * 366);
					memset(collisionHistory.monthly, 0, sizeof(SHI::Data::StCollisionHistoryMonthly) * 12);
					m_integratedMonitoringInfo->UpdateCollisionHistory(id, year, collisionHistory);
				}

				SHI::Data::StOperationHistory operationHistory;
				succeed = m_dbConnector->ReadOperationHistory(id, year, operationHistory);
				if (succeed)
				{
					m_integratedMonitoringInfo->UpdateOperationHistory(id, year, operationHistory);
				}
				else
				{
					operationHistory.crane = crane;
					operationHistory.pier = pier;
					operationHistory.year = year;
					memset(operationHistory.dayly, 0, sizeof(SHI::Data::StOperationHistoryDaily) * 366);
					m_integratedMonitoringInfo->UpdateOperationHistory(id, year, operationHistory);
				}

				SHI::Data::StCraneAttitude craneAttitude;
				succeed = m_dbConnector->ReadCraneAttitude(id, craneAttitude);
				if (succeed)
				{
					m_integratedMonitoringInfo->UpdateCraneAttitude(id, craneAttitude);
				}
				else
				{
					craneAttitude.pierId = pier;
					craneAttitude.craneId = crane;
					m_integratedMonitoringInfo->UpdateCraneAttitude(id, craneAttitude);
				}
			}
		}
	}

	// Create monitoring
	printf("Create monitoring\n");
	m_integratedmonitoringInterfaceManager = new (std::nothrow)IntegratedRouter::CMonitoring(this);
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->Create();
	}

	//pjh
	//// Create collision manager
	//printf("Create collision manager\n");
	//m_collisionManager = new (std::nothrow)IntegratedRouter::CCollisionManager(this);
	//if (m_collisionManager)
	//{
	//	bool bCreateCollisioManager = m_collisionManager->Create();
	//	bCreateCollisioManager;
	//}
	//

	// Add pinger
	printf("Add pinger\n");
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);
			std::string strPier = SHI::ConvPierStr(pier);
			std::string strCrane = SHI::ConvCraneStr(pier, crane);
			std::string name = strPier + "_" + strCrane;
			int32_t port = Routine::GetConfigInt(name.c_str(), "collisionManagerPort", 9098, "IntegratedMonitoringRouter.json");
			std::string ip = Routine::GetConfigString(name.c_str(), "collisionManagerIP", "127.0.0.1", "IntegratedMonitoringRouter.json");

			AddPinger(id, ip.c_str());
		}
	}

	m_bInit = true;
	//SetTimer(USER_TIMER_1HZ, 1000, NULL);
	//SetTimer(USER_TIMER_10HZ, 10000, NULL);
	m_threadPRocessPeoridic.StartThread(&CIntegratedMonitoringRouterGuiDlg::OnProcessPeoridicThread, this);

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

BOOL CIntegratedMonitoringRouterGuiDlg::DestroyWindow()
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	if (m_dbConnector) delete m_dbConnector;
	if (m_craneStatus) delete m_craneStatus;
	if (m_integratedMonitoringInfo) delete m_integratedMonitoringInfo;

	return __super::DestroyWindow();
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CIntegratedMonitoringRouterGuiDlg::OnPaint()
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
HCURSOR CIntegratedMonitoringRouterGuiDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

//////////////////////////////////////////////////////////////////////////
// ProcessCollisionManager
void CIntegratedMonitoringRouterGuiDlg::OnServerConnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
{
	printf("OnServerConnectedMonitoring %s(%d)\n", ip.c_str(), port);
	UpdateListItemStatus(id, true);
}

void CIntegratedMonitoringRouterGuiDlg::OnServerDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
{
	printf("OnServerDisconnectedMonitoring %s(%d)\n", ip.c_str(), port);
	UpdateListItemStatus(id, false);
}

void CIntegratedMonitoringRouterGuiDlg::OnServerSystemStatus(int32_t id, SHI::Data::StSystemStatus* pSystemStatus)
{
	
	//printf("OnServerSystemStatus\n");
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendSystemStatus(id, pSystemStatus);
	}

	if (m_craneStatus)
	{
		m_craneStatus->UpdateCraneStatus(id, *pSystemStatus);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerDistance(int32_t id, SHI::Data::StDistanceSocket* pDistance)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendDistance(id, pDistance);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendDistanceCompressed(id, pDistance);

		if (SHI::GetID2Pier(id) == SHI::PIERJ ||
			SHI::GetID2Pier(id) == SHI::PIERHAN || 
			SHI::GetID2Pier(id) == SHI::PIERK || 
			SHI::GetID2Pier(id) == SHI::PIER6 ||
			SHI::GetID2Pier(id) == SHI::G2DOCK ||
			SHI::GetID2Pier(id) == SHI::G3DOCK ||
			SHI::GetID2Pier(id) == SHI::G4DOCK ||
			SHI::GetID2Pier(id) == SHI::PIERZ ||
			SHI::GetID2Pier(id) == SHI::PIERY)
		{
			m_dbConnector->InsertDistanceCompressed(id, pDistance);
		}
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerResponseRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendResponseRotorParameter(id, pRotorParameter);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerResponseCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendResponseCollisionZoneLength(id, pCollisionZoneLength);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerResponseCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendResponseCollisionZonepDecelerationRate(id, pCollisionZoneDecelerationRate);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendReportMonitoringUserInfo(id, pUserInfo);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerMiniInfo(int32_t id, SHI::Data::StCraneMiniInfo* info)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendMiniInfo(id, info);
	}

	if (m_collisionManager)
	{
		int32_t curPier = SHI::GetID2Pier(id);
		int32_t curCrane = SHI::GetID2Crane(id);
		uint32_t numCrane = SHI::GetNumCrane(curPier);

		if (curPier == SHI::PIERHAN || 
			curPier == SHI::PIER6 ||
			curPier == SHI::G2DOCK ||
			curPier == SHI::G3DOCK ||
			curPier == SHI::G4DOCK)
		{
			for (uint32_t crane = 0; crane < numCrane; crane++)
			{
				if (crane == curCrane) continue;

				int32_t _id = SHI::GetCraneId(curPier, crane);
				m_collisionManager->SendMiniInfo(_id, info);
			}
		}
	}

	if (m_integratedMonitoringInfo)
	{
		// update miniInfo
		m_integratedMonitoringInfo->UpdateMiniInfo(id, *info);

		// Update crane attitude
		SHI::Data::StCraneAttitude* attitude = new (std::nothrow) SHI::Data::StCraneAttitude;
		if (attitude)
		{
			if (m_integratedMonitoringInfo->GetCraneAttitude(id, *attitude))
			{
				*attitude = info->attitude;
				m_integratedMonitoringInfo->UpdateCraneAttitude(id, *attitude);
			}
			delete attitude;
		}

		// Update collision history
		SHI::Data::StCollisionHistory* history = new (std::nothrow) SHI::Data::StCollisionHistory;
		if (history)
		{
			Routine::CTime t;
			t.UpdateCurrentTime();
			int32_t year = t.Year();
			int32_t iydate = t.YearDay();
			int32_t imonth = t.Month() - 1;
			int32_t idate = t.Day() - 1;
			int32_t ihour = t.Hour();

			if (m_integratedMonitoringInfo->GetCollisionHistory(id, year, *history))
			{
				int32_t collisionLevel = info->collisionTotal;
				float minDistance = info->minDistance;
				history->pier = SHI::GetID2Pier(id);
				history->crane = SHI::GetID2Crane(id);
				history->year = year;
				history->monthly;

				if (history->dayly[iydate].minDistance[ihour] < FLT_EPSILON) history->dayly[iydate].minDistance[ihour] = FLT_MAX;
				if (history->monthly[imonth].minDistance[idate] < FLT_EPSILON) history->monthly[imonth].minDistance[idate] = FLT_MAX;

				if (collisionLevel == 1) history->dayly[iydate].level1[ihour] ++;
				if (collisionLevel == 2) history->dayly[iydate].level2[ihour] ++;
				if (collisionLevel == 3) history->dayly[iydate].level3[ihour] ++;
				history->dayly[iydate].minDistance[ihour] = (std::min)(history->dayly[iydate].minDistance[ihour], minDistance);

				if (collisionLevel == 1) history->monthly[imonth].level1[idate]++;
				if (collisionLevel == 2) history->monthly[imonth].level2[idate]++;
				if (collisionLevel == 3) history->monthly[imonth].level3[idate]++;
				history->monthly[imonth].minDistance[idate] = (std::min)(history->monthly[imonth].minDistance[idate], minDistance);

				m_integratedMonitoringInfo->UpdateCollisionHistory(id, year, *history);
			}
			delete history;
		}
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerMaintenanceInfo(int32_t id, SHI::Data::StMaintenanceInfo* info)
{

	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendMaintenanceInfo(id, info);
	}

	if (m_craneStatus)
	{
		m_craneStatus->UpdateCraneStatus(id, *info);
	}
}

 void CIntegratedMonitoringRouterGuiDlg::OnServerPLCInfo(int32_t id, SHI::Data::StPlcInfo* info)
{

	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendPLCInfo(id, info);
	}

	if (m_craneStatus)
	{
		m_craneStatus->UpdateCraneStatus(id, *info);
	}
}


 void CIntegratedMonitoringRouterGuiDlg::OnServerPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
{

	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendPlcControlInfo(id, pControl);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerModelDistanceSocket(int32_t id, SHI::Data::StModelDistanceSocket* distance)
{

	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendModelDistanceSocket(id, distance);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
{

	if (pCooperationMode->CooperationMode == SHI::Data::COOP_COMMAND_REQUEST)
	{
		m_dbConnector->InsertCooperation(id, pCooperationMode);
	}
	else if (
		pCooperationMode->CooperationMode == SHI::Data::COOP_COMMAND_REQUEST_ACCEPTED ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_COMMAND_ON)
	{
		m_dbConnector->InsertCooperationDone(id, pCooperationMode);
	}
	else if (
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_NONE ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_REQUEST ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_REQUEST_ACCEPTED ||
		pCooperationMode->CooperationMode == SHI::Data::COOP_INDICATING_ON)
	{
		if (m_integratedmonitoringInterfaceManager)
		{
			m_integratedmonitoringInterfaceManager->SendCooperationMode(id, pCooperationMode);
		}
	}
	else
	{
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnServerMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
{
	if (m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendMaxDistance(id, pMaxDistance);
	}
}
#ifdef TEMP_KSG
void CIntegratedMonitoringRouterGuiDlg::OnServerRotorControlSocket(int32_t id, SHI::Data::StRotorControl* pRotorControl)
{
	if(m_integratedmonitoringInterfaceManager)
	{
		m_integratedmonitoringInterfaceManager->SendRotorContorl(id, pRotorControl);
	}
}
#endif

//////////////////////////////////////////////////////////////////////////
// ProcessMonitoringInterface
void CIntegratedMonitoringRouterGuiDlg::OnClientConnectedMonitoring(int32_t id, const std::string& ip,  int32_t port)
{
	
	//printf("OnClientConnectedMonitoring %s(%d)\n", ip, port);
	if (m_updateList.find(id) != m_updateList.end())
	{
		int32_t numConnections = m_updateList[id].numConnections + 1;

		UpdateListItemNumConnections(id, numConnections);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
{

	//printf("OnClientDisconnectedMonitoring %s(%d)\n", ip, port);
	if (m_updateList.find(id) != m_updateList.end())
	{
		int32_t numConnections = m_updateList[id].numConnections - 1;

		UpdateListItemNumConnections(id, numConnections);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
{

	if (m_collisionManager)
	{

		m_collisionManager->SendCooperationMode(id, pCooperationMode);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientAlarmUse(int32_t id, const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse)
{

	if (m_collisionManager)
	{
		uint32_t uIp = Routine::IO::SocketUtility::ipToAddr(ip);
		SHI::Data::StAlarmUse alarmUse;
		alarmUse.address[0] = (uIp & 0x000000FF);
		alarmUse.address[1] = (uIp & 0x0000FF00) >> 8;
		alarmUse.address[2] = (uIp & 0x00FF0000) >> 16;
		alarmUse.address[3] = (uIp & 0xFF000000) >> 24;
		alarmUse.port = port;
		alarmUse.bAlarmUse = pAlarmUse->bAlarmUse;
		m_collisionManager->SendAlarmUse(id, &alarmUse);
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendCollisionZoneLength(id, pCollisionZoneLength);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendCollisionZoneDecelerationRate(id, pCollisionZoneDecelerationRate);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRotorParameter(id, pRotorParameter);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRotorControl(int32_t id, SHI::Data::StRotorControl* pRotorControl)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRotorControl(id, pRotorControl);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRequestRotorParameter(int32_t id)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRequestRotorParameter(id);
		
	}
}

#ifdef TEMP_KSG
void CIntegratedMonitoringRouterGuiDlg::OnClientRequestSetNewParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
{

	if(m_collisionManager)
	{
		m_collisionManager->SendRequestSetNewParameter(id, pRotorParameter);
	}
}
#endif
void CIntegratedMonitoringRouterGuiDlg::OnClientRequestCollisionZoneLength(int32_t id, SHI::Data::StCollisionRequestZone* pCollisionRequestZone)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRequestCollisionZoneLength(id, pCollisionRequestZone);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRequestCollisionZoneDecelerationRate(int32_t id)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRequestCollisionZoneDecelerationRate(id);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendReportMonitoringUserInfo(id, pUserInfo);
		
	}

	if (m_craneStatus)
	{
		m_craneStatus->UpdateCraneStatus(id, *pUserInfo);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateSensorManagerProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdateSensorManagerProc(id, pUpdateData, size);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateInterfaceProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdateInterfaceProc(id, pUpdateData, size);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateClusterProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdateClusterProc(id, pUpdateData, size);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateCollisionManagerProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdateCollisionManagerProc(id, pUpdateData, size);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateDistanceProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdateDistanceProc(id, pUpdateData, size);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateCollisionProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdateCollisionProc(id, pUpdateData, size);
		
	}
}

 void CIntegratedMonitoringRouterGuiDlg::OnClientUpdatePLCProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendUpdatePLCProc(id, pUpdateData, size);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientUpdateMonitoring(int32_t id, unsigned char* pUpdateData, uint32_t size)
{
	// 짝이 없음...
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRequestCollisionHistory(int32_t id)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRequestCollisionHistory(id);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRequestCraneAttitude(int32_t id)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRequestCraneAttitude(id);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientRequestOperationHistory(int32_t id)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendRequestOperationHistory(id);
		
	}
}

 void CIntegratedMonitoringRouterGuiDlg::OnClientPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendPlcControlInfo(id, pControl);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnClientMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
{

	if (m_collisionManager)
	{
		m_collisionManager->SendMaxDistance(id, pMaxDistance);
		
	}
}

void CIntegratedMonitoringRouterGuiDlg::UpdateListItemPing(int32_t id, bool pingConnected)
{
	if (m_updateList.find(id) != m_updateList.end())
	{
		if (m_updateList[id].pingConnected != pingConnected)
		{
			m_updateList[id].pingConnected = pingConnected;
			m_updateList[id].updatePing = true;
			PostMessage(WM_USER_UPDATE_LIST, 0, 0);
			
		}
	}
}

void CIntegratedMonitoringRouterGuiDlg::UpdateListItemStatus(int32_t id, bool connected)
{
	if (m_updateList.find(id) != m_updateList.end())
	{
		if (m_updateList[id].connected != connected)
		{
			m_updateList[id].updateConnection = true;
			m_updateList[id].connected = connected;
			PostMessage(WM_USER_UPDATE_LIST, 0, 0);
			
		}
	}
}

void CIntegratedMonitoringRouterGuiDlg::UpdateListItemNumConnections(int32_t id, int32_t numConnections)
{
	if (m_updateList.find(id) != m_updateList.end())
	{
		if (m_updateList[id].numConnections != numConnections)
		{
			m_updateList[id].updateNumConnections = true;
			m_updateList[id].numConnections = numConnections;
			PostMessage(WM_USER_UPDATE_LIST, 0, 0);
			
		}
	}
}

LRESULT CIntegratedMonitoringRouterGuiDlg::OnUpdateListItems(WPARAM wParam, LPARAM lParam)
{
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);
			std::string strPier = SHI::ConvPierStr(pier);
			std::string strCrane = SHI::ConvCraneStr(pier, crane);
			std::string name = strPier + "_" + strCrane;
			if (m_updateList.find(id) != m_updateList.end())
			{
				if (m_updateList[id].updatePing)
				{
					char _ping[32] = "";
					sprintf(_ping, "%s", m_updateList[id].pingConnected ? "O" : "X");
					m_listConnection.SetItemText(id, 2, _ping);
					m_updateList[id].updatePing = false;
				}
				if (m_updateList[id].updateConnection)
				{
					char _status[32] = "";
					sprintf(_status, "%s", m_updateList[id].connected ? "O" : "X");
					m_listConnection.SetItemText(id, 3, _status);
					m_updateList[id].updateConnection = false;
				}
				if (m_updateList[id].updateNumConnections)
				{
					char _connections[32] = "";
					sprintf(_connections, "%d", m_updateList[id].numConnections);
					m_listConnection.SetItemText(id, 5, _connections);
					m_updateList[id].updateNumConnections = false;
				}
			}
		}
		
	}
	return 0;
}

//void CIntegratedMonitoringRouterGuiDlg::OnTimer(UINT_PTR nIDEvent)
//{
//	//switch (nIDEvent)
//	//{
//	//case USER_TIMER_1HZ:
//	//	OnTimer1Sec();
//	//	break;
//	//case USER_TIMER_10HZ:
//	//	OnTimer10Sec();
//	//	break;
//	//default:
//	//	break;
//	//}
//	__super::OnTimer(nIDEvent);
//}

void CIntegratedMonitoringRouterGuiDlg::OnTimer1Sec()
{
	Routine::CTime time;
	m_bInit = false;
	time.UpdateCurrentTime();
	Routine::IO::FileInfo m_FileInfo;
	size_t size = Routine::IO::ReadFolder(m_FileInfo, "./");
	auto item = m_FileInfo.find("./\\IntegratedMonitoringScheduler.exe");
	
	
	if (item != m_FileInfo.end())
	{
		switch (m_SchedulerOption)
		{
			case 0:  // Daily
				if (time.Hour() == hour && time.Min() == minute && time.Second() == seconds)
				{
					printf("Daily start IntegratedMonitoringScheduler.exe \n");
					system("start IntegratedMonitoringScheduler.exe");
				}
				break;
			case 1:  // Weekly
				if (time.m_time.tm_wday == 0 && time.Hour() == hour && time.Min() == minute && time.Second() == seconds)
				{
					printf("Weekly start IntegratedMonitoringScheduler.exe \n");
					system("start IntegratedMonitoringScheduler.exe");
				}
				break;
			case 2: // Monthly
				if (time.Day() == 15 && time.Hour() == hour && time.Min() == minute && time.Second() == seconds)
				{
					printf("Monthly start IntegratedMonitoringScheduler.exe \n");
					system("start IntegratedMonitoringScheduler.exe");
				}
				break;
			default:
				break;
		}
	}
	else
	{
		//printf("Not Execute IntegratedMonitoringScheduler \n");
	}

	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);

			m_integratedmonitoringInterfaceManager->SendReportMonitoringHeartBeat(id);

			bool bPingConnectrd = IsPingConnected(id);
			if (bPingConnectrd)
			{
				// Send crane alive signal to monitoring
				m_integratedmonitoringInterfaceManager->SendCraneAlive(id);

				// Update operation history
				SHI::Data::StOperationHistory* hist = new (std::nothrow) SHI::Data::StOperationHistory;
				if (hist)
				{
					Routine::CTime t;
					t.UpdateCurrentTime();
					int32_t year = t.Year();
					int32_t idxDate = t.YearDay();

					if (m_integratedMonitoringInfo->GetOperationHistory(id, year, *hist))
					{
						hist->pier = pier;
						hist->crane = crane;
						hist->year = year;
						if (hist->dayly[idxDate].lastHour == 0 && hist->dayly[idxDate].lastMin == 0)
						{
							hist->dayly[idxDate].startHour = t.Hour();
							hist->dayly[idxDate].startMin = t.Min();
						}

						hist->dayly[idxDate].lastHour = t.Hour();
						hist->dayly[idxDate].lastMin = t.Min();

						m_integratedMonitoringInfo->UpdateOperationHistory(id, year, *hist);
					}
					delete hist;
				}
			}

			UpdateListItemPing(id, bPingConnectrd);

			if (m_craneStatus)
			{
				SHI::Data::StCraneStatus craneStatus;
				memset(&craneStatus, 0, sizeof(craneStatus));
				if (m_craneStatus->GetCraneStatus(id, craneStatus))
				{
					if (!craneStatus.IsNull())
					{
						if (m_dbConnector) m_dbConnector->InsertCraneStatus(id, &craneStatus);
					}
				}
			}

			
		}
	}
}

void CIntegratedMonitoringRouterGuiDlg::OnTimer10Sec()
{
	SHI::Data::StCollisionHistory* collisionHistory = new (std::nothrow) SHI::Data::StCollisionHistory;
	SHI::Data::StOperationHistory* operationHistory = new (std::nothrow) SHI::Data::StOperationHistory;
	SHI::Data::StCraneAttitude* attitude = new (std::nothrow) SHI::Data::StCraneAttitude;

	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);
			
			// Update DB
			Routine::CTime t;
			t.UpdateCurrentTime();
			int32_t year = t.Year();
			if (m_integratedMonitoringInfo)
			{
				if (collisionHistory)
				{
					if (m_integratedMonitoringInfo->GetCollisionHistory(id, year, *collisionHistory))
					{
						if (!collisionHistory->IsNull())
						{
							m_dbConnector->UpdateCollisionHistory(id, year, collisionHistory);
						}
					}
				}

				if (operationHistory)
				{
					if (m_integratedMonitoringInfo->GetOperationHistory(id, year, *operationHistory))
					{
						if (!operationHistory->IsNull())
						{
							m_dbConnector->UpdateOperationHistory(id, year, operationHistory);
						}
					}
				}

				if (attitude)
				{
					if (m_integratedMonitoringInfo->GetCraneAttitude(id, *attitude))
					{
						if (!attitude->IsNull())
						{
							m_dbConnector->UpdateCraneAttitude(id, attitude);
						}
					}
				}
			}

			// 연경 상태 확인 및 재연결
			int32_t count = m_collisionManager->GetLastRecvCount(id);
			if (count <= 0)
			{
				m_collisionManager->ResetInterface(id);
			}
			else
			{
				// 협업 모드 확인
				Routine::CTime time;
				SHI::Data::StDateTime now;
				now.year = time.Year();
				now.month = time.Month();
				now.date = time.Day();
				now.hour = time.Hour();
				now.min = time.Min();
				now.sec = time.Second();

				std::vector<SHI::Data::StCooperationMode> coopAcceptedPrevious;
				std::vector<SHI::Data::StCooperationMode> coopAcceptedNow;
				std::vector<SHI::Data::StCooperationMode> coopAcceptedFuture;
				uint32_t numCooperation = m_dbConnector->ReadNumCooperationDone(id, now.year, now.month, now.date);
				for (uint32_t i = 0; i < numCooperation; i++)
				{
					SHI::Data::StCooperationMode cooperation;
					if (m_dbConnector->ReadCooperationDone(id, now.year, now.month, now.date, cooperation, i))
					{
						if (cooperation.ContainsTime(now))
						{
							coopAcceptedNow.push_back(cooperation);
						}
						else if (now < cooperation.startTime)
						{
							coopAcceptedFuture.push_back(cooperation);
						}
						else if (cooperation.endTime < now)
						{
							coopAcceptedPrevious.push_back(cooperation);
						}
						else
						{
						}
					}
				}

				std::vector<SHI::Data::StCooperationMode> coopRequest;
				uint32_t numCooperationReq = m_dbConnector->ReadNumCooperation(id);
				for (uint32_t i = 0; i < numCooperationReq; i++)
				{
					SHI::Data::StCooperationMode cooperation;
					if (m_dbConnector->ReadCooperation(id, cooperation, i))
					{
						if (now < cooperation.endTime)
						{
							coopRequest.push_back(cooperation);
						}
					}
				}

				if (coopAcceptedNow.size())
				{
					std::sort(coopAcceptedNow.begin(), coopAcceptedNow.end());

					SHI::Data::StCooperationMode cooperation;
					cooperation = coopAcceptedNow[0];
					cooperation.CooperationMode = SHI::Data::COOP_INDICATING_ON;
					m_collisionManager->SendCooperationMode(id, &cooperation);
				}
				else if (coopAcceptedFuture.size())
				{
					std::sort(coopAcceptedFuture.begin(), coopAcceptedFuture.end());

					SHI::Data::StCooperationMode cooperation;
					cooperation = coopAcceptedFuture[0];
					cooperation.CooperationMode = SHI::Data::COOP_INDICATING_REQUEST_ACCEPTED;
					m_collisionManager->SendCooperationMode(id, &cooperation);
				}
				else if (coopRequest.size())
				{
					std::sort(coopRequest.begin(), coopRequest.end());

					SHI::Data::StCooperationMode cooperation;
					cooperation = coopRequest[0];
					cooperation.CooperationMode = SHI::Data::COOP_INDICATING_REQUEST;
					m_collisionManager->SendCooperationMode(id, &cooperation);
				}
				else
				{
					SHI::Data::StCooperationMode cooperation;
					cooperation.CooperationMode = SHI::Data::COOP_INDICATING_NONE;
					m_collisionManager->SendCooperationMode(id, &cooperation);
				}
			}
			
		}
	}

	if (collisionHistory) delete collisionHistory;
	if (operationHistory) delete operationHistory;
	if (attitude) delete attitude;
}