
#include "IntegratedMonitoringInterface.h"
#include <Routine/Include/Base/RoutineUtility.h>
#include <Config/CraneInfo.h>
#include <Utility/Types.h>
#include <Utility/Transform.h>
#include <Utility/Filter.h>
#include "../colorTable.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <../Config/CraneGps.h>//pjh

CIntegratedMonitoringInterface::CIntegratedMonitoringInterface()
{
	m_timer.StartTimer(1000, &CIntegratedMonitoringInterface::OnTimer, this);

	uint32_t numPiers = SHI::GetNumPier();
	m_status = new StCraneStatusData[numPiers];
	for (uint32_t i = 0; i < numPiers; ++i)
	{
		//m_status[i].pierId = SHI::GetPierId(i);
		m_status[i].pierId = static_cast<SHI::PIER>(i);
	}

	m_IntegratedMonitoringRouter = new (std::nothrow)CMonitoringRouter;
	if (m_IntegratedMonitoringRouter)
	{
		m_IntegratedMonitoringRouter->SetObserver(this);
	}
	m_integratedmonitoringUnity = new (std::nothrow)CMonitoring;
	if (m_integratedmonitoringUnity)
	{
		m_integratedmonitoringUnity->SetObserver(this);
	}

}

CIntegratedMonitoringInterface::~CIntegratedMonitoringInterface()
{
	m_timer.StopTimer();
	delete[] m_status;
	delete m_IntegratedMonitoringRouter;
	delete m_integratedmonitoringUnity;
}

bool CIntegratedMonitoringInterface::Create(int32_t monitoringPort)
{
	bool createRouter = false;
	bool createMonitoring = false;
	bool createDbConnector = false;
	if (m_IntegratedMonitoringRouter && m_integratedmonitoringUnity)
	{
		bool createRouter = m_IntegratedMonitoringRouter->Create();

		//pjh TODO
		// Test후 주석을 살리고 수정된 코드를 삭제
		// 라우터 포트포워딩을 하여 라우터와 통신하는 것으로 추정
		std::string integratedMonitoringUnityIP = Routine::GetConfigString("integratedMonitoringUnity", "IP", "127.0.0.1");
		bool createMonitoring = m_integratedmonitoringUnity->Create(integratedMonitoringUnityIP, monitoringPort);
		//bool createMonitoring = m_integratedmonitoringUnity->Create(monitoringPort);
		//

		char IP[64] = "60.100.91.175";
		std::string dbIP = Routine::GetConfigString("DB", "IP", IP);
		int32_t dbPort = Routine::GetConfigInt("DB", "Port", 27017);
		createDbConnector = m_dbConnector.ConnectDB(dbIP, dbPort);

		m_dbPlayer = new CProcessDBPlayer(m_dbConnector, this);

		int32_t colorMode = Routine::GetConfigInt("Test", "ColorMode", 0);

		SetColorMode(colorMode);
	}
	return (createRouter && createMonitoring && createDbConnector);
}

void CIntegratedMonitoringInterface::Test()
{
	SHI::Data::StDateTime t;
	t.year = 2021;
	t.month = 6;
	t.date = 15;
	t.hour = 12;
	t.min = 05;
	m_dbPlayer->StartPlay(SHI::PIERHAN, t);
}

void CIntegratedMonitoringInterface::OnDBPlayerDistance(int32_t pier, int32_t crane, SHI::Data::StDistanceSocket* pDistance)
{
	OnRouterDistance(pier, crane, pDistance);
}

void CIntegratedMonitoringInterface::OnDBPlayerInfo(SHI::Data::StDateTime start, SHI::Data::StDateTime end, SHI::Data::StDateTime cur)
{
	StIndicateLogPlay data;
	data.pierId = 3;
	data.craneId = 0;
	data.startYear = start.year;
	data.startMonth = start.month;
	data.startDay = start.date;
	data.startHour = start.hour;
	data.startMinute = start.min;
	data.startSec = start.sec;
	data.endYear = end.year;
	data.endMonth = end.month;
	data.endDay = end.date;
	data.endHour = end.hour;
	data.endMinute = end.min;
	data.endSec = end.sec;
	data.curYear = cur.year;
	data.curMonth = cur.month;
	data.curDay = cur.date;
	data.curHour = cur.hour;
	data.curMinute = cur.min;
	data.curSec = cur.sec;
	int ret = m_integratedmonitoringUnity->SendLogPlayInfo(&data);
	printf("OnDBPlayerInfo - Send = %d \n", ret);
}

void CIntegratedMonitoringInterface::OnCraneStatus(int32_t pier, int32_t crane, SHI::Data::StCraneStatus* pStatus)
{
	OnRouterPLCInfo(pier, crane, &pStatus->PLCInfo);
	OnRouterSystemStatus(pier, crane, &pStatus->SystemStatus);
	pStatus->DriverInfo;
	pStatus->maintenanceInfo;
}

void CIntegratedMonitoringInterface::OnConnectedIntegratedMonitoring()
{
	printf("OnConnected IntegratedMonitoring\n");

	Routine::CTime t;
	t.UpdateCurrentTime();
	StRequestCooperationList data;
	data.year = t.Year();
	data.month = t.Month();
	data.date = t.Day();

	OnRequestCooperationList(&data);
}

void CIntegratedMonitoringInterface::OnDisconnectedIntegratedMonitoring()
{
	printf("OnDisconnected IntegratedMonitoring\n");
}

void CIntegratedMonitoringInterface::OnRequestCollisionHistory(StRequestCollisionHistory* pData)
{
	Routine::CTime t;
	t.UpdateCurrentTime();
	int32_t year = pData->year;
	int32_t id = SHI::GetCraneId(pData->pierId, pData->craneId);

	StCollisionHistory* histroy = new(std::nothrow) StCollisionHistory(pData->pierId, pData->craneId);
	if (histroy && m_integratedmonitoringUnity)
	{
		if (GetCollisionHistory(histroy, id, year))
		{
			m_integratedmonitoringUnity->SendCollisionHistory(histroy);
		}
		delete histroy;
	}
	Sleep(10);

	StOperationHistory* histroyOperatoin = new(std::nothrow) StOperationHistory(pData->pierId, pData->craneId);
	if (histroyOperatoin && m_integratedmonitoringUnity)
	{
		if (GetOperationHistory(histroyOperatoin, id, year))
		{
			m_integratedmonitoringUnity->SendOperationHistory(histroyOperatoin);
		}

		delete histroyOperatoin;
	}
}

void CIntegratedMonitoringInterface::OnCooperationMode(StCooperationMode* pData)
{
	SHI::Data::StCooperationMode cooperation;
	cooperation.pier = pData->pierId;
	cooperation.crane = pData->craneId;
	cooperation.CooperationMode = pData->cooperationMode;

	Routine::CTime t;
	cooperation.timestamp.year = t.Year();
	cooperation.timestamp.month = t.Month();
	cooperation.timestamp.date = t.Day();
	cooperation.timestamp.hour = t.Hour();
	cooperation.timestamp.min = t.Min();
	cooperation.timestamp.sec = t.Second();

	cooperation.startTime.year = pData->startYear;
	cooperation.startTime.month = pData->startMonth;
	cooperation.startTime.date = pData->startDate;
	cooperation.startTime.hour = pData->startHour;
	cooperation.startTime.min = pData->startMin;
	cooperation.startTime.sec = pData->startSec;

	cooperation.endTime.year = pData->endYear;
	cooperation.endTime.month = pData->endMonth;
	cooperation.endTime.date = pData->endDate;
	cooperation.endTime.hour = pData->endHour;
	cooperation.endTime.min = pData->endMin;
	cooperation.endTime.sec = pData->endSec;

	cooperation.operatorInfo.Authority = pData->authority;
	memcpy(cooperation.operatorInfo.ID, pData->id, 32);
	memcpy(cooperation.operatorInfo.Name, pData->name, 32);
	memcpy(cooperation.operatorInfo.Reserved, pData->reserved, 32);

	if (SHI::IsValidPier(cooperation.pier) && SHI::IsValidCrane(cooperation.pier, cooperation.crane))
	{
		int32_t id = SHI::GetCraneId(cooperation.pier, cooperation.crane);

		if (cooperation.CooperationMode == SHI::Data::COOP_COMMAND_REQUEST_ACCEPTED)
		{
			m_dbConnector.InsertCooperationDone(id, &cooperation);
		}
		else
		{
			m_dbConnector.InsertCooperation(id, &cooperation);
		}
	}

	StRequestCooperationList data;
	data.year = cooperation.startTime.year;
	data.month = cooperation.startTime.month;
	data.date = cooperation.startTime.date;
	OnRequestCooperationList(&data);
}

void CIntegratedMonitoringInterface::OnRequestCooperationList(StRequestCooperationList* pData)
{
	std::vector<SHI::Data::StCooperationMode> cooperations;
	uint32_t numRequest = 0;
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		if (pier != SHI::PIERHAN) continue;

		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);

			// DB 정보 조회(협업 요청 이력)
			int32_t sizeReq = m_dbConnector.ReadNumCooperation(id);
			for (uint32_t i = 0; i < sizeReq; i++)
			{
				SHI::Data::StCooperationMode data;
				if (m_dbConnector.ReadCooperation(id, data, i))
				{
					printf("# [%d] Cooperation / %d:%d ~ %d:%d (%s) \n", i, data.startTime.hour, data.startTime.min, data.endTime.hour, data.endTime.min, data.operatorInfo.Name);
					data.CooperationMode = SHI::Data::COOP_INDICATING_REQUEST;
					cooperations.push_back(data);
				}
			}

			numRequest += sizeReq;

			// DB 정보 조회(협업 승인 이력)
			Routine::CTime t;
			t.m_time.tm_year = pData->year;
			t.m_time.tm_mon = pData->month;
			t.m_time.tm_mday = pData->date;
			int32_t sizeAccept = m_dbConnector.ReadNumCooperationDone(id, t.Year(), t.Month(), t.Day());
			for (uint32_t i = 0; i < sizeAccept; i++)
			{
				SHI::Data::StCooperationMode data;
				if (m_dbConnector.ReadCooperationDone(id, t.Year(), t.Month(), t.Day(), data, i))
				{
					printf("# [%d - %d] Cooperation done / %d:%d ~ %d:%d (%s) \n", id, i, data.startTime.hour, data.startTime.min, data.endTime.hour, data.endTime.min, data.operatorInfo.Name);
					data.CooperationMode = SHI::Data::COOP_INDICATING_REQUEST_ACCEPTED;
					cooperations.push_back(data);
				}
			}

			// 협업 취소 이력 조회
			int32_t sizeCancle = m_dbConnector.ReadNumCooperationCancled(id, t.Year(), t.Month(), t.Day());
			for (uint32_t i = 0; i < sizeCancle; i++)
			{
				SHI::Data::StCooperationMode data;
				if (m_dbConnector.ReadCooperationCancled(id, t.Year(), t.Month(), t.Day(), data, i))
				{
					data.CooperationMode = SHI::Data::COOP_COMMAND_NONE;
					cooperations.push_back(data);
				}
			}
		}
	}

	StCooperationModeList *sendData = new (std::nothrow)StCooperationModeList();
	if (sendData)
	{
		uint32_t size = (cooperations.size() > 30) ? (30) : (cooperations.size());
		for (uint32_t i = 0; i < size; i++)
		{
			sendData->list[i].pierId = cooperations[i].pier;
			sendData->list[i].craneId = cooperations[i].crane;
			sendData->list[i].cooperationMode = 0;
			if (cooperations[i].CooperationMode == SHI::Data::COOP_INDICATING_REQUEST)
			{
				sendData->list[i].cooperationMode = StCooperationMode::COOP_REQUEST;
			}
			else if (cooperations[i].CooperationMode == SHI::Data::COOP_INDICATING_REQUEST_ACCEPTED)
			{
				sendData->list[i].cooperationMode = StCooperationMode::COOP_ACCEPT;
			}
			else
			{
				sendData->list[i].cooperationMode = StCooperationMode::COOP_REFUSE;
			}

			sendData->list[i].startYear = cooperations[i].startTime.year;
			sendData->list[i].startMonth = cooperations[i].startTime.month;
			sendData->list[i].startDate = cooperations[i].startTime.date;
			sendData->list[i].startHour = cooperations[i].startTime.hour;
			sendData->list[i].startMin = cooperations[i].startTime.min;
			sendData->list[i].startSec = cooperations[i].startTime.sec;
			sendData->list[i].endYear = cooperations[i].endTime.year;
			sendData->list[i].endMonth = cooperations[i].endTime.month;
			sendData->list[i].endDate = cooperations[i].endTime.date;
			sendData->list[i].endHour = cooperations[i].endTime.hour;
			sendData->list[i].endMin = cooperations[i].endTime.min;
			sendData->list[i].endSec = cooperations[i].endTime.sec;
			sendData->list[i].authority = cooperations[i].operatorInfo.Authority;
			memcpy(sendData->list[i].id, cooperations[i].operatorInfo.ID, 32);
			memcpy(sendData->list[i].name, cooperations[i].operatorInfo.Name, 32);
			memcpy(sendData->list[i].reserved, cooperations[i].operatorInfo.Reserved, 32);
		}
		sendData->pierId = 3;
		sendData->craneId = 0;
		sendData->numList = size;
		m_integratedmonitoringUnity->SendCooperationModeList(sendData);

		delete sendData;
	}

	SetNumCooperation(numRequest);
}

void CIntegratedMonitoringInterface::OnRequestCooperationAccept(StRequestCooperationAccept* pData)
{
	if (SHI::IsValidPier(pData->pierId) && SHI::IsValidCrane(pData->pierId, pData->craneId))
	{
		int32_t id = SHI::GetCraneId(pData->pierId, pData->craneId);
		int32_t idx = pData->codeIndex;
		if (pData->acceptCode == StRequestCooperationAccept::COOP_ACCEPT)
		{
			SHI::Data::StCooperationMode cooperation;
			if (m_dbConnector.ReadCooperation(id, cooperation, idx))
			{
				cooperation.CooperationMode = SHI::Data::COOP_COMMAND_REQUEST_ACCEPTED;
				memcpy(cooperation.operatorInfo.ID, pData->id, 32);
				memcpy(cooperation.operatorInfo.Name, pData->id, 32);
				memset(cooperation.operatorInfo.Password, 0, 32);
				memset(cooperation.operatorInfo.Reserved, 0, 32);
				m_dbConnector.InsertCooperationDone(id, &cooperation);
			}
			m_dbConnector.RemoveCooperation(id, idx);
		}
		else if (pData->acceptCode == StRequestCooperationAccept::COOP_REFUSE)
		{
			m_dbConnector.RemoveCooperation(id, idx);
		}
		else {}
	}

	Routine::CTime t;
	t.UpdateCurrentTime();
	StRequestCooperationList data;
	data.year = t.Year();
	data.month = t.Month();
	data.date = t.Day();
}

void CIntegratedMonitoringInterface::OnRequestCooperationRemove(StRequestCooperationRemove* pData)
{
	if (SHI::IsValidPier(pData->pierId) && SHI::IsValidCrane(pData->pierId, pData->craneId))
	{
		int32_t id = SHI::GetCraneId(pData->pierId, pData->craneId);
		int32_t idx = pData->codeIndex;

		SHI::Data::StCooperationMode cooperation;
		if (m_dbConnector.ReadCooperationDone(id, pData->year, pData->month, pData->date, cooperation, idx))
		{
			m_dbConnector.RemoveCooperationDone(id, pData->year, pData->month, pData->date, idx);

			memcpy(cooperation.operatorInfo.ID, pData->id, 32);
			memcpy(cooperation.operatorInfo.Name, pData->id, 32);
			memset(cooperation.operatorInfo.Password, 0, 32);
			memset(cooperation.operatorInfo.Reserved, 0, 32);
			m_dbConnector.InsertCooperationCancled(id, &cooperation);
		}
	}

	Routine::CTime t;
	t.UpdateCurrentTime();
	StRequestCooperationList data;
	data.year = pData->year;
	data.month = pData->month;
	data.date = pData->date;
}

void CIntegratedMonitoringInterface::OnRequesAddUser(StRequstAddUser* pData)
{
	SHI::Data::StLoginInfo info;
	memcpy(info.id, pData->id, 32);
	memcpy(info.password, pData->password, 32);
	memcpy(info.info, pData->info, 31);
	info.additionalInfo = pData->admin;
	m_dbConnector.InsertLoginInfo(&info);
}

void CIntegratedMonitoringInterface::OnRequesLogin(StRequstLogin* pData)
{
	std::string cmpId(pData->id);
	std::string cmpPass(pData->password);
	//printf("input ID: %s, PW: %s\n", cmpId.c_str(), cmpPass.c_str());
	bool bAccept = false;
	char admin = 0;

	std::vector<SHI::Data::StLoginInfo> referanceInfo;
	if (m_dbConnector.ReadLoginInfo(referanceInfo))
	{
		for (uint32_t i = 0; i < referanceInfo.size(); i++)
		{
			std::string refId(referanceInfo[i].id);
			std::string refPass(referanceInfo[i].password);
	
			//printf("ID: %s, PW: %s\n", refId.c_str(), refPass.c_str());
			if (cmpId.compare(refId) == 0 && cmpPass.compare(refPass) == 0)
			{
				admin = referanceInfo[i].additionalInfo;
				bAccept = true;
				break;
			}
		}
	}

	if (m_integratedmonitoringUnity)
	{
		StReplyLogin reply;
		if (bAccept)
		{
			memcpy(reply.userid, pData->id, 32);
			reply.code = 0;

			if(admin) reply.code = 2;
			
			printf("LoginAccepted\n");
		}
		else
		{
			memcpy(reply.userid, pData->id, 32);
			reply.code = 1;
			printf("LoginRefused\n");
		}

		m_integratedmonitoringUnity->SendReplyLogin(&reply);
	}
}

void CIntegratedMonitoringInterface::OnRequestLogPlay(StRequestLogPlay* pData)
{
	printf("OnRequestLogPlay (%d-%d) %d, %d, %d, %d, %d, %d \n", pData->pierId, pData->code, pData->year, pData->month, pData->day, pData->hour, pData->minute, pData->sec);
	
	if (pData->code == 0) // LOGPLAY_OPEN = 0
	{
		SHI::Data::StDateTime t(pData->year, pData->month, pData->day, pData->hour, pData->minute, pData->sec);
		
		m_dbPlayer->StartLoading(pData->pierId, t);

		m_IntegratedMonitoringRouter->SetObserver(0);
		m_bLogPlay = true;
	}
	else if (pData->code == 1) // LOGPLAY_START = 1
	{
		SHI::Data::StDateTime t(pData->year, pData->month, pData->day, pData->hour, pData->minute, pData->sec);
		m_dbPlayer->StartPlay(pData->pierId, t);
		m_IntegratedMonitoringRouter->SetObserver(0);
		m_bLogPlay = true;
	}
	else if (pData->code == 2) // LOGPLAY_PAUSE = 2
	{
		SHI::Data::StDateTime t(pData->year, pData->month, pData->day, pData->hour, pData->minute, pData->sec);
		m_dbPlayer->StopPlay();
		m_IntegratedMonitoringRouter->SetObserver(0);
		m_bLogPlay = true;
	}
	else if (pData->code == 3) // LOGPLAY_READY = 3
	{
		m_IntegratedMonitoringRouter->SetObserver(0);
		m_dbPlayer->StopPlay();
		m_bLogPlay = true;
	}
	else if (pData->code == 4) // LOGPLAY_CLOSE = 4
	{
		m_IntegratedMonitoringRouter->SetObserver(this);
		m_dbPlayer->StopPlay();
		m_bLogPlay = false;
	}
	else
	{
	}
}

void CIntegratedMonitoringInterface::OnRouterConnected(int32_t pier, int32_t crane, const std::string& ip, int32_t port)
{
	//printf("OnConnected Router %s %s(%s, %d) \n", SHI::ConvPierStr(pier).c_str(), SHI::ConvCraneStr(pier, crane).c_str(), ip, port);

	int32_t id = SHI::GetCraneId(pier, crane);
	m_IntegratedMonitoringRouter->SendRequestCraneAttitude(id);
	m_IntegratedMonitoringRouter->SendRequestCollisionHistory(id);
	m_IntegratedMonitoringRouter->SendRequestOperationHistory(id);
}

void CIntegratedMonitoringInterface::OnRouterDisconnected(int32_t pier, int32_t crane, const std::string& ip, int32_t port)
{
	//printf("OnDisconnected Router %s %s(%s, %d) \n", SHI::ConvPierStr(pier).c_str(), SHI::ConvCraneStr(pier, crane).c_str(), ip, port);
}

bool IsValidGps(SHI::Data::StCraneAttitude& attitude)
{
	bool ret = false;
	if (attitude.gpsTimestamp > 0 &&
		attitude.gpsLatitude > FLT_EPSILON &&
		attitude.gpsLongitude > FLT_EPSILON  &&
		attitude.quality >= 4// && pjh
		//abs(attitude.azimuth) > FLT_EPSILON pjh
		)
	{
		ret = true;
	}
	return ret;
}

bool IsValidGps(SHI::CraneAttitude& attitude)
{
	bool ret = false;
	if (attitude.gpsTimestamp > 0 &&
		attitude.gpsLatitude > FLT_EPSILON &&
		attitude.gpsLongitude > FLT_EPSILON  &&
		attitude.quality >= 4 //&& pjh
		//abs(attitude.azimuth) > FLT_EPSILON pjh
		)
	{
		ret = true;
	}
	return ret;
}

void CIntegratedMonitoringInterface::OnRouterDistance(int32_t pier, int32_t crane, SHI::Data::StDistanceSocket* pDistance)
{
	if (pDistance->attitude.pierId == 5 ||
		pDistance->attitude.pierId == 6 ||
		pDistance->attitude.pierId == 7  )
	{
		pDistance->attitude.azimuth = -pDistance->attitude.azimuth;
	}


	if (pier == SHI::PIERJ ||
		pier == SHI::PIERK || 
		pier == SHI::PIERHAN || 
		pier == SHI::PIER6 ||
		pier == SHI::G2DOCK ||
		pier == SHI::G3DOCK ||
		pier == SHI::G4DOCK ||
		pier == SHI::PIERZ || //pjh
		pier == SHI::PIERY //Y
		)
	{
		int32_t id = SHI::GetCraneId(pier, crane);

		UpdateCraneAttitude(id, pDistance->attitude);
		SHI::CraneAttitude attitude;
		memcpy_s(&attitude, sizeof(attitude), &pDistance->attitude, sizeof(pDistance->attitude));
		if (IsValidGps(attitude))
		{
			//printf("%lf\n", attitude.azimuth);
			// Send point cloud, crane data
			//pjh

			StPointDisplayData* pointDispData = new (std::nothrow)StPointDisplayData;
			memset(pointDispData, 0, sizeof(StPointDisplayData));

			ProcessDisplayData(*pointDispData, pDistance);

			m_integratedmonitoringUnity->SendPointData(pointDispData);

			StCraneDisplayData craneDispData;

			if (pier >= SHI::PIERZ)
			{
				//Attitude Copy
				craneDispData.pierId = attitude.pierId;
				craneDispData.craneId = attitude.craneId;

				craneDispData.transform[0].x = attitude.gpsLatitude;
				craneDispData.transform[0].y = attitude.gpsLongitude;
				craneDispData.transform[0].z = attitude.azimuth;
				craneDispData.transform[0].rx = attitude.pose[SHI::Pier6::LLC_JIB];
				craneDispData.transform[0].rz = attitude.height;
			}
			else
			{
				ProcessCraneDisplayData(craneDispData, *pointDispData, attitude);
			}

			craneDispData.transform[1].y = attitude.pose[SHI::PierHan::GC_ROOM];

			craneDispData.transform[1].rx = attitude.hookPoint[0].x;
			craneDispData.transform[1].ry = attitude.hookPoint[0].y;
			craneDispData.transform[1].rz = attitude.hookPoint[0].z;

			craneDispData.transform[2].x = attitude.hookPoint[1].x;
			craneDispData.transform[2].y = attitude.hookPoint[1].y;
			craneDispData.transform[2].z = attitude.hookPoint[1].z;

			craneDispData.transform[2].rx = attitude.hookPoint[2].x;
			craneDispData.transform[2].ry = attitude.hookPoint[2].y;
			craneDispData.transform[2].rz = attitude.hookPoint[2].z;

			m_integratedmonitoringUnity->SendCraneData(&craneDispData);

			delete pointDispData;
			//~pjh
			
			
			// Send distance
			StDistanceDisplayData* distanceDispData = new (std::nothrow)StDistanceDisplayData;
				if (distanceDispData)
				{
					ProcessDistanceDisplayData(*distanceDispData, pDistance);

					m_integratedmonitoringUnity->SendDistanceData(distanceDispData);

					delete distanceDispData;
				}

			m_status[pier].dataStatus[crane] = 1; // 정상
		}
		else
		{
			// GPS 이상
			m_status[pier].dataStatus[crane] = 2; // 데이터 수신되나, GPS 이상
		}
		printf("%s(%d) CraneAlive: %d, DataStatus : %d, GPS Quality : %d\n", SHI::ConvCraneStr(pier, crane).c_str(), crane, m_status[pier].craneAlive[crane], m_status[pier].dataStatus[crane], attitude.quality);

		// Update status
		float minDistance = FLT_MAX;
		for (uint32_t i = 0; i < pDistance->GetDistanceInfoSize(); i++)
		{
			if (pDistance->GetDistanceLabel()[i] == SHI::DISTANCE_NORMAL)
			{
				minDistance = (std::min)(minDistance, pDistance->GetDistanceInfo()[i].Distance);
			}
		}
		if (minDistance > 100.0f) minDistance = 0.0f;

		m_status[pier].distanceStatus[crane] = pDistance->collisionTotal;
		m_status[pier].distance[crane] = minDistance;
		m_status[pier].craneAlive[crane] = 1;

		// Send user info
		StMonitoringUserInfo userInfo;
		if (GetUserInfo(&userInfo, id))
		{
			m_integratedmonitoringUnity->SendUserInfo(&userInfo);
		}

		// Send system status
		StSystemStatusData* systemStatus = new (std::nothrow)StSystemStatusData;
		if (systemStatus)
		{
			memset(&systemStatus->pierId, 0, sizeof(StSystemStatusData) - sizeof(uint32_t));
			if (GetSystemStatus(systemStatus, id))
			{
				m_integratedmonitoringUnity->SendSystempStatus(systemStatus);
			}
			delete systemStatus;
		}
	}
}

void CIntegratedMonitoringInterface::OnRouterCollisionHistory(int32_t pier, int32_t crane, SHI::Data::StCollisionHistory* history)
{
	StCollisionHistory* data = new(std::nothrow) StCollisionHistory;
	if (data)
	{
		int32_t id = SHI::GetCraneId(pier, crane);
		data->pierId = history->pier;
		data->craneId = history->crane;
		data->year = history->year;
		memcpy_s(data->dayly, sizeof(data->dayly), history->dayly, sizeof(history->dayly));
		memcpy_s(data->monthly, sizeof(StCollisionHistoryMonthly) * 12, history->monthly, sizeof(StCollisionHistoryMonthly) * 12);

		// Update collision history
		UpdateCollisionHistory(id, data);

		delete data;
	}
}

void CIntegratedMonitoringInterface::OnRouterOperationHistory(int32_t pier, int32_t crane, SHI::Data::StOperationHistory* history)
{
	StOperationHistory* data = new(std::nothrow) StOperationHistory;
	if (data)
	{
		int32_t id = SHI::GetCraneId(pier, crane);
		data->pierId = history->pier;
		data->craneId = history->crane;
		data->year = history->year;
		memcpy_s(data->dayly, sizeof(data->dayly), history->dayly, sizeof(history->dayly));

		// Update Operation history
		UpdateOperationHistory(id, data);
		
		delete data;
	}
}

void CIntegratedMonitoringInterface::OnRouterUserInfo(int32_t pier, int32_t crane, SHI::Data::StMonitoringUserInfo* pUserInfo)
{
	int32_t id = SHI::GetCraneId(pier, crane);
	StMonitoringUserInfo info;
	info.pierId = pier;
	info.craneId = crane;
	memcpy(info.ID, pUserInfo->ID, sizeof(info.ID));
	info.Authority = pUserInfo->Authority;
	memcpy(info.Name, pUserInfo->Name, sizeof(info.Name));
	memcpy(info.Password, pUserInfo->Password, sizeof(info.Password));
	memcpy(info.Reserved, pUserInfo->Reserved, sizeof(info.Reserved));
	UpdateUserInfo(id, &info);
}

void CIntegratedMonitoringInterface::OnRouterHeartBeat(int32_t pier, int32_t crane)
{
	int32_t id = SHI::GetCraneId(pier, crane);
	UpdateHeartbeatCount(id);
}

void CIntegratedMonitoringInterface::OnRouterSystemStatus(int32_t pier, int32_t crane, SHI::Data::StSystemStatus* pSystemStatus)
{
	int32_t id = SHI::GetCraneId(pier, crane);
	
	StSystemStatusData *data = new StSystemStatusData;
	data->pierId = pier;
	data->craneId = crane;
	data->timestamp = pSystemStatus->timeStamp;
	data->InterfaceVersion = pSystemStatus->Version.Interface;
	data->ClusterVersion = pSystemStatus->Version.Cluster;
	data->DistanceVersion = pSystemStatus->Version.Distance;
	data->CollisionVersion = pSystemStatus->Version.Collision;
	data->ManagerSensorVersion = pSystemStatus->Version.ManagerSensor;
	data->ManagerCollisionVersion = pSystemStatus->Version.ManagerSensor;
	data->cooperationStatus = pSystemStatus->StatusCooperationMode;
	data->numRequestCooperations = getNumCooperations();

	int32_t numSensor = pSystemStatus->GetRotorStatusSize();
	data->numSensor = numSensor;
	for (int32_t i=0; i<numSensor; i++)
	{
		data->rotorFps[i] = pSystemStatus->GetRotorStatus()[i].ActuratorFPS;
		data->rotorRpm[i] = static_cast<unsigned int>(pSystemStatus->GetRotorStatus()[i].ActuratorRpm);
		data->errorCode[i] = pSystemStatus->GetRotorStatus()[i].ErrorCode;
		data->lidarFps[i] = pSystemStatus->GetRotorStatus()[i].LidarFps;
	}

	data->windSpeed = pSystemStatus->WindSpeed;
	data->winddirection = pSystemStatus->WindDirection;
	
	UpdateSystemStatus(id, data);
	delete data;

	m_status[pier].craneAlive[crane] = 1;
}

void CIntegratedMonitoringInterface::OnRouterCraneAttitude(int32_t pier, int32_t crane, SHI::Data::StCraneAttitude* attitude)
{
	int32_t id = SHI::GetCraneId(pier, crane);
	UpdateCraneAttitude(id, *attitude);
}

void CIntegratedMonitoringInterface::OnRouterCraneAlive(int32_t pier, int32_t crane)
{
	m_status[pier].craneAlive[crane] = 1;
}

void CIntegratedMonitoringInterface::OnRouterCooperationMode(int32_t pier, int32_t crane, SHI::Data::StCooperationMode* pCooperationMode)
{
}

 void CIntegratedMonitoringInterface::OnRouterPLCInfo(int32_t pier, int32_t crane, SHI::Data::StPlcInfo* pInfo)
{
	StPLCInfo info;
	info.pierId = pier;
	info.craneId = crane;

	info.Hoist1DecelControl = pInfo->Hoist1DecelControl;
	info.Hoist2DecelControl = pInfo->Hoist2DecelControl;
	info.Hoist3DecelControl = pInfo->Hoist3DecelControl;
	info.Trolley1DecelControl = pInfo->Trolley1DecelControl;
	info.Trolley2DecelControl = pInfo->Trolley2DecelControl;
	info.GantryDecelControl = pInfo->GantryDecelControl;
	info.AuxHoistDecelControl = pInfo->AuxHoistDecelControl;
	info.SlewingDecelControl = pInfo->SlewingDecelControl;
	info.bUserEnableSwich = pInfo->bUserEnableSwich;
	info.bGantryCommandFoward = pInfo->bGantryCommandFoward;
	info.bGantryCommandReverse = pInfo->bGantryCommandReverse;
	info.bTrolley1CommandRight = pInfo->bTrolley1CommandRight;
	info.bTrolley1CommandLeft = pInfo->bTrolley1CommandLeft;
	info.bTrolley2CommandRight = pInfo->bTrolley2CommandRight;
	info.bTrolley2CommandLeft = pInfo->bTrolley2CommandLeft;
	info.bHoist1CommandUp = pInfo->bHoist1CommandUp;
	info.bHoist1CommandDown = pInfo->bHoist1CommandDown;
	info.bHoist2CommandUp = pInfo->bHoist2CommandUp;
	info.bHoist2CommandDown = pInfo->bHoist2CommandDown;
	info.bHoist3CommandUp = pInfo->bHoist3CommandUp;
	info.bHoist3CommandDown = pInfo->bHoist3CommandDown;
	info.bAuxHoistCommandUp = pInfo->bAuxHoistCommandUp;
	info.bAuxHoistCommandDown = pInfo->bAuxHoistCommandDown;
	info.Trolley1Position = pInfo->Trolley1Position; // mm
	info.Trolley2Position = pInfo->Trolley2Position; // mm
	info.GoliathPosition1 = pInfo->GoliathPosition1; // mm
	info.GoliathPosition2 = pInfo->GoliathPosition2; // mm
	info.Hoist1Position = pInfo->Hoist1Position; // mm
	info.Hoist2Position = pInfo->Hoist2Position; // mm
	info.Hoist3Position = pInfo->Hoist3Position; // mm
	info.Hoist1Weight = pInfo->Hoist1Weight; // 0.1ton
	info.Hoist2Weight = pInfo->Hoist2Weight; // 0.1ton
	info.Hoist3Weight = pInfo->Hoist3Weight; // 0.1ton
	info.AuxHoistWeight = pInfo->AuxHoistWeight;
	info.plcConnected = pInfo->plcConnected;
	memset(info.reserved, 0, sizeof(info.reserved));

	if (m_integratedmonitoringUnity)
	{
		m_integratedmonitoringUnity->SendPlcInfo(&info);
	}
}

void CIntegratedMonitoringInterface::OnRouterMaxDistance(int32_t pier, int32_t crane,
	SHI::Data::StMaxDistance* pMaxDistance)
{
}

void CIntegratedMonitoringInterface::OnTimer()
{
	static int32_t count10sec = 0;
	static int32_t count5sec = 0;
	
	if (m_bLogPlay)
	{
		//printf("Log play(True) \n");
		if (++count5sec > 5)
		{
			count5sec = 0;

			// 포인트 색상 모드 갱신
			int32_t mode = Routine::GetConfigInt("Test", "ColorMode", 0);
			SetColorMode(mode);

			for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
			{
				if (pier == SHI::PIER7) continue;

				// 상태 정보 송신
				m_integratedmonitoringUnity->SendStatus(&m_status[pier]);

				for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
				{
					int32_t id = SHI::GetCraneId(pier, crane);
					
					// DB 정보 조회(충돌 이력, 운용 이력)
					Routine::CTime t;
					t.UpdateCurrentTime();
					int32_t year = m_dbPlayer->GetSelectedDate().year;
					int32_t maxYear = 5;
					for (int32_t dyear = 0; dyear < maxYear; dyear++)
					{
						SHI::Data::StCollisionHistory collisionHistory;
						if (m_dbConnector.ReadCollisionHistory(id, year - dyear, collisionHistory))
						{
							OnRouterCollisionHistory(pier, crane, &collisionHistory);
						}

						SHI::Data::StOperationHistory operationHistory;
						if (m_dbConnector.ReadOperationHistory(id, year - dyear, operationHistory))
						{
							OnRouterOperationHistory(pier, crane, &operationHistory);
						}
					}

					// 각 크레인의 충돌 알림 이력 송신
					StRequestCollisionHistory* reqHistory = new (std::nothrow)StRequestCollisionHistory;
					if (reqHistory)
					{
						for (int32_t dyear = 0; dyear < maxYear; dyear++)
						{
							reqHistory->craneId = crane;
							reqHistory->pierId = pier;
							reqHistory->year = year - dyear;
							OnRequestCollisionHistory(reqHistory);
						}
						delete reqHistory;
					}
				}

				// 통신 상태 'Fault'로 리셋
				memset(m_status[pier].craneAlive, 0, sizeof(m_status[pier].craneAlive));

				memset(m_status[pier].dataStatus, 0, sizeof(m_status[pier].dataStatus));
			}
		}
	}
	else
	{
		if (++count5sec > 5)
		{
			count5sec = 0;

			// 포인트 색상 모드 갱신
			int32_t mode = Routine::GetConfigInt("Test", "ColorMode", 0);
			SetColorMode(mode);

			for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
			{
				if (pier == SHI::PIER7) continue;

				// 상태 정보 송신
				m_integratedmonitoringUnity->SendStatus(&m_status[pier]);

				for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
				{
					int32_t id = SHI::GetCraneId(pier, crane);
					 
					// DB 정보 조회(크레인 자세)
					SHI::Data::StCraneAttitude attitude;
					if (m_dbConnector.ReadCraneAttitude(id, attitude))
					{
						UpdateCraneAttitude(id, attitude);
					}

					// DB 정보 조회(충돌 이력, 운용 이력)
					Routine::CTime t;
					t.UpdateCurrentTime();
					int32_t year = t.Year();
					int32_t maxYear = 5;
					for (int32_t dyear = 0; dyear < maxYear; dyear++)
					{
						SHI::Data::StCollisionHistory collisionHistory;
						if (m_dbConnector.ReadCollisionHistory(id, year - dyear, collisionHistory))
						{
							//printf("CollisionHistory -> Pier = %d, Crane = %d, DB History -> Pier => %d , Crane => %d\n", pier, crane, collisionHistory.pier, collisionHistory.crane);
							//printf("Daye = %4d:%2d:%2d\n", collisionHistory.year, collisionHistory.monthly, collisionHistory.dayly);
							OnRouterCollisionHistory(pier, crane, &collisionHistory);
						}

						SHI::Data::StOperationHistory operationHistory;
						if (m_dbConnector.ReadOperationHistory(id, year - dyear, operationHistory))
						{
							//printf("OperationHistory -> Pier = %d, Crane = %d, DB History -> Pier => %d , Crane => %d\n", pier, crane, operationHistory.pier, operationHistory.crane);
							//printf("Daye = %4d:%2d:%2d\n", collisionHistory.year, collisionHistory.monthly, collisionHistory.dayly);
							OnRouterOperationHistory(pier, crane, &operationHistory);
						}
					}

					// 상태 '미접속'인 경우 마지막 상태 송신
					//if (m_status[pier].status[crane] == 0 ||
					//	(m_status[pier].status[crane] == 1 && m_status[pier].reserved[crane]==0))
					if (m_status[pier].dataStatus[crane] == 0)
					{
						SHI::Data::StCraneAttitude attitude;
						if (GetCraneAttitude(attitude, id))
						{
							if (attitude.craneId == crane && attitude.pierId == pier)
							{
								SHI::CraneAttitude _attitude;
								memcpy_s(&_attitude, sizeof(SHI::CraneAttitude), &attitude, sizeof(SHI::Data::StCraneAttitude));
								if (_attitude.pierId == 5 ||
									_attitude.pierId == 6 ||
									_attitude.pierId == 7)
								{
									_attitude.azimuth = -attitude.azimuth;
								}
								StCraneDisplayData dispData;
								float dx = 0, dy = 0, dz = 0, rz = 0;
								SHI::Transform::GetGlobalPositionCrane(dispData.transform[0].x, dispData.transform[0].y, dispData.transform[0].z, dispData.transform[0].rx, dispData.transform[0].rz, _attitude);
								dispData.pierId = pier;
								dispData.craneId = crane;
								m_integratedmonitoringUnity->SendCraneData(&dispData);
							}
						}
					}

					// 각 크레인의 충돌 알림 이력 송신
					StRequestCollisionHistory* reqHistory = new (std::nothrow)StRequestCollisionHistory;
					if (reqHistory)
					{
						for (int32_t dyear = 0; dyear < maxYear; dyear++)
						{
							reqHistory->craneId = crane;
							reqHistory->pierId = pier;
							reqHistory->year = year - dyear;
							OnRequestCollisionHistory(reqHistory);
						}
						delete reqHistory;
					}

					// Heartbeat 미수신 시 재연결 시도
					int32_t heartBeat = GetHeartbeatCount(id);
					int32_t heartBeatPrev = GetHeartbeatCountPrev(id);
					UpdateHeartbeatCountPrev(id, heartBeat);
					if (heartBeat == heartBeatPrev)
					{
						//printf("[%s %s] No heartbeat -> drop and reconnect\n", SHI::ConvPierStr(pier).c_str(), SHI::ConvCraneStr(pier, crane).c_str());
						m_IntegratedMonitoringRouter->ResetInterface(id);
					}
				}

				// 통신 상태 'Fault'로 리셋
				memset(m_status[pier].craneAlive, 0, sizeof(m_status[pier].craneAlive));

				memset(m_status[pier].dataStatus, 0, sizeof(m_status[pier].dataStatus));
			}
		}

		if (++count10sec > 10)
		{
			count10sec = 0;

			uint32_t numRequest = 0;
			for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
			{
				if (pier == SHI::PIER7) continue;
				for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
				{
					int32_t id = SHI::GetCraneId(pier, crane);

					m_IntegratedMonitoringRouter->SendRequestCraneAttitude(id);

					if (m_status[pier].craneAlive[crane] == 1)
					{
						m_IntegratedMonitoringRouter->SendRequestCollisionHistory(id);
						m_IntegratedMonitoringRouter->SendRequestOperationHistory(id);
					}

					if (m_IntegratedMonitoringRouter->GetRecvCount(id) == 0)
					{
						m_IntegratedMonitoringRouter->ResetInterface(id);
					}

					numRequest += m_dbConnector.ReadNumCooperation(id);
				}
			}

			SetNumCooperation(numRequest);
		}
	}
}

void CIntegratedMonitoringInterface::ProcessDisplayData(StPointDisplayData& dispData, SHI::Data::StDistanceSocket* pDistance)
{
	SHI::CraneAttitude attitude;
	memcpy_s(&attitude, sizeof(attitude), &pDistance->attitude, sizeof(pDistance->attitude));
	
	// Update XYZ
	dispData.craneId = attitude.craneId;
	dispData.pierId = attitude.pierId;
	dispData.timestamp = pDistance->attitude.gpsTimestamp;

	SHI::PointCloudPtr cloud(new SHI::PointCloud);
	cloud->resize(pDistance->GetXYZSize());
	for (uint32_t i = 0; i < pDistance->GetXYZSize(); i++)
	{
		cloud->points[i].x = pDistance->GetXYZ()[i].X * 0.01f;
		cloud->points[i].y = pDistance->GetXYZ()[i].Y * 0.01f;
		cloud->points[i].z = pDistance->GetXYZ()[i].Z * 0.01f;
	}

	// 안벽 좌표계로 변환
	//pjhSHI::Transform::TransformPointCloudGps(cloud, *cloud, attitude);
	
	// 타 크레인의 몸체 데이터 제거
	bool bRemoveCranePoint = Routine::GetConfigInt("Test", "bRemoveCranePoint", 1) == 0 ? (false) : (true);
	if (bRemoveCranePoint)
	{
		for (uint32_t i = 0; i < SHI::GetNumCrane(attitude.pierId); i++)
		{
			int32_t targetId = SHI::GetCraneId(attitude.pierId, i);
			SHI::Data::StCraneAttitude attitude;
			SHI::CraneAttitude targetAttitude;
			memcpy_s(&targetAttitude, sizeof(targetAttitude), &attitude, sizeof(attitude));
			if (attitude.craneId != i)
			{
				/*pjhSHI::Data::StCraneAttitude attitude;
				SHI::CraneAttitude targetAttitude;
				memcpy_s(&targetAttitude, sizeof(targetAttitude), &attitude, sizeof(attitude));*/

				bool succeed = GetCraneAttitude(attitude, targetId);
				if (succeed && IsValidGps(attitude))
				{
					SHI::CraneAttitude targetAttitude;
					memcpy_s(&targetAttitude, sizeof(targetAttitude), &attitude, sizeof(attitude));
					int32_t prevSize = cloud->size();

					// 타 크레인 좌표계로 변환
					//pjhSHI::Transform::TransformPointCloudGpsInv(cloud, *cloud, targetAttitude);

					// 몸체 포인트 분류
					SHI::PointCloudPtr cloudOutlier(new SHI::PointCloud);
					SHI::PointCloudPtr cloudInlier(new SHI::PointCloud);

					//pjh
					// ROI 축소
					for (uint32_t i = 0; i < targetAttitude.numPart; i++)
					{
						targetAttitude.craneRoi[i].roi.minX = 0;
						targetAttitude.craneRoi[i].roi.minY = 0;
						targetAttitude.craneRoi[i].roi.minZ = 0;
						targetAttitude.craneRoi[i].roi.maxX = 0;
						targetAttitude.craneRoi[i].roi.maxY = 0;
						targetAttitude.craneRoi[i].roi.maxZ = 0;
					}
					//~pjh

					SHI::Filter::FilterCrane(*cloudOutlier, *cloudInlier, cloud, targetAttitude);

					// 안벽 좌표계로 변환
					//pjhSHI::Transform::TransformPointCloudGps(cloudInlier, *cloud, targetAttitude);
					cloud = cloudInlier;
				}
			}
		}
	}

	//pjh
	int towerIdx = SHI::PierK::JIB_TOWER;
	if (attitude.craneId == SHI::PierK::TTC23 && attitude.pierId == SHI::PIERK)
	{
		towerIdx = SHI::PierK::TTC_TOWER;
	}
	if (attitude.pierId == SHI::PIERHAN)
	{
		towerIdx = SHI::PierHan::TTC_TOWER;
	}
	if (attitude.pierId == SHI::PIER6)
	{
		towerIdx = SHI::Pier6::LLC_TOWER;
	}
	if (attitude.pierId == SHI::G2DOCK)
	{
		towerIdx = SHI::G2Dock::LLC_TOWER;
	}
	if (attitude.pierId == SHI::G3DOCK)
	{
		towerIdx = SHI::G3Dock::LLC_TOWER;
	}
	if (attitude.pierId == SHI::G4DOCK)
	{
		towerIdx = SHI::G4Dock::LLC_TOWER;
	}
	if (attitude.pierId == SHI::PIERZ)
	{
		towerIdx = SHI::PierZ::LLC_TOWER;
	}
	//Y
	if (attitude.pierId == SHI::PIERY)
	{
		// cms
		// 이거 이렇게 분기하는게 맞는지 확신은 안드는데 일단 진행
		if(attitude.craneId == SHI::PierY::JIB1 || attitude.craneId == SHI::PierY::JIB2 ||
		attitude.craneId == SHI::PierY::JIB3 || attitude.craneId == SHI::PierY::JIB4 ||
		attitude.craneId == SHI::PierY::JIB5)
		{
			towerIdx = SHI::PierY::JIB_TOWER;
		}
		// TC는 tower 위치가 다름(enum에서 1임)
		else if(attitude.craneId == SHI::PierY::TC1)
		{
			towerIdx = SHI::PierY::TC_TOWER;
		}
	}
	//~Y
	
	//SHI::GpsAttitude gpsAttitude = SHI::GetGpsParam(attitude.pierId, attitude.craneId);
	SHI::PointCloud tmp;//pjh
	tmp.resize(pDistance->GetXYZSize());
	for (unsigned int i = 0; i < cloud->size(); i++)
	{
		Eigen::Matrix<float_t, 3, 1> pt(cloud->points[i].x - attitude.jointInfo[towerIdx].cx, cloud->points[i].y - attitude.jointInfo[towerIdx].cy, cloud->points[i].z);
		tmp[i].x = cloud->points[i].x;
		tmp[i].y = cloud->points[i].y;
		auto dz = attitude.height;// +gpsAttitude.gpsOffsetZ;
		tmp[i].z = static_cast<float_t> (pt.coeffRef(2) + dz);
	}
	//~pjh

	float minHeight = 14;
	float maxHeight = 60;
	dispData.numPoint = cloud->size();
	if (GetColorMode() == CCraneStatus::COLOR_CRANE)
	{
		for (uint32_t i = 0; i < cloud->size(); i++)
		{
			dispData.points[i].x = cloud->points[i].x;
			dispData.points[i].y = cloud->points[i].y;
			dispData.points[i].z = cloud->points[i].z;
			//pjh
			if (tmp.size() > 0 && minHeight < tmp.points[i].z)
			{
				dispData.points[i].r = MyColor::GetTable(attitude.craneId, 0, 5).r / 255.0f;
				dispData.points[i].g = MyColor::GetTable(attitude.craneId, 0, 5).g / 255.0f;
				dispData.points[i].b = MyColor::GetTable(attitude.craneId, 0, 5).b / 255.0f;
			}
			//~pjh
			else if (minHeight < cloud->points[i].z)
			{
				dispData.points[i].r = MyColor::GetTable(attitude.craneId, 0, 5).r / 255.0f;
				dispData.points[i].g = MyColor::GetTable(attitude.craneId, 0, 5).g / 255.0f;
				dispData.points[i].b = MyColor::GetTable(attitude.craneId, 0, 5).b / 255.0f;
			}
			else
			{
				dispData.points[i].r = 0.1f;
				dispData.points[i].g = 0.1f;
				dispData.points[i].b = 0.1f;
			}
		}
	}
	else if (GetColorMode() == CCraneStatus::COLOR_HEIGHT)
	{
		int32_t r = Routine::GetConfigInt("Test", "R", 0);
		int32_t g = Routine::GetConfigInt("Test", "G", 192);
		int32_t b = Routine::GetConfigInt("Test", "B", 0);

		for (uint32_t i = 0; i < cloud->size(); i++)
		{
			dispData.points[i].x = cloud->points[i].x;
			dispData.points[i].y = cloud->points[i].y;
			dispData.points[i].z = cloud->points[i].z;
			//pjh
			if (tmp.size() > 0 && minHeight < tmp.points[i].z)
			{
				dispData.points[i].r = MyColor::GetTable(tmp.points[i].z, minHeight, maxHeight).r / 255.0f;
				dispData.points[i].g = MyColor::GetTable(tmp.points[i].z, minHeight, maxHeight).g / 255.0f;
				dispData.points[i].b = MyColor::GetTable(tmp.points[i].z, minHeight, maxHeight).b / 255.0f;
			}
			//~pjh
			else if (minHeight < cloud->points[i].z)
			{
				dispData.points[i].r = MyColor::GetTable(cloud->points[i].z, minHeight, maxHeight).r / 255.0f;
				dispData.points[i].g = MyColor::GetTable(cloud->points[i].z, minHeight, maxHeight).g / 255.0f;
				dispData.points[i].b = MyColor::GetTable(cloud->points[i].z, minHeight, maxHeight).b / 255.0f;
			}
			else
			{
				dispData.points[i].r = r/255.0f;
				dispData.points[i].g = g/255.0f;
				dispData.points[i].b = b/255.0f;
			}
		}
	}
	else
	{

	}
}

void CIntegratedMonitoringInterface::ProcessCraneDisplayData(StCraneDisplayData& craneDispData, StPointDisplayData& pointDispData, SHI::CraneAttitude &attitude)
{
	craneDispData.pierId = attitude.pierId;
	craneDispData.craneId = attitude.craneId;
	SHI::Transform::GetGlobalPositionCrane(craneDispData.transform[0].x, craneDispData.transform[0].y, craneDispData.transform[0].z, craneDispData.transform[0].rx, craneDispData.transform[0].rz, attitude);
}

void CIntegratedMonitoringInterface::ProcessDistanceDisplayData(StDistanceDisplayData& distanceDispData, SHI::Data::StDistanceSocket* pDistance)
{
	distanceDispData.pierId = pDistance->attitude.pierId;
	distanceDispData.craneId = pDistance->attitude.craneId;
	distanceDispData.numDistance = 0;

	std::vector<StDistanceDisplay> vDistanceHook, vDistanceNormal, vDistance1, vDistance2, vDistance3;
	for (uint32_t i = 0; i < pDistance->GetDistanceInfoSize(); i++)
	{
		if (pDistance->GetDistanceLabel()[i] == SHI::DISTANCE_NORMAL)
		{
			StDistanceDisplay d;
			d.distance = pDistance->GetDistanceInfo()[i].Distance;
			d.level = pDistance->GetCollisionInfo()[i];
			d.p1[0] = pDistance->GetDistanceInfo()[i].PosCrane.X;
			d.p1[1] = pDistance->GetDistanceInfo()[i].PosCrane.Y;
			d.p1[2] = pDistance->GetDistanceInfo()[i].PosCrane.Z;
			d.p2[0] = pDistance->GetDistanceInfo()[i].PosCluster.X;
			d.p2[1] = pDistance->GetDistanceInfo()[i].PosCluster.Y;
			d.p2[2] = pDistance->GetDistanceInfo()[i].PosCluster.Z;

			switch (pDistance->GetCollisionInfo()[i])
			{
			case 0:
				vDistanceNormal.push_back(d);
				break;
			case 1:
				vDistance1.push_back(d);
				break;
			case 2:
				vDistance2.push_back(d);
				break;
			case 3:
				vDistance3.push_back(d);
				break;
			default:
				break;
			}

			if (pDistance->attitude.pierId == SHI::PIERHAN && pDistance->attitude.craneId == SHI::PierHan::GC1)
			{
				if (pDistance->GetDistanceInfo()[i].CraneIndex == SHI::PierHan::GC_LOWER_TROLLY_HOOK ||
					pDistance->GetDistanceInfo()[i].CraneIndex == SHI::PierHan::GC_UPPER_TROLLY_HOOK)
				{
					vDistanceHook.push_back(d);
				}
			}
			else if (pDistance->attitude.pierId == SHI::PIERHAN && pDistance->attitude.craneId == SHI::PierHan::GC2)
			{
				if (pDistance->GetDistanceInfo()[i].CraneIndex == SHI::PierHan::GC_LOWER_TROLLY_HOOK ||
					pDistance->GetDistanceInfo()[i].CraneIndex == SHI::PierHan::GC_UPPER_TROLLY_HOOK)
				{
					vDistanceHook.push_back(d);
				}
			}
			else if (pDistance->attitude.pierId == SHI::PIERHAN && pDistance->attitude.craneId == SHI::PierHan::TTC4)
			{
				if (pDistance->GetDistanceInfo()[i].CraneIndex == SHI::PierHan::TTC_TROLLY ||
					pDistance->GetDistanceInfo()[i].CraneIndex == SHI::PierHan::TTC_TROLLY2)
				{
					vDistanceHook.push_back(d);
				}
			}
			else
			{
			}

		}
	}

	// 가장 가까운 거리값 하나만 전달한다.
	if (vDistance3.size() > 0)
	{
		distanceDispData.numDistance = 1;
		memcpy(distanceDispData.distnace, vDistance3.data(), vDistance3.size() * sizeof(StDistanceDisplay));
	}
	else if (vDistance2.size() > 0)
	{
		distanceDispData.numDistance = 1;
		memcpy(distanceDispData.distnace, vDistance2.data(), vDistance2.size() * sizeof(StDistanceDisplay));
	}
	else if (vDistance1.size() > 0)
	{
		distanceDispData.numDistance = 1;
		memcpy(distanceDispData.distnace, vDistance1.data(), vDistance1.size() * sizeof(StDistanceDisplay));
	}
	else if (vDistanceHook.size() > 0)
	{
		distanceDispData.numDistance = 1;
		memcpy(distanceDispData.distnace, vDistanceHook.data(), vDistanceHook.size() * sizeof(StDistanceDisplay));
	}
	else if (vDistanceNormal.size() > 0)
	{
		distanceDispData.numDistance = 1;
		memcpy(distanceDispData.distnace, vDistanceNormal.data(), vDistanceNormal.size() * sizeof(StDistanceDisplay));
	}
	else
	{
	}
}
