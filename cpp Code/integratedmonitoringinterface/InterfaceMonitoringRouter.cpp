
#include "InterfaceMonitoringRouter.h"
#include <Config/CraneInfo.h>

CMonitoringRouter::CMonitoringRouter()
	: m_pObserver(NULL)
{
}

CMonitoringRouter::~CMonitoringRouter()
{
	std::map<int32_t, SHI::Interface::Monitoring::Client::CClientMonitoringLabeled*>::iterator it;
	for (it = m_routers.begin(); it != m_routers.end(); ++it)
	{
		if (it->second)
		{
			delete it->second;
			it->second = 0;
		}
	}

	std::map<int32_t, SHI::Compressor::CCompressDistance*>::iterator itComp;
	for (itComp = m_decoders.begin(); itComp != m_decoders.end(); ++itComp)
	{
		if (itComp->second)
		{
			delete itComp->second;
			itComp->second = 0;
		}
	}
}

void CMonitoringRouter::SetObserver(CObserverMonitoringRouter* pObserver)
{
	m_pObserver = pObserver;
}

bool CMonitoringRouter::Create()
{
	bool ret = false;
	uint32_t numPier = SHI::GetNumPier();
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		uint32_t numCrane = SHI::GetNumCrane(pier);
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);
			std::string strPier = SHI::ConvPierStr(pier);
			std::string strCrane = SHI::ConvCraneStr(pier, crane);
			std::string fieldName = strPier + "_" + strCrane;

			// 라우터 인터페이스 생성
			std::string routereIP = Routine::GetConfigString(fieldName.c_str(), "routerIP", "127.0.0.1");
			int32_t routerPort = Routine::GetConfigInt(fieldName.c_str(), "routerPort", 10000 + pier * 10 + crane);
			SHI::Interface::Monitoring::Client::CClientMonitoringLabeled* pObjCollisionManager;
			pObjCollisionManager = new SHI::Interface::Monitoring::Client::CClientMonitoringLabeled(this, id);
			ret = pObjCollisionManager->Create(routereIP, routerPort, sizeof(SHI::Data::StDistanceSocket));
			m_routers[id] = pObjCollisionManager;

			// 디코더 객체 생성
			m_decoders[id] = new (std::nothrow) SHI::Compressor::CCompressDistance();

			m_countRecv[id] = 0;

			//pjh
			printf("Create %s %s, %d : %s\n", fieldName.c_str(), routereIP.c_str(), routerPort, ret ? "Succesd CreateTcpClient" : "Failed CreateTcpClient");
		}
	}
	return ret;
}

void CMonitoringRouter::ResetInterface(int32_t id)
{
	if (m_routers.find(id) != m_routers.end())
	{
		if (m_routers[id])
		{
			int32_t pier = SHI::GetID2Pier(id);
			int32_t crane = SHI::GetID2Crane(id);
			std::string strPier = SHI::ConvPierStr(pier);
			std::string strCrane = SHI::ConvCraneStr(pier, crane);
			std::string fieldName = strPier + "_" + strCrane;
			std::string routereIP = Routine::GetConfigString(fieldName.c_str(), "routerIP", "127.0.0.1");
			int32_t routerPort = Routine::GetConfigInt(fieldName.c_str(), "routerPort", 10000 + pier * 10 + crane);

			//m_routers[id]->GetSocketTCP()->AutoConnectStop();
			m_routers[id]->GetSocketTCP()->Destroy();
			//m_routers[id]->GetSocketTCP()->WaitForEndThread();

			m_routers[id]->GetSocketTCP()->CreateTcpClient(routereIP, routerPort);
			//m_routers[id]->GetSocketTCP()->AutoConnectStart();
			//m_routers[id]->GetSocketTCP()->ServiceStart();
		}
	}
}

bool CMonitoringRouter::SendRequestCraneAttitude(int32_t id)
{
	bool ret = false;
	if (m_routers.find(id) != m_routers.end())
	{
		if (m_routers[id])
		{
			m_routers[id]->SendRequestCraneAttitude();
			ret = true;
		}
	}
	return ret;
}

bool CMonitoringRouter::SendRequestCollisionHistory(int32_t id)
{
	bool ret = false;
	if (m_routers.find(id) != m_routers.end())
	{
		if (m_routers[id])
		{
			m_routers[id]->SendRequestCollisionHistory();
			ret = true;
		}
	}
	return ret;
}

bool CMonitoringRouter::SendRequestOperationHistory(int32_t id)
{
	bool ret = false;
	if (m_routers.find(id) != m_routers.end())
	{
		if (m_routers[id])
		{
			m_routers[id]->SendRequestOperationHistory();
			ret = true;
		}
	}
	return ret;
}

uint32_t CMonitoringRouter::GetRecvCount(int32_t id)
{
	uint32_t ret = 0;
	if (m_countRecv.find(id) != m_countRecv.end())
	{
		ret = m_countRecv[id];
		m_countRecv[id] = 0;
	}
	return ret;
}

//std::map<int32_t, SHI::Interface::Monitoring::Client::CClientMonitoringLabeled*> &CInterfaceMonitoringRouter::GetMonitoringRouterList()
//{
//	return m_vRouter;
//}

//SHI::Interface::Monitoring::Client::CClientMonitoringLabeled* CInterfaceMonitoringRouter::GetMonitoringRouter(int32_t idx)
//{
//	SHI::Interface::Monitoring::Client::CClientMonitoringLabeled* ret = NULL;
//	if (m_vRouter.size() > idx)
//	{
//		ret = m_vRouter[idx];
//	}
//	return ret;
//}
//
//SHI::Interface::Monitoring::Client::CClientMonitoringLabeled* CInterfaceMonitoringRouter::GetMonitoringRouter(int32_t pier, int32_t crane)
//{
//	SHI::Interface::Monitoring::Client::CClientMonitoringLabeled* ret = NULL;
//	for (uint32_t i=0; i<m_vRouter.size(); i++)
//	{
//		if (m_vRouter[i]->GetLabel() == Crane2Id(pier, crane))
//		{
//			ret = GetMonitoringRouter(i);
//			break;
//		}
//	}
//	return ret;
//}

void CMonitoringRouter::OnServerConnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
{
	if(m_pObserver) m_pObserver->OnRouterConnected(SHI::GetID2Pier(id), SHI::GetID2Crane(id), ip, port);
}

void CMonitoringRouter::OnServerDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
{
	if (m_pObserver) m_pObserver->OnRouterDisconnected(SHI::GetID2Pier(id), SHI::GetID2Crane(id), ip, port);
}

void CMonitoringRouter::OnServerSystemStatus(int32_t id, SHI::Data::StSystemStatus* pSystemStatus)
{
	if (m_pObserver) m_pObserver->OnRouterSystemStatus(SHI::GetID2Pier(id), SHI::GetID2Crane(id), pSystemStatus);
}

void CMonitoringRouter::OnServerDistance(int32_t id, SHI::Data::StDistanceSocket* pDistance)
{
	if (m_pObserver) m_pObserver->OnRouterDistance(SHI::GetID2Pier(id), SHI::GetID2Crane(id), pDistance);
}

void CMonitoringRouter::OnServerDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance)
{
	if (m_decoders.find(id) != m_decoders.end())
	{
		SHI::Compressor::CCompressDistance* decoder = m_decoders[id];
		SHI::Data::StDistanceSocket* uncompressed = new (std::nothrow)SHI::Data::StDistanceSocket;
		if (uncompressed)
		{
			if (decoder->Decode(*uncompressed, *pDistance))
			{
				if (m_pObserver) m_pObserver->OnRouterDistance(SHI::GetID2Pier(id), SHI::GetID2Crane(id), uncompressed);
			}
			delete uncompressed;
		}
	}
}

void CMonitoringRouter::OnServerResponseRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
{

}

void CMonitoringRouter::OnServerResponseCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
{

}

void CMonitoringRouter::OnServerResponseCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
{

}

void CMonitoringRouter::OnServerMaintenanceInfo(int32_t id, SHI::Data::StMaintenanceInfo* info)
{

}

 void CMonitoringRouter::OnServerPLCInfo(int32_t id, SHI::Data::StPlcInfo* pInfo)
{
	if (m_pObserver) m_pObserver->OnRouterPLCInfo(SHI::GetID2Pier(id), SHI::GetID2Crane(id), pInfo);
}

void CMonitoringRouter::OnServerCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
{
	if (m_pObserver) m_pObserver->OnRouterCooperationMode(SHI::GetID2Pier(id), SHI::GetID2Crane(id), pCooperationMode);
}

void CMonitoringRouter::OnServerMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
{
	if (m_pObserver) m_pObserver->OnRouterMaxDistance(SHI::GetID2Pier(id), SHI::GetID2Crane(id), pMaxDistance);
}
#ifdef TEMP_KSG
void CMonitoringRouter::OnServerRotorControlSocket(int32_t id, SHI::Data::StRotorControl* pRotorContorl)
{
	assert(false);
}
#endif				

void CMonitoringRouter::OnServerResponseCollisionHistory(int32_t id, SHI::Data::StCollisionHistory* history)
{
	if (m_pObserver) m_pObserver->OnRouterCollisionHistory(SHI::GetID2Pier(id), SHI::GetID2Crane(id), history);
}

void CMonitoringRouter::OnServerResponseOperationHistory(int32_t id, SHI::Data::StOperationHistory* history)
{
	if (m_pObserver) m_pObserver->OnRouterOperationHistory(SHI::GetID2Pier(id), SHI::GetID2Crane(id), history);
}

void CMonitoringRouter::OnServerMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
{
	if (m_pObserver) m_pObserver->OnRouterUserInfo(SHI::GetID2Pier(id), SHI::GetID2Crane(id), pUserInfo);
}

void CMonitoringRouter::OnServerHeartBeat(int32_t id)
{
	if (m_pObserver) m_pObserver->OnRouterHeartBeat(SHI::GetID2Pier(id), SHI::GetID2Crane(id));

	if (m_countRecv.find(id) != m_countRecv.end())
	{
		uint32_t count = m_countRecv[id];
		m_countRecv[id] = ++count;
	}
}

void CMonitoringRouter::OnServerResponseCraneAttitude(int32_t id, SHI::Data::StCraneAttitude* attitude)
{
	if (m_pObserver) m_pObserver->OnRouterCraneAttitude(SHI::GetID2Pier(id), SHI::GetID2Crane(id), attitude);
}

void CMonitoringRouter::OnServerCraneAlive(int32_t id)
{
	if (m_pObserver) m_pObserver->OnRouterCraneAlive(SHI::GetID2Pier(id), SHI::GetID2Crane(id));
}

 void CMonitoringRouter::OnServerPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
{
}