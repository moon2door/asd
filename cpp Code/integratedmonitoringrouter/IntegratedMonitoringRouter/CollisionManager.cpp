#include "CollisionManager.h"
#include <Routine/include/Base/RoutineUtility.h>
#include <Config/CraneInfo.h>

namespace IntegratedRouter
{
	CCollisionManager::CCollisionManager(CObserverCollisionManager* observer)
		: m_pObserver(observer)
	{

	}

	CCollisionManager::~CCollisionManager()
	{

	}

	bool CCollisionManager::Create()
	{
		bool ret = false;
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

				SHI::Interface::Monitoring::Client::CClientMonitoringLabeled* pObjCollisionManager;
				pObjCollisionManager = new SHI::Interface::Monitoring::Client::CClientMonitoringLabeled((CCollisionManager*)this, id);
				
				bool bCreated = pObjCollisionManager->Create(ip, port, sizeof(SHI::Data::StDistanceSocket));
				if (bCreated)
				{
					printf("[%s] %s:%d create succeed.\n", name.c_str(), ip.c_str(), port);
				}
				else
				{
					printf("[%s] %s:%d create failed.\n", name.c_str(), ip.c_str(), port);
				}

				ret = ret && bCreated;

				m_collisionManagers[id] = pObjCollisionManager;
				m_countRecv[id] = 0;
			}
		}
		return ret;
	}

	uint32_t CCollisionManager::SendMiniInfo(int32_t id, SHI::Data::StCraneMiniInfo* info)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendMiniInfo(info);
		}
		return ret;
	}

	void CCollisionManager::ResetInterface(int32_t id)
	{
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			int32_t pier = SHI::GetID2Pier(id);
			int32_t crane = SHI::GetID2Crane(id);
			std::string strPier = SHI::ConvPierStr(pier);
			std::string strCrane = SHI::ConvCraneStr(pier, crane);
			std::string name = strPier + "_" + strCrane;
			int32_t port = Routine::GetConfigInt(name.c_str(), "collisionManagerPort", 9098, "IntegratedMonitoringRouter.json");
			std::string ip = Routine::GetConfigString(name.c_str(), "collisionManagerIP", "127.0.0.1", "IntegratedMonitoringRouter.json");

			if (m_collisionManagers[id])
			{
				m_collisionManagers[id]->GetSocketTCP()->Destroy();
				m_collisionManagers[id]->GetSocketTCP()->CreateTcpClient(ip, port);
			}
		}
	}

	int32_t CCollisionManager::GetLastRecvCount(int32_t id)
	{
		int32_t ret = -1;
		if (m_countRecv.find(id) != m_countRecv.end())
		{
			ret = static_cast<int32_t>(m_countRecv[id]);
			m_countRecv[id] = 0;
		}
		return ret;
	}

	bool CCollisionManager::IsConnected(int32_t id)
	{
		bool ret = 0;
		//pjh
		if(m_collisionManagers.size()>0)
		ret = m_collisionManagers[id]->isConnected;
		/*if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->GetSocketTCP()->IsCreated();
		}*/
		//
		return ret;
	}

	uint32_t CCollisionManager::SendCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendCooperationMode(pCooperationMode);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendAlarmUse(int32_t id, SHI::Data::StAlarmUse* pAlarmUse)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendAlarmUse(pAlarmUse);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZone)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendCollisionZoneLength(pCollisionZone);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZone)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendCollisionZoneDecelerationRate(pCollisionZone);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRotorParameter(pRotorParameter);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRotorControl(int32_t id, SHI::Data::StRotorControl* pRotorControl)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRotorControl(pRotorControl);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRequestRotorParameter(int32_t id)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRequestRotorParameter();
		}
		return ret;
	}

#ifdef TEMP_KSG
	uint32_t CCollisionManager::SendRequestSetNewParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		int32_t ret = 0;
		if( m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			printf("Send\n");
			ret = m_collisionManagers[id]->SendSetNewParameter(pRotorParameter);
		}
		return ret;
	}
#endif
	uint32_t CCollisionManager::SendRequestCollisionZoneLength(int32_t id, SHI::Data::StCollisionRequestZone* pCollisionRequestZone)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRequestCollisionZoneLength(pCollisionRequestZone);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRequestCollisionZoneDecelerationRate(int32_t id)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRequestCollisionZoneDecelerationRate();
		}
		return ret;
	}

	uint32_t CCollisionManager::SendReportMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendReportMonitoringUserInfo(pUserInfo);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdateSensorManagerProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdateSensorManagerProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdateInterfaceProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdateInterfaceProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdateClusterProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdateClusterProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdateCollisionManagerProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdateCollisionManagerProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdateDistanceProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdateDistanceProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdateCollisionProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdateCollisionProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendUpdatePLCProc(int32_t id, unsigned char* pData, uint32_t size)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendUpdatePLCProc(pData, size);
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRequestCollisionHistory(int32_t id)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRequestCollisionHistory();
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRequestOperationHistory(int32_t id)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRequestOperationHistory();
		}
		return ret;
	}

	uint32_t CCollisionManager::SendRequestCraneAttitude(int32_t id)
	{
		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendRequestCraneAttitude();
		}
		return ret;
	}

	uint32_t CCollisionManager::SendPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
	{
		uint32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			if (m_collisionManagers[id])
			{
				ret = m_collisionManagers[id]->SendPlcControlInfo(pControl);
			}
		}
		return ret;
	}

	uint32_t CCollisionManager::SendMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
	{

		int32_t ret = 0;
		if (m_collisionManagers.find(id) != m_collisionManagers.end())
		{
			ret = m_collisionManagers[id]->SendMaxDistance(pMaxDistance);
		}
		return ret;
	}

	void CCollisionManager::OnServerConnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{
		if (m_pObserver)
		{
			m_pObserver->OnServerConnectedMonitoring(id, ip, port);
			//pjh
			m_collisionManagers[id]->isConnected = true;
			//
		}
	}

	void CCollisionManager::OnServerDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{
		if (m_pObserver)
		{
			m_pObserver->OnServerDisconnectedMonitoring(id, ip, port);
			//pjh
			m_collisionManagers[id]->isConnected = false;
			//
		}
	}

	void CCollisionManager::OnServerSystemStatus(int32_t id, SHI::Data::StSystemStatus* pSystemStatus)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerSystemStatus(id, pSystemStatus);
		}
	}

	void CCollisionManager::OnServerDistance(int32_t id, SHI::Data::StDistanceSocket* pDistance)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerDistance(id, pDistance);
		}
	}

	void CCollisionManager::OnServerDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerDistanceCompressed(id, pDistance);
		}
	}

	void CCollisionManager::OnServerResponseRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerResponseRotorParameter(id, pRotorParameter);
		}
	}

	void CCollisionManager::OnServerResponseCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerResponseCollisionZoneLength(id, pCollisionZoneLength);
		}
	}

	void CCollisionManager::OnServerResponseCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerResponseCollisionZoneDecelerationRate(id, pCollisionZoneDecelerationRate);
		}
	}

	void CCollisionManager::OnServerMaintenanceInfo(int32_t id, SHI::Data::StMaintenanceInfo* info)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerMaintenanceInfo(id, info);
		}
	}

	void CCollisionManager::OnServerPLCInfo(int32_t id, SHI::Data::StPlcInfo* pInfo)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerPLCInfo(id, pInfo);
		}
	}

	void CCollisionManager::OnServerPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerPlcControlInfo(id, pControl);
		}
	}

	void CCollisionManager::OnServerMiniInfo(int32_t id, SHI::Data::StCraneMiniInfo* info)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerMiniInfo(id, info);
		}
	}

	void CCollisionManager::OnServerModelDistanceSocket(int32_t id, SHI::Data::StModelDistanceSocket* distance)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerModelDistanceSocket(id, distance);
		}
	}

	void CCollisionManager::OnServerCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
	{
		m_countRecv[id]++;
		if (m_pObserver)
		{
			m_pObserver->OnServerCooperationMode(id, pCooperationMode);
		}
	}

	void CCollisionManager::OnServerMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
	{
		if (m_pObserver)
		{
			m_pObserver->OnServerMaxDistance(id, pMaxDistance);
		}
	}

#ifdef TEMP_KSG
	void CCollisionManager::OnServerRotorControlSocket(int32_t id, SHI::Data::StRotorControl* pRotorControl)
	{
		if (m_pObserver)
		{
			m_pObserver->OnServerRotorControlSocket(id, pRotorControl);
		}
	}
#endif
}
