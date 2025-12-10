#include "IntegratedMonitoringRouter.h"
#include <Routine/Include/Base/RoutineUtility.h>
#include <Utility/Types.h>
#include <Config/CraneInfo.h>
#include <string>
#include <Utility/BuildInfo.h>

namespace IntegratedRouter
{
	CIntegratedMonitoringRouter::CIntegratedMonitoringRouter()
		: m_collisionManager(nullptr), m_monitoring(nullptr), m_dbConnector(nullptr), m_craneStatus(nullptr)
	{
		m_dbConnector = new(std::nothrow) CProcessDBConnector;
		m_craneStatus = new(std::nothrow) CModelCraneStatus;
	}

	CIntegratedMonitoringRouter::~CIntegratedMonitoringRouter()
	{
		if (m_dbConnector) delete m_dbConnector;
		if (m_craneStatus) delete m_craneStatus;
	}

	bool CIntegratedMonitoringRouter::Create()
	{
		bool ret = false;
		printf("=======================================\n");
		printf("IntegratedMonitoringRouter\n");
		printf(" - Version %u\n", SHI::BuildInfo::GetBuildDate());
		printf("=======================================\n");
		
		// DB Initialize
		int32_t dbPort = Routine::GetConfigInt("MongoDB", "Port", 27017, Routine::GetMakeConfigName());
		std::string dbIp = Routine::GetConfigString("MongoDB", "IP", "127.0.0.1");

		m_dbConnector->ConnectDB(dbIp, dbPort);
		for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
		for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
		{
			for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
			{
				int32_t id = SHI::GetCraneId(pier, crane);
				
				//pjh
				if (!m_collisionManager || !m_collisionManager->IsConnected(id))continue;
				//

				if (m_dbConnector)
				{
					printf("------------------------------------------------- \n");
					m_dbConnector->InsertCraneInfo(id);
				}
			}
		}

		// Create monitoring
		m_monitoring = new (std::nothrow)CMonitoring(this);
		if (m_monitoring)
		{
			ret = m_monitoring->Create();
		}

		// Create collision manager
		m_collisionManager = new (std::nothrow)CCollisionManager(this);
		if (m_collisionManager)
		{
			bool bCreateCollisioManager = m_collisionManager->Create();
			ret = ret && bCreateCollisioManager;
		}

		// Add pinger
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
		m_timer1Hz.StartTimer(1000, &CIntegratedMonitoringRouter::OnTimer1sec, this);
		m_timer10Hz.StartTimer(10000, &CIntegratedMonitoringRouter::OnTimer10Sec, this);

		return ret;
	}

	void CIntegratedMonitoringRouter::OnTimer1sec()
	{
		for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
		{
			for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
			{
				int32_t id = SHI::GetCraneId(pier, crane);

				//pjh
				if (!m_collisionManager->IsConnected(id))continue;
				//

				m_monitoring->SendReportMonitoringHeartBeat(id);

				if (IsPingConnected(id))
				{
					m_monitoring->SendCraneAlive(id);

					SHI::Data::StOperationHistory* hist = new (std::nothrow) SHI::Data::StOperationHistory;
					if (hist)
					{
						// DB 운용 이력 정보 업데이트
						// m_dbConnector->Get...
						// m_dbConnector->Update...
					}
				}

				if (m_craneStatus)
				{
					SHI::Data::StCraneStatus craneStatus;
					if (m_craneStatus->GetCraneStatus(id, craneStatus))
					{
						if(m_dbConnector) m_dbConnector->InsertCraneStatus(id, &craneStatus);
					}
				}
			}
		}
	}

	void CIntegratedMonitoringRouter::OnTimer10Sec()
	{
		// 연결 상태 확인 및 재연결
		for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
		{
			for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
			{
				int32_t id = SHI::GetCraneId(pier, crane);

				//pjh
				if (!m_collisionManager->IsConnected(id))continue;
				//

				int32_t count = m_collisionManager->GetLastRecvCount(id);
				if (count <= 0)
				{
					m_collisionManager->ResetInterface(id);
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// ProcessCollisionManager
	void CIntegratedMonitoringRouter::OnServerConnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{
		printf("OnServerConnectedMonitoring %s %d\n", ip, port);
	}

	void CIntegratedMonitoringRouter::OnServerDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{

	}

	void CIntegratedMonitoringRouter::OnServerSystemStatus(int32_t id, SHI::Data::StSystemStatus* pSystemStatus)
	{
		if (m_monitoring)
		{
			m_monitoring->SendSystemStatus(id, pSystemStatus);
		}

		if (m_craneStatus)
		{
			m_craneStatus->UpdateCraneStatus(id, *pSystemStatus);
		}
	}

	void CIntegratedMonitoringRouter::OnServerDistance(int32_t id, SHI::Data::StDistanceSocket* pDistance)
	{
		if (m_monitoring)
		{
			m_monitoring->SendDistance(id, pDistance);
		}
	}

	void CIntegratedMonitoringRouter::OnServerDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance)
	{
		if (m_monitoring)
		{
			m_monitoring->SendDistanceCompressed(id, pDistance);

			m_dbConnector->InsertDistanceCompressed(id, pDistance);
		}
	}

	void CIntegratedMonitoringRouter::OnServerResponseRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		if (m_monitoring)
		{
			m_monitoring->SendResponseRotorParameter(id, pRotorParameter);
		}
	}

	void CIntegratedMonitoringRouter::OnServerResponseCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		if (m_monitoring)
		{
			m_monitoring->SendResponseCollisionZoneLength(id, pCollisionZoneLength);
		}
	}

	void CIntegratedMonitoringRouter::OnServerResponseCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		if (m_monitoring)
		{
			m_monitoring->SendResponseCollisionZonepDecelerationRate(id, pCollisionZoneDecelerationRate);
		}
	}

	void CIntegratedMonitoringRouter::OnServerCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
	{
	}

	void CIntegratedMonitoringRouter::OnServerMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
	{
		if (m_monitoring)
		{
			m_monitoring->SendMaxDistance(id, pMaxDistance);
		}
	}

	void CIntegratedMonitoringRouter::OnServerMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
	{
		if (m_monitoring)
		{
			m_monitoring->SendReportMonitoringUserInfo(id, pUserInfo);
		}
	}

	void CIntegratedMonitoringRouter::OnServerCraneMini(int32_t id, SHI::Data::StCraneMiniInfo* info)
	{
		if (m_monitoring)
		{
			m_monitoring->SendMiniInfo(id, info);
		}
	}

	void CIntegratedMonitoringRouter::OnServerMaintenanceInfo(int32_t id, SHI::Data::StMaintenanceInfo* info)
	{
		if (m_monitoring)
		{
			m_monitoring->SendMaintenanceInfo(id, info);
		}

		if (m_craneStatus)
		{
			m_craneStatus->UpdateCraneStatus(id, *info);
		}
	}

	void CIntegratedMonitoringRouter::OnServerPLCInfo(int32_t id, SHI::Data::StPlcInfo* info)
	{
		if (m_monitoring)
		{
			m_monitoring->SendPLCInfo(id, info);
		}

		if (m_craneStatus)
		{
			m_craneStatus->UpdateCraneStatus(id, *info);
		}
	}

	void CIntegratedMonitoringRouter::OnServerPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
	{
	}

	void CIntegratedMonitoringRouter::OnServerMiniInfo(int32_t id, SHI::Data::StCraneMiniInfo* info)
	{
	}

	void CIntegratedMonitoringRouter::OnServerModelDistanceSocket(int32_t id,
		SHI::Data::StModelDistanceSocket* distance)
	{
	}

	//////////////////////////////////////////////////////////////////////////
	// ProcessMonitoringInterface
	void CIntegratedMonitoringRouter::OnClientConnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{
	}

	void CIntegratedMonitoringRouter::OnClientDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{

	}
	
	void CIntegratedMonitoringRouter::OnClientCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendCooperationMode(id, pCooperationMode);
		}
	}

	void CIntegratedMonitoringRouter::OnClientAlarmUse(int32_t id, const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendAlarmUse(id, pAlarmUse);
		}
	}

	void CIntegratedMonitoringRouter::OnClientCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendCollisionZoneLength(id, pCollisionZoneLength);
		}
	}

	void CIntegratedMonitoringRouter::OnClientCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendCollisionZoneDecelerationRate(id, pCollisionZoneDecelerationRate);
		}
	}

	void CIntegratedMonitoringRouter::OnClientRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRotorParameter(id, pRotorParameter);
		}
	}

	void CIntegratedMonitoringRouter::OnClientRotorControl(int32_t id, SHI::Data::StRotorControl* pRotorControl)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRotorControl(id, pRotorControl);
		}
	}

	void CIntegratedMonitoringRouter::OnClientRequestRotorParameter(int32_t id)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRequestRotorParameter(id);
		}
	}
#ifdef TEMP_KSG
	void CIntegratedMonitoringRouter::OnClientRequestSetNewParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		if(m_collisionManager)
		{
			m_collisionManager->SendRequestSetNewParameter(id, pRotorParameter);
		}
	}
#endif
	void CIntegratedMonitoringRouter::OnClientRequestCollisionZoneLength(int32_t id, SHI::Data::StCollisionRequestZone* pCollisionRequestZone)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRequestCollisionZoneLength(id, pCollisionRequestZone);
		}
	}

	void CIntegratedMonitoringRouter::OnClientRequestCollisionZoneDecelerationRate(int32_t id)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRequestCollisionZoneDecelerationRate(id);
		}
	}

	void CIntegratedMonitoringRouter::OnClientMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
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

	void CIntegratedMonitoringRouter::OnClientUpdateSensorManagerProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdateSensorManagerProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdateInterfaceProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdateInterfaceProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdateClusterProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdateClusterProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdateCollisionManagerProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdateCollisionManagerProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdateDistanceProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdateDistanceProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdateCollisionProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdateCollisionProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdatePLCProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendUpdatePLCProc(id, pUpdateData, size);
		}
	}

	void CIntegratedMonitoringRouter::OnClientUpdateMonitoring(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		// 짝이 없음...
	}

	void CIntegratedMonitoringRouter::OnClientRequestCollisionHistory(int32_t id)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRequestCollisionHistory(id);
		}
	}

	void CIntegratedMonitoringRouter::OnClientRequestCraneAttitude(int32_t id)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRequestCraneAttitude(id);
		}
	}

	void CIntegratedMonitoringRouter::OnClientRequestOperationHistory(int32_t id)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendRequestOperationHistory(id);
		}
	}
	
	inline void CIntegratedMonitoringRouter::OnClientPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
	{
		if(m_collisionManager)
		{
			m_collisionManager->SendPlcControlInfo(id, pControl);
		}

	}

	inline void CIntegratedMonitoringRouter::OnClientMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
	{
		if (m_collisionManager)
		{
			m_collisionManager->SendMaxDistance(id, pMaxDistance);
		}

	}
#ifdef TEMP_KSG
	void CIntegratedMonitoringRouter::OnServerRotorControlSocket(int32_t id, SHI::Data::StRotorControl* pRotorControl)
	{
		assert(false);
	}
#endif

}