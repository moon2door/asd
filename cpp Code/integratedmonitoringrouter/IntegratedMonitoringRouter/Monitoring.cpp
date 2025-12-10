#include "Monitoring.h"
#include <Routine/include/Base/RoutineUtility.h>
#include <Config/CraneInfo.h>

namespace IntegratedRouter
{
	CMonitoring::CMonitoring(CObserverMonitoring* pObserver)
		: m_pObserver(pObserver)
	{
	}

	CMonitoring::~CMonitoring()
	{
	}

	bool CMonitoring::Create()
	{
		bool ret = false;
		for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
		{
			for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
			{
				int32_t id = SHI::GetCraneId(pier, crane);
				std::string strPier = SHI::ConvPierStr(pier);
				std::string strCrane = SHI::ConvCraneStr(pier, crane);
				std::string fieldName = strPier + "_" + strCrane;

				//pjh
				/*char ip[32] = "127.0.0.1";
				char bindIp[32] = "127.0.0.1";*/
				std::string ip = Routine::GetConfigString(fieldName.c_str(), "integratedMonitoringInterfaceIP", "127.0.0.1", "IntegratedMonitoringRouter.json");
				//

				//pjh
				/*int32_t port = Routine::GetConfigInt(fieldName.c_str(), "monitoringPort", 19098);
				int32_t poerUdp = port + 1;*/

				int32_t port = Routine::GetConfigInt(fieldName.c_str(), "integratedMonitoringInterfacePort", 10000 + pier * 10 + crane, "IntegratedMonitoringRouter.json");
				//

				SHI::Interface::Monitoring::Server::CServerMonitoringLabeled* objMonitoring;
				objMonitoring = new SHI::Interface::Monitoring::Server::CServerMonitoringLabeled(this, id);
				bool bCreated = objMonitoring->Create(ip, port, sizeof(SHI::Data::StDistanceSocket));
				//if (bCreated)
				//{
				//	printf("[Remote %s] %s:%d create succeed.\n", fieldName.c_str(), monitoringIP, monitoringTCPPort);
				//}
				//else
				//{
				//	printf("[Remote %s] %s:%d create failed.\n", fieldName.c_str(), monitoringIP, monitoringTCPPort);
				//}
				m_integratedmonitoringInterfaces[id] = objMonitoring;
			}
		}
		return ret;
	}

	uint32_t CMonitoring::GetNumConnections(int32_t id)
	{
		uint32_t nConnection = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				nConnection = m_integratedmonitoringInterfaces[id]->GetSocketTCP()->GetClientInfo().size();
			}
		}
		return nConnection;
	}

	uint32_t CMonitoring::SendSystemStatus(int32_t id, SHI::Data::StSystemStatus* pSystemStatus)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendSystemStatus(pSystemStatus);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendDistance(int32_t id, SHI::Data::StDistanceSocket* pDistance)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendDistance(pDistance);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendDistanceCompressed(pDistance);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendCollisionInformation(int32_t id, SHI::Data::StCollisionInformation* pCollisionInformation)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendCollisionInformation(pCollisionInformation);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendResponseRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendMonitorResponseRotorParameter(pRotorParameter);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendResponseCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendResponseCollisionZoneLength(pCollisionZoneLength);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendResponseCollisionZonepDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendResponseCollisionZonepDecelerationRate(pCollisionZoneDecelerationRate);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendUpdateMonitoring(int32_t id, unsigned char* pData, uint32_t size)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendUpdateMonitoring(pData, size);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendCollisionHistory(int32_t id, SHI::Data::StCollisionHistory* history)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendCollisionHistory(history);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendOperationHistory(int32_t id, SHI::Data::StOperationHistory* history)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendOperationHistory(history);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendReportMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendReportMonitoringUserInfo(pUserInfo);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendReportMonitoringHeartBeat(int32_t id)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendReportMonitoringHeartBeat();
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendCraneAttitude(int32_t id, SHI::Data::StCraneAttitude* attitude)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendCraneAttitude(attitude);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendCraneAlive(int32_t id)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendCraneAlive();
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendMiniInfo(int32_t id, SHI::Data::StCraneMiniInfo* info)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendMiniInfo(info);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendMaintenanceInfo(int32_t id, SHI::Data::StMaintenanceInfo* info)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendMaintenanceInfo(info);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendPLCInfo(int32_t id, SHI::Data::StPlcInfo* info)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendPLCInfo(info);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendModelDistanceSocket(int32_t id, SHI::Data::StModelDistanceSocket* distance)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendModelDistanceSocket(distance);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendCooperationMode(pCooperationMode);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendPlcControlInfo(pControl);
			}
		}
		return ret;
	}

	uint32_t CMonitoring::SendMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendMaxDistance(pMaxDistance);
			}
		}
		return ret;
	}

#ifdef TEMP_KSG
	uint32_t CMonitoring::SendRotorContorl(int32_t id, SHI::Data::StRotorControl* pRotorControl)
	{
		uint32_t ret = 0;
		if (m_integratedmonitoringInterfaces.find(id) != m_integratedmonitoringInterfaces.end())
		{
			if (m_integratedmonitoringInterfaces[id])
			{
				ret = m_integratedmonitoringInterfaces[id]->SendRotorControlSocket(pRotorControl);
			}
		}
		return ret;
	}
#endif
	void CMonitoring::OnClientConnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientConnectedMonitoring(id, ip, port);
		}
	}

	void CMonitoring::OnClientDisconnectedMonitoring(int32_t id, const std::string& ip, int32_t port)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientDisconnectedMonitoring(id, ip, port);
		}
	}

	void CMonitoring::OnClientCooperationMode(int32_t id, SHI::Data::StCooperationMode* pCooperationMode)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientCooperationMode(id, pCooperationMode);
		}
	}

	void CMonitoring::OnClientAlarmUse(int32_t id, const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientAlarmUse(id, ip, port, pAlarmUse);
		}
	}

	void CMonitoring::OnClientCollisionZoneLength(int32_t id, SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientCollisionZoneLength(id, pCollisionZoneLength);
		}
	}

	void CMonitoring::OnClientCollisionZoneDecelerationRate(int32_t id, SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientCollisionZoneDecelerationRate(id, pCollisionZoneDecelerationRate);
		}
	}

	void CMonitoring::OnClientRotorParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRotorParameter(id, pRotorParameter);
		}
	}

	void CMonitoring::OnClientRotorControl(int32_t id, SHI::Data::StRotorControl* pRotorControl)
	{
		if (m_pObserver)
		{
			printf("ReceiveRotorContorl\n");
			m_pObserver->OnClientRotorControl(id, pRotorControl);
		}
	}

	void CMonitoring::OnClientRequestRotorParameter(int32_t id)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRequestRotorParameter(id);
		}
	}

#ifdef TEMP_KSG
	void CMonitoring::OnClientRequestSetNewParameter(int32_t id, SHI::Data::StRotorParameter* pRotorParameter)
	{
		if(m_pObserver)
		{
			m_pObserver->OnClientRequestSetNewParameter(id, pRotorParameter);
		}		
	}
#endif
	void CMonitoring::OnClientRequestCollisionZoneLength(int32_t id, SHI::Data::StCollisionRequestZone* pCollisionRequestZone)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRequestCollisionZoneLength(id, pCollisionRequestZone);
		}
	}

	void CMonitoring::OnClientRequestCollisionZoneDecelerationRate(int32_t id)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRequestCollisionZoneDecelerationRate(id);
		}
	}

	void CMonitoring::OnClientMonitoringUserInfo(int32_t id, SHI::Data::StMonitoringUserInfo* pUserInfo)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientMonitoringUserInfo(id, pUserInfo);
		}
	}

	void CMonitoring::OnClientWindInfo(int32_t id, SHI::Data::StWindInfo* info)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientWindInfo(id, info);
		}
	}

	void CMonitoring::OnClientPlcControlInfo(int32_t id, SHI::Data::StPlcControlInfo* pControl)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientPlcControlInfo(id, pControl);
		}
	}

	void CMonitoring::OnClientMaxDistance(int32_t id, SHI::Data::StMaxDistance* pMaxDistance)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientMaxDistance(id, pMaxDistance);
		}
	}

	void CMonitoring::OnClientUpdateSensorManagerProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateSensorManagerProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdateInterfaceProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateInterfaceProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdateClusterProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateClusterProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdateCollisionManagerProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateCollisionManagerProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdateDistanceProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateDistanceProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdateCollisionProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateCollisionProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdatePLCProc(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdatePLCProc(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientUpdateMonitoring(int32_t id, unsigned char* pUpdateData, uint32_t size)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientUpdateMonitoring(id, pUpdateData, size);
		}
	}

	void CMonitoring::OnClientRequestCollisionHistory(int32_t id)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRequestCollisionHistory(id);
		}
	}

	void CMonitoring::OnClientRequestCraneAttitude(int32_t id)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRequestCraneAttitude(id);
		}
	}

	void CMonitoring::OnClientRequestOperationHistory(int32_t id)
	{
		if (m_pObserver)
		{
			m_pObserver->OnClientRequestOperationHistory(id);
		}
	}

}