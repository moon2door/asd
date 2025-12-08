#include "ProcessCraneMonitoring.h"
#include "ProcessCollisionManager.h"
#include <Routine/include/Base/RoutineUtility.h>
#include <vector>
#include <string>

namespace SHI
{
	CProcessCraneMonitoring::CProcessCraneMonitoring()
		: m_craneMonitoringUnity(this)
	{
		//pjh
		/*const std::string ip = "127.0.0.1";
		const auto port = Routine::GetConfigInt("ServerMonitoring", "PORT", 9098);
		m_monitoring.Create(ip, port + 30000, sizeof(SHI::Data::StDistanceSocket));*/

		const auto ip = Routine::GetConfigString("CraneMonitoringUnity", "IP", "127.0.0.1");
		const auto port = Routine::GetConfigInt("CraneMonitoringUnity", "PORT", 9098);
		m_craneMonitoringUnity.Create(ip, port + 30000, sizeof(SHI::Data::StDistanceSocket));
		//
	}

	void CProcessCraneMonitoring::OnConnectedMonitoring(const std::string& ip, int32_t port)
	{
		printf("OnConnected Monitoring %s(%d) \n", ip.c_str(), port);
	}

	void CProcessCraneMonitoring::OnDisconnectedMonitoring(const std::string& ip, int32_t port)
	{
		printf("OnDisconnected Monitoring %s(%d) \n", ip.c_str(), port);
	}

	void CProcessCraneMonitoring::OnCooperationMode(SHI::Data::StCooperationMode* pCooperationMode)
	{
		CProcessCollisionManager::Instance()->SendCooperationMode(pCooperationMode);
	}

	void CProcessCraneMonitoring::OnAlarmUse(const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse)
	{
		//pjh sprintf_s(m_address, sizeof(m_address), "%d.%d.%d.%d", pAlarmUse->address[0], pAlarmUse->address[1], pAlarmUse->address[2], pAlarmUse->address[3]);
		//pjh
		int pos = 0;
		std::string _ip = ip;
		std::string delimiter = ".";
		uint8_t address[4] = { 0 };

		for (int i = 0; i < 4; i++)
		{
			pos = _ip.find(delimiter);
			if (pos != std::string::npos)
			{
				address[i] = static_cast<uint8_t>(std::stoi(_ip.substr(0, pos)));
				_ip.erase(0, pos + delimiter.length());
			}
			else
			{
				address[i] = static_cast<uint8_t>(std::stoi(_ip));
				break;
			}
		}

		for (int i = 0; i < 4; i++)
		{
			pAlarmUse->address[i] = address[i];
		}
		//~pjh
		printf("OnAlarmUse ip : %d.%d.%d.%d", pAlarmUse->address[0], pAlarmUse->address[1], pAlarmUse->address[2], pAlarmUse->address[3]);
		CProcessCollisionManager::Instance()->SendAlarmUse(pAlarmUse);
	}

	void CProcessCraneMonitoring::OnCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		CProcessCollisionManager::Instance()->SendCollisionZoneLength(pCollisionZoneLength);
	}

	void CProcessCraneMonitoring::OnCollisionZoneDecelerationRate(
		SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		CProcessCollisionManager::Instance()->SendCollisionZoneDecelerationRate(pCollisionZoneDecelerationRate);
	}

	void CProcessCraneMonitoring::OnRotorParameter(SHI::Data::StRotorParameter* pRotorParameter)
	{
		CProcessCollisionManager::Instance()->SendRotorParameter(pRotorParameter);
	}

	void CProcessCraneMonitoring::OnRotorControl(SHI::Data::StRotorControl* pRotorControl)
	{
		CProcessCollisionManager::Instance()->SendRotorControl(pRotorControl);
	}

	void CProcessCraneMonitoring::OnRequestRotorParameter()
	{
		CProcessCollisionManager::Instance()->SendRequestRotorParameter();
	}

#ifdef TEMP_KSG
	void CProcessCraneMonitoring::OnRequestSetNewParameter(SHI::Data::StRotorParameter* pRotorParameter)
	{
		CProcessCollisionManager::Instance()->SendSetNewParameter(pRotorParameter);
	}
#endif
	void CProcessCraneMonitoring::OnRequestCollisionZoneLength(SHI::Data::StCollisionRequestZone* pCollisionRequestZone)
	{
		CProcessCollisionManager::Instance()->SendRequestCollisionZoneLength(pCollisionRequestZone);
	}

	void CProcessCraneMonitoring::OnRequestCollisionZoneDecelerationRate()
	{
		CProcessCollisionManager::Instance()->SendRequestCollisionZoneDecelerationRate();
	}

	void CProcessCraneMonitoring::OnMonitoringUserInfo(SHI::Data::StMonitoringUserInfo* pUserInfo)
	{
		CProcessCollisionManager::Instance()->SendReportMonitoringUserInfo(pUserInfo);
	}

	void CProcessCraneMonitoring::OnWindInfo(SHI::Data::StWindInfo* info)
	{
		CProcessCollisionManager::Instance()->SendWindInfo(info);
	}

	void CProcessCraneMonitoring::OnPlcControlInfo(SHI::Data::StPlcControlInfo* pControl)
	{
		CProcessCollisionManager::Instance()->SendPlcControlInfo(pControl);
	}

	void CProcessCraneMonitoring::OnMonitoringOnLabelInfo(SHI::Data::StMonitoringLabelInfo* pInfo)
	{
	}

	void CProcessCraneMonitoring::OnRequestCollisionHistory()
	{
		CProcessCollisionManager::Instance()->SendRequestCollisionHistory();
	}

	void CProcessCraneMonitoring::OnRequestOprtationHistory()
	{
		CProcessCollisionManager::Instance()->SendRequestOperationHistory();
	}

	void CProcessCraneMonitoring::OnRequestCraneAttitude()
	{
		CProcessCollisionManager::Instance()->SendRequestCraneAttitude();
	}

	void CProcessCraneMonitoring::OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info)
	{
		CProcessCollisionManager::Instance()->SendMiniInfo(info);
	}

	void CProcessCraneMonitoring::OnMaxDistance(SHI::Data::StMaxDistance* info)
	{
		printf("%f \n", info->maxDistance);
		CProcessCollisionManager::Instance()->SendMaxDistance(info);
	}
	void CProcessCraneMonitoring::OnRotorControlSocket(SHI::Data::StRotorControl* pRotorContorl)
	{
		CProcessCollisionManager::Instance()->SendRotorControlSocket(pRotorContorl);
	}
}

