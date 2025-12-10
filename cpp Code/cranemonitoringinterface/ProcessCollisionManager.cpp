#include "ProcessCollisionManager.h"
#include "ProcessCraneMonitoring.h"
#include <Routine/include/Base/RoutineUtility.h>

namespace SHI
{
	CProcessCollisionManager::CProcessCollisionManager()
		: m_collisionManager(this), m_unCompressedBuffer(new SHI::Data::StDistanceSocket)
	{
		const auto ip = Routine::GetConfigString("CollisionManager", "IP", "127.0.0.1");
		const auto port = Routine::GetConfigInt("CollisionManager", "PORT", 9098);
		m_collisionManager.Create(ip, port, sizeof(SHI::Data::StDistanceSocket));
		const auto reconnectTime = Routine::GetConfigInt("ReconnectTime", "Time", 10000);
		m_timerCollisionManager.StartTimer(reconnectTime, &CProcessCollisionManager::OnTimerCollisionManager, this);
	}

	// ReSharper disable once CppMemberFunctionMayBeStatic
	void CProcessCollisionManager::OnTimerCollisionManager()
	{
		if(m_countReceive == 0)
		{
			// Reset connection
			const auto ip = Routine::GetConfigString("CollisionManager", "IP", "127.0.0.1");
			const auto port = Routine::GetConfigInt("CollisionManager", "PORT", 9098);
			printf("Reset CollisionManager %s(%d)\n", ip.c_str(), port);
			m_collisionManager.GetSocketTCP()->Destroy();
			m_collisionManager.GetSocketTCP()->CreateTcpClient(ip, port);
		}
		m_countReceive = 0;
	}

	void CProcessCollisionManager::OnConnectedMonitoring(const std::string& ip, int32_t port)
	{
		printf("OnConnected CollisionManager %s(%d) \n", ip.c_str(), port);
	}

	void CProcessCollisionManager::OnDisconnectedMonitoring(const std::string& ip, int32_t port)
	{
		printf("OnDisconnected CollisionManager %s(%d) \n", ip.c_str(), port);
	}

	void CProcessCollisionManager::OnSystemStatus(SHI::Data::StSystemStatus* pSystemStatus)
	{
		m_countReceive++;
		printf("OnSystemStatus %lf \n", pSystemStatus->timeStamp);
		CProcessCraneMonitoring::Instance()->SendSystemStatus(pSystemStatus);
	}

	void CProcessCollisionManager::OnDistance(SHI::Data::StDistanceSocket* pDistance)
	{
		m_countReceive++;
		printf("GPS quality %d \n", pDistance->attitude.quality);//pjh

		CProcessCraneMonitoring::Instance()->SendDistance(pDistance);
	}

	void CProcessCollisionManager::OnDistanceCompressed(SHI::Data::StDistanceSocketCompressed* pDistance)
	{
		m_countReceive++;

#ifdef _DEBUG
		//printf("OnDistanceCompressed: lat:%.2f, att.lon:%.2f, hgt:%.1f\n", pDistance->attitude.gpsLatitude, pDistance->attitude.gpsLongitude, pDistance->attitude.height);
#endif
		printf("Received DistanceCompressed. GPS quality %d \n", pDistance->attitude.quality);//pjh
		if(m_compressor.Decode(*m_unCompressedBuffer, *pDistance))
		{
			CProcessCraneMonitoring::Instance()->SendDistance(m_unCompressedBuffer.get());
		}
	}

	void CProcessCollisionManager::OnResponseRotorParameter(SHI::Data::StRotorParameter* pRotorParameter)
	{
		m_countReceive++;
		CProcessCraneMonitoring::Instance()->SendMonitorResponseRotorParameter(pRotorParameter);
	}

	void CProcessCollisionManager::OnResponseCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		printf("%d = %lf, %lf, %lf, %lf \n", pCollisionZoneLength->nZone, pCollisionZoneLength->Zone1, pCollisionZoneLength->Zone2, pCollisionZoneLength->Zone3, pCollisionZoneLength->Zone4);
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendResponseCollisionZoneLength(pCollisionZoneLength);
	}

	void CProcessCollisionManager::OnResponseCollisionZoneDecelerationRate(SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendResponseCollisionZonepDecelerationRate(pCollisionZoneDecelerationRate);
	}

	void CProcessCollisionManager::OnMaintenanceInfo(SHI::Data::StMaintenanceInfo* pInfo)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendMaintenanceInfo(pInfo);
	}

	void CProcessCollisionManager::OnPLCInfo(SHI::Data::StPlcInfo* pInfo)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendPLCInfo(pInfo);
	}

	void CProcessCollisionManager::OnCooperationMode(SHI::Data::StCooperationMode* pCooperationMode)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendCooperationMode(pCooperationMode);
	}

	void CProcessCollisionManager::OnPlcControlInfo(SHI::Data::StPlcControlInfo* pControl)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendPlcControlInfo(pControl);
	}

	void CProcessCollisionManager::OnResponseCollisionHistory(SHI::Data::StCollisionHistory* history)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendCollisionHistory(history);
	}

	void CProcessCollisionManager::OnResponseOperationHistory(SHI::Data::StOperationHistory* history)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendOperationHistory(history);
	}

	void CProcessCollisionManager::OnMonitoringUserInfo(SHI::Data::StMonitoringUserInfo* pUserInfo)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendReportMonitoringUserInfo(pUserInfo);
	}

	void CProcessCollisionManager::OnMonitoringHeartBeat()
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendReportMonitoringHeartBeat();
	}	

	void CProcessCollisionManager::OnResponseCraneAttitude(SHI::Data::StCraneAttitude* attitude)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendCraneAttitude(attitude);
	}

	void CProcessCollisionManager::OnMonitoringCraneAlive()
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendCraneAlive();
	}

	void CProcessCollisionManager::OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info)
	{
		m_countReceive++;
		CProcessCraneMonitoring::Instance()->SendMiniInfo(info);
	}

	void CProcessCollisionManager::OnModelDistanceSocket(SHI::Data::StModelDistanceSocket* distance)
	{
		m_countReceive++;

		CProcessCraneMonitoring::Instance()->SendModelDistanceSocket(distance);
	}

	void CProcessCollisionManager::OnRotorControlSocket(SHI::Data::StRotorControl* rotorControl)
	{
		printf("---------- [%d]InterfaceRotor Order = %s\n", rotorControl->SensorNumber, rotorControl->ControlMotor.bStart > 0 ? "Start" : "Stop");//pjh
		CProcessCraneMonitoring::Instance()->SendRotorControlSocket(rotorControl);
	}

#ifdef TEMP_KSG
	void CProcessCollisionManager::OnRotorParameter(SHI::Data::StRotorParameter* rotorParameter)
	{
		CProcessCraneMonitoring::Instance()->SendMonitorResponseRotorParameter(rotorParameter);

	}
#endif
	void CProcessCollisionManager::OnMaxDistance(SHI::Data::StMaxDistance* pMaxDistance)
	{
		CProcessCraneMonitoring::Instance()->SendMaxDistance(pMaxDistance);
	}
}

