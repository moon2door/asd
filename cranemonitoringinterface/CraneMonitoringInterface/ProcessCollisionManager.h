#pragma once
#include <Routine/include/Routines/Template/CSingletonT.h>
#include <Routine/include/Base/CTimer.h>
#include <Interface/Monitoring/Client/ClientMonitoring.h>
#include <Utility/Compressor.h>

namespace SHI
{
	class CProcessCollisionManager
		: public Routine::CSingletonT<CProcessCollisionManager>
		, public Interface::Monitoring::Client::CObserverClientMonitoring
	{
	public:
		CProcessCollisionManager();
		virtual ~CProcessCollisionManager() = default;

	private:		
		Interface::Monitoring::Client::CClientMonitoring m_collisionManager;

		/// <summary>
		/// Connect to CollisionProcessorManager
		/// </summary>
		/// <param name="ip"></param>
		/// <param name="port"></param>
		void OnConnectedMonitoring(const std::string& ip, int32_t port) override;
		void OnDisconnectedMonitoring(const std::string& ip, int32_t port) override;
		void OnSystemStatus(SHI::Data::StSystemStatus* pSystemStatus) override;
		void OnDistance(SHI::Data::StDistanceSocket* pDistance) override;
		void OnDistanceCompressed(SHI::Data::StDistanceSocketCompressed* pDistance) override;
		void OnResponseRotorParameter(SHI::Data::StRotorParameter* pRotorParameter) override;
		void OnResponseCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZoneLength) override;
		void OnResponseCollisionZoneDecelerationRate(SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate) override;
		void OnMaintenanceInfo(SHI::Data::StMaintenanceInfo* pInfo) override;
		void OnPLCInfo(SHI::Data::StPlcInfo* pInfo) override;
		void OnCooperationMode(SHI::Data::StCooperationMode* pCooperationMode) override;
		void OnPlcControlInfo(SHI::Data::StPlcControlInfo* pControl) override;
		void OnResponseCollisionHistory(SHI::Data::StCollisionHistory* history) override;
		void OnResponseOperationHistory(SHI::Data::StOperationHistory* history) override;
		void OnMonitoringUserInfo(SHI::Data::StMonitoringUserInfo* pUserInfo) override;
		void OnMonitoringHeartBeat() override;
		void OnResponseCraneAttitude(SHI::Data::StCraneAttitude* attitude) override;
		void OnMonitoringCraneAlive() override;
		void OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info) override;
		void OnModelDistanceSocket(SHI::Data::StModelDistanceSocket* distance) override;
		void OnRotorControlSocket(SHI::Data::StRotorControl* rotorControl) override;
#ifdef TEMP_KSG
		void OnRotorParameter(SHI::Data::StRotorParameter* pRotorParameter) override;
#endif
		void OnMaxDistance(SHI::Data::StMaxDistance* pControl) override;

		uint32_t m_countReceive = 0;
		Routine::CTimer m_timerCollisionManager;
		void OnTimerCollisionManager();

		Compressor::CCompressDistance m_compressor;
		std::shared_ptr<SHI::Data::StDistanceSocket> m_unCompressedBuffer;

	public:
		CProcessCollisionManager(const CProcessCollisionManager& other) = delete;
		CProcessCollisionManager(CProcessCollisionManager&&) = delete;
		CProcessCollisionManager& operator=(const CProcessCollisionManager&) = delete;
		CProcessCollisionManager& operator=(CProcessCollisionManager&&) = delete;
		Interface::Monitoring::Client::CClientMonitoring* operator->()
		{
			return &m_collisionManager;
		}

		Interface::Monitoring::Client::CClientMonitoring& operator*()
		{
			return m_collisionManager;
		}
	};
}

