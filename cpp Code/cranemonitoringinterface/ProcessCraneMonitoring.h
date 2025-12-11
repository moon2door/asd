#pragma once
#include <Routine/include/Routines/Template/CSingletonT.h>
#include <Interface/Monitoring/Server/ServerMonitoring.h>

namespace SHI
{
	class CProcessCraneMonitoring
		: public Routine::CSingletonT<CProcessCraneMonitoring>
		, public Interface::Monitoring::Server::CObserverServerMonitoring
	{
	public:
		CProcessCraneMonitoring();
		virtual ~CProcessCraneMonitoring() = default;

	private:
		void OnConnectedMonitoring(const std::string& ip, int32_t port) override;
		void OnDisconnectedMonitoring(const std::string& ip, int32_t port) override;
		void OnCooperationMode(SHI::Data::StCooperationMode* pCooperationMode) override;
		void OnAlarmUse(const std::string& ip, uint16_t port, SHI::Data::StAlarmUse* pAlarmUse) override;
		void OnCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZoneLength) override;
		void OnCollisionZoneDecelerationRate(SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate) override;
		void OnRotorParameter(SHI::Data::StRotorParameter* pRotorParameter) override;
		void OnRotorControl(SHI::Data::StRotorControl* pRotorControl) override;
		void OnRequestRotorParameter() override;
#ifdef TEMP_KSG
		void OnRequestSetNewParameter(SHI::Data::StRotorParameter* pRotorParameter) override;
#endif
		void OnRequestCollisionZoneLength(SHI::Data::StCollisionRequestZone* pCollisionRequestZone) override;
		void OnRequestCollisionZoneDecelerationRate() override;
		void OnMonitoringUserInfo(SHI::Data::StMonitoringUserInfo* pUserInfo) override;
		void OnWindInfo(SHI::Data::StWindInfo* info) override;
		void OnPlcControlInfo(SHI::Data::StPlcControlInfo* pControl) override;
		void OnMonitoringOnLabelInfo(SHI::Data::StMonitoringLabelInfo* pInfo) override;
		void OnMaxDistance(SHI::Data::StMaxDistance* pMaxDistance) override;
		void OnRotorControlSocket(SHI::Data::StRotorControl* pRotorContorl) override;

		// 통합관제
		void OnRequestCollisionHistory() override;
		void OnRequestOprtationHistory() override;
		void OnRequestCraneAttitude() override;
		void OnMonitoringMiniInfo(SHI::Data::StCraneMiniInfo* info) override;

		Interface::Monitoring::Server::CServerMonitoring m_craneMonitoringUnity;

	public:
		CProcessCraneMonitoring(const CProcessCraneMonitoring& other) = delete;
		CProcessCraneMonitoring(CProcessCraneMonitoring&&) = delete;
		CProcessCraneMonitoring& operator=(const CProcessCraneMonitoring&) = delete;
		CProcessCraneMonitoring& operator=(CProcessCraneMonitoring&&) = delete;

		Interface::Monitoring::Server::CServerMonitoring* operator->()
		{
			return &m_craneMonitoringUnity;
		}

		Interface::Monitoring::Server::CServerMonitoring& operator*()
		{
			return m_craneMonitoringUnity;
		}
	};
}

