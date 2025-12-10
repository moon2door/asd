#include "ModelCraneStatus.h"
#include <Config/CraneInfo.h>

CModelCraneStatus::CModelCraneStatus()
{
	for (uint32_t pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (uint32_t crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int32_t id = SHI::GetCraneId(pier, crane);
			m_craneStatus[id] = SHI::Data::StCraneStatus();
		}
	}
}


CModelCraneStatus::~CModelCraneStatus()
{
}

void CModelCraneStatus::UpdateCraneStatus(int32_t id, const SHI::Data::StCraneStatus& status)
{
	m_craneStatus[id] = status;
}

 bool CModelCraneStatus::UpdateCraneStatus(int32_t id, const SHI::Data::StPlcInfo& info)
{
	bool ret = false;
	if (m_craneStatus.find(id) != m_craneStatus.end())
	{
		m_craneStatus[id].PLCInfo = info;
		ret = true;
	}
	return ret;
}

bool CModelCraneStatus::UpdateCraneStatus(int32_t id, const SHI::Data::StMaintenanceInfo& info)
{
	bool ret = false;
	if (m_craneStatus.find(id) != m_craneStatus.end())
	{
		m_craneStatus[id].maintenanceInfo = info;
		ret = true;
	}
	return ret;
}

bool CModelCraneStatus::UpdateCraneStatus(int32_t id, const SHI::Data::StSystemStatus& status)
{
	bool ret = false;
	if (m_craneStatus.find(id) != m_craneStatus.end())
	{
		m_craneStatus[id].SystemStatus = status;
		ret = true;
	}
	return ret;
}

bool CModelCraneStatus::UpdateCraneStatus(int32_t id, const SHI::Data::StMonitoringUserInfo& info)
{
	bool ret = false;
	if (m_craneStatus.find(id) != m_craneStatus.end())
	{
		m_craneStatus[id].DriverInfo = info;
		ret = true;
	}
	return ret;
}

bool CModelCraneStatus::GetCraneStatus(int32_t id, SHI::Data::StCraneStatus& status)
{
	bool ret = false;
	if (m_craneStatus.find(id) != m_craneStatus.end())
	{
		status = m_craneStatus[id];
		ret = true;
	}
	return ret;
}
