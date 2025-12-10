#include "ModelIntegratedMonitoringInfo.h"
#include <Config/CraneInfo.h>
#include <Routine/Include/Base/CTime.h>

CModelIntegratedMonitoringInfo::CModelIntegratedMonitoringInfo()
{
	for (unsigned int pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (unsigned int crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int id = SHI::GetCraneId(pier, crane);
			m_craneAttitude[id] = SHI::Data::StCraneAttitude();
			for (int dy = 0; dy < 5; dy++)
			{
				Routine::CTime t;
				t.UpdateCurrentTime();
				int year = t.Year() - dy;

				m_operationHistory[id][year] = SHI::Data::StOperationHistory();
				m_collisionHistory[id][year] = SHI::Data::StCollisionHistory();
			}
		}
	}
}

CModelIntegratedMonitoringInfo::~CModelIntegratedMonitoringInfo()
{
}

bool CModelIntegratedMonitoringInfo::UpdateMiniInfo(int id, const SHI::Data::StCraneMiniInfo& miniInfo)
{
	m_miniInfo[id] = miniInfo;

	return false;
}

bool CModelIntegratedMonitoringInfo::UpdateCraneAttitude(int id, const SHI::Data::StCraneAttitude& attitude)
{
	m_craneAttitude[id] = attitude;

	return false;
}

bool CModelIntegratedMonitoringInfo::UpdateOperationHistory(int id, int year, const SHI::Data::StOperationHistory& history)
{
	m_operationHistory[id][year] = history;

	return false;
}

bool CModelIntegratedMonitoringInfo::UpdateCollisionHistory(int id, int year, const SHI::Data::StCollisionHistory& history)
{
	m_collisionHistory[id][year] = history;

	return false;
}

bool CModelIntegratedMonitoringInfo::GetMiniInfo(int id, SHI::Data::StCraneMiniInfo& miniInfo)
{
	bool ret = false;
	if (m_miniInfo.find(id) != m_miniInfo.end())
	{
		miniInfo = m_miniInfo[id];
		ret = true;
	}
	return ret;
}

bool CModelIntegratedMonitoringInfo::GetCraneAttitude(int id, SHI::Data::StCraneAttitude& attitude)
{
	bool ret = false;
	if (m_craneAttitude.find(id) != m_craneAttitude.end())
	{
		attitude = m_craneAttitude[id];
		ret = true;
	}
	return ret;
}

bool CModelIntegratedMonitoringInfo::GetOperationHistory(int id, int year, SHI::Data::StOperationHistory& history)
{
	bool ret = false;
	if (m_operationHistory.find(id) != m_operationHistory.end())
	{
		if (m_operationHistory[id].find(year) != m_operationHistory[id].end())
		{
			history = m_operationHistory[id][year];
			ret = true;
		}
	}
	return ret;
}

bool CModelIntegratedMonitoringInfo::GetCollisionHistory(int id, int year, SHI::Data::StCollisionHistory& history)
{
	bool ret = false;
	if (m_collisionHistory.find(id) != m_collisionHistory.end())
	{
		if (m_collisionHistory[id].find(year) != m_collisionHistory[id].end())
		{
			history = m_collisionHistory[id][year];
			ret = true;
		}
	}
	return ret;
}