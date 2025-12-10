
#include "CraneStatus.h"
#include <Config/CraneInfo.h>
#include <Routine/Include/Base/CTime.h>
#include <direct.h>

CCraneStatus::CCraneStatus()
{
	Routine::CTime t;
	t.UpdateCurrentTime();
	int year = t.Year();

	unsigned int numCranes = SHI::GetTotalNumCrane();
	for (int pier = 0; pier < SHI::GetNumPier(); pier++)
	{
		for (int crane = 0; crane < SHI::GetNumCrane(pier); crane++)
		{
			int id = SHI::GetCraneId(pier, crane);
			m_vAttitude[id] = new (std::nothrow)SHI::CraneAttitude;
			m_vUserinfo[id] = new (std::nothrow)StMonitoringUserInfo;
			m_vSystemStatus[id] = new (std::nothrow)StSystemStatusData;
			m_vHeartbeatCount[id] = 0;
			m_vHeartbeatCountPrev[id] = 0;

			memset(m_vAttitude[id], 0, sizeof(SHI::Data::StCraneAttitude));
			memset(m_vUserinfo[id], 0, sizeof(StMonitoringUserInfo));
			memset(m_vSystemStatus[id], 0, sizeof(StSystemStatusData));

			for (unsigned int dy = 0; dy < 5; dy++)
			{
				StCollisionHistory* history = new(std::nothrow) StCollisionHistory;
				memset(history, 0, sizeof(StCollisionHistory));
				m_vHistory[std::pair<int, int>(id, year - dy)] = history;

				StOperationHistory* operationHistory = new(std::nothrow) StOperationHistory;
				memset(operationHistory, 0, sizeof(StOperationHistory));
				m_vOperationHistory[std::pair<int, int>(id, year - dy)] = operationHistory;
			}
		}
	}

	m_colorMode = 0;
	m_numCooperations = 0;
}

CCraneStatus::~CCraneStatus()
{
	//std::map<int, SHI::CraneAttitude*>::iterator itAttitude;
	//std::map<std::pair<int, int>, StCollisionHistory*>::iterator itvHistory;
	//std::map<std::pair<int, int>, StOperationHistory*>::iterator itvOperationHistory;
	//std::map<int, StMonitoringUserInfo*>::iterator itvUserinfo;
	//std::map<int, int>::iterator itvHeartbeatCount;
	//std::map<int, int>::iterator itvHeartbeatCountPrev;
	//std::map<int, StSystemStatusData*>::iterator itvSystemStatus;

	//for (itAttitude = m_vAttitude.begin(); itAttitude != m_vAttitude.end(); ++itAttitude)
	//{
	//	if (itAttitude->second)
	//	{
	//		delete itAttitude->second;
	//		itAttitude->second = 0;
	//	}
	//}
}

void CCraneStatus::UpdateCraneAttitude(int id, SHI::Data::StCraneAttitude attitude)
{
	if (m_vAttitude.find(id) != m_vAttitude.end())
	{
		if (m_vAttitude[id])
		{
			memcpy_s(m_vAttitude[id], sizeof(SHI::CraneAttitude), &attitude, sizeof(SHI::Data::StCraneAttitude));
		}
	}
}

bool CCraneStatus::GetCraneAttitude(SHI::Data::StCraneAttitude& dst, int id)
{
	bool ret = false;
	if (m_vAttitude.find(id) != m_vAttitude.end())
	{
		if (m_vAttitude[id])
		{
			int pier = SHI::GetID2Pier(id);
			int crane = SHI::GetID2Crane(id);
			if (m_vAttitude[id]->numPart > 0 &&
				m_vAttitude[id]->craneId == crane &&
				m_vAttitude[id]->pierId == pier)
			{
				memcpy_s(&dst, sizeof(SHI::Data::StCraneAttitude), m_vAttitude[id], sizeof(SHI::CraneAttitude));
				ret = true;
			}
		}
	}
	return !dst.IsNull();
}

void CCraneStatus::UpdateCollisionHistory(int id, StCollisionHistory* history)
{
	int year = history->year;

	if (m_vHistory.find(std::pair<int, int>(id, year)) != m_vHistory.end())
	{
		if (m_vHistory[std::pair<int, int>(id, year)])
		{
			memcpy_s(m_vHistory[std::pair<int, int>(id, year)], sizeof(StCollisionHistory), history, sizeof(StCollisionHistory));
		}
	}
	else
	{
		m_vHistory[std::pair<int, int>(id, year)] = new (std::nothrow) StCollisionHistory;
		if (m_vHistory[std::pair<int, int>(id, year)])
		{
			memcpy_s(m_vHistory[std::pair<int, int>(id, year)], sizeof(StCollisionHistory), history, sizeof(StCollisionHistory));
		}
	}
}

bool CCraneStatus::GetCollisionHistory(StCollisionHistory* dst, int id, int year)
{
	bool ret = false;
	if (m_vHistory.find(std::pair<int, int>(id, year)) != m_vHistory.end())
	{
		if (m_vHistory[std::pair<int, int>(id, year)])
		{
			ret = true;
			memcpy_s(dst, sizeof(StCollisionHistory), m_vHistory[std::pair<int, int>(id, year)], sizeof(StCollisionHistory));
		}
	}
	return !dst->IsNull();
}

void CCraneStatus::UpdateOperationHistory(int id, StOperationHistory* history)
{
	int year = history->year;

	if (m_vOperationHistory.find(std::pair<int, int>(id, year)) != m_vOperationHistory.end())
	{
		if (m_vOperationHistory[std::pair<int, int>(id, year)])
		{
			memcpy_s(m_vOperationHistory[std::pair<int, int>(id, year)], sizeof(StOperationHistory), history, sizeof(StOperationHistory));
		}
	}
	else
	{
		m_vOperationHistory[std::pair<int, int>(id, year)] = new (std::nothrow) StOperationHistory;
		if (m_vOperationHistory[std::pair<int, int>(id, year)])
		{
			memcpy_s(m_vOperationHistory[std::pair<int, int>(id, year)], sizeof(StOperationHistory), history, sizeof(StOperationHistory));
		}
	}
}

bool CCraneStatus::GetOperationHistory(StOperationHistory* dst, int id, int year)
{
	bool ret = false;
	if (m_vOperationHistory.find(std::pair<int, int>(id, year)) != m_vOperationHistory.end())
	{
		if (m_vOperationHistory[std::pair<int, int>(id, year)])
		{
			ret = true;
			memcpy_s(dst, sizeof(StOperationHistory), m_vOperationHistory[std::pair<int, int>(id, year)], sizeof(StOperationHistory));
		}
	}
	return !dst->IsNull();
}

void CCraneStatus::UpdateHeartbeatCount(int id)
{
	if (m_vHeartbeatCount.find(id) != m_vHeartbeatCount.end())
	{
		m_vHeartbeatCount[id]++;
	}
}

int CCraneStatus::GetHeartbeatCount(int id)
{
	int ret = -1;
	if (m_vHeartbeatCount.find(id) != m_vHeartbeatCount.end())
	{
		ret = m_vHeartbeatCount[id];
	}
	return ret;
}

void CCraneStatus::UpdateHeartbeatCountPrev(int id, int count)
{
	if (m_vHeartbeatCountPrev.find(id) != m_vHeartbeatCountPrev.end())
	{
		m_vHeartbeatCountPrev[id] = count;
	}
}

int CCraneStatus::GetHeartbeatCountPrev(int id)
{
	int ret = -1;
	if (m_vHeartbeatCountPrev.find(id) != m_vHeartbeatCountPrev.end())
	{
		ret = m_vHeartbeatCountPrev[id];
	}
	return ret;
}

void CCraneStatus::UpdateSystemStatus(int id, StSystemStatusData *status)
{
	if (m_vSystemStatus.find(id) != m_vSystemStatus.end())
	{
		memcpy(m_vSystemStatus[id], status, sizeof(StSystemStatusData));
	}
}

bool CCraneStatus::GetSystemStatus(StSystemStatusData* dst, int id)
{
	int ret = false;
	if (m_vSystemStatus.find(id) != m_vSystemStatus.end())
	{
		if (m_vSystemStatus[id])
		{
			if (m_vSystemStatus[id]->messageId == 7)
			{
				memcpy(dst, m_vSystemStatus[id], sizeof(StSystemStatusData));
				ret = true;
			}
		}
	}
	return !dst->IsNull();
}

void CCraneStatus::UpdateUserInfo(int id, StMonitoringUserInfo* info)
{
	if (m_vUserinfo.find(id) != m_vUserinfo.end())
	{
		memcpy(m_vUserinfo[id], info, sizeof(StMonitoringUserInfo));
	}
}

 bool CCraneStatus::GetUserInfo(StMonitoringUserInfo* dst, int id)
 {
	 int ret = false;
	if (m_vUserinfo.find(id) != m_vUserinfo.end())
	{
		if (m_vUserinfo[id])
		{
			memcpy_s(dst, sizeof(StMonitoringUserInfo), m_vUserinfo[id], sizeof(StMonitoringUserInfo));
			ret = true;
		}
	}
	return !dst->IsNull();
}