#pragma once

#include <Config/CraneInfo.h>
#include <Data/StCraneMiniInfo.h>
#include <map>

class CModelCraneMiniInfo
{
public:
	CModelCraneMiniInfo()
	{
	}

	inline SHI::Data::StCraneMiniInfo GetCraneMiniInfo(int id)
	{
		SHI::Data::StCraneMiniInfo info = {0,};
		if (m_craneMiniInfo.find(id) != m_craneMiniInfo.end())
		{
			info = m_craneMiniInfo[id];
		}
		return info;
	}

	inline void UpdateMiniInfo(SHI::Data::StCraneMiniInfo* info)
	{
		int pier = info->attitude.pierId;
		int crane = info->attitude.craneId;
		int id = SHI::GetCraneId(pier, crane);

		m_craneMiniInfo[id] = *info;
	}

private:
	std::map<int, SHI::Data::StCraneMiniInfo> m_craneMiniInfo;

};