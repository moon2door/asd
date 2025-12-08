#pragma once
#include <Data/StCraneMiniInfo.h>
#include <Config/CraneInfo.h>
#include <map>

class CModelCooperationMode
{
public:
	CModelCooperationMode()
	{
		//SHI::GetNumPier()
	}
	/*
	inline SHI::Data::StCooperationMode& GetCooperationMode() { return m_cooperationMode;  }

	inline void UpdateCooperationMode(SHI::Data::StCooperationMode* pCooperationMode)
	{
		m_cooperationMode = *pCooperationMode;
	}*/
private:
	std::map<int, SHI::Data::StCraneMiniInfo> m_craneMiniInfo;
};