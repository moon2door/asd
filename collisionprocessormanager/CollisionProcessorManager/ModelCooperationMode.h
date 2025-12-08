#pragma once

class CModelCooperationMode
{
public:
	CModelCooperationMode()
	{
		memset(&m_cooperationMode, 0, sizeof(SHI::Data::StCooperationMode));
	}
	
	inline SHI::Data::StCooperationMode& GetCooperationMode() { return m_cooperationMode;  }

	inline void UpdateCooperationMode(SHI::Data::StCooperationMode* pCooperationMode)
	{
		m_cooperationMode = *pCooperationMode;
	}
private:
	SHI::Data::StCooperationMode	m_cooperationMode;

};