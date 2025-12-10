
#include "ProcessPing.h"
#include <Config/CraneInfo.h>

namespace IntegratedRouter
{
	CProcessPing::CProcessPing()
	{
	}

	CProcessPing::~CProcessPing()
	{
	}

	void CProcessPing::AddPinger(int32_t id, const char* address)
	{
		if (m_pingers.find(id) == m_pingers.end())
		{
			CPingProcesser *pinger = new (std::nothrow) CPingProcesser(address);
			if (pinger)
			{
				m_pingers[id] = pinger;
			}
		}
	}

	bool CProcessPing::IsPingConnected(int32_t id)
	{
		bool ret = false;
		if (m_pingers.find(id) != m_pingers.end())
		{
			if (m_pingers[id])
			{
				ret = m_pingers[id]->IsConnected();
			}
		}
		return ret;
	}

}