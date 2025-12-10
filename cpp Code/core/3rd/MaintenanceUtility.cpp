#include "MaintenanceUtility.h"
#include "../DevLib/Include/Utility/Config.h"
#include "../DevLib/Include/Core/CTime/Time.h"
#include "../DevLib/Include/Core/CElapseTimer/ElapseTimer.h"
#include <windows.h>

namespace Utility
{

	CMaintenanceUtility::CMaintenanceUtility()
	{
		DevLib::CTimer::SetTimerType(false);
		DevLib::CTimer::StartTimer(1000 * 10, this, 1000 * 10);
		m_csMap.Initialize();

		// Read maintenance info from file
		//FILE* fp = fopen("./maintenance.bin", "rb");
		//if (fp)
		//{
		//	Utility::StMaintenance data;
		//	int nRead = 0;
		//	do
		//	{
		//		nRead = fread(fp, 1, sizeof(Utility::StMaintenance), fp);

		//		m_csMap.Enter();
		//		m_map[data.name] = data;
		//		m_csMap.Leave();
		//	} while (nRead > 0);
		//	fclose(fp);
		//}
	}

	CMaintenanceUtility::~CMaintenanceUtility()
	{
	}

	void CMaintenanceUtility::UpdateUsageTime(std::string keyValue)
	{
		// 마지막 갱신 이후 경과 시간(dt) 계산
		DevLib::CTime now;
		DevLib::CTime aa = m_lastUpdateTime[keyValue];
		DevLib::CTime dt = aa - now;

		m_lastUpdateTime[keyValue] = now;

		// Read info
		m_csMap.Enter();
		StMaintenance val = m_map[keyValue];
		m_csMap.Leave();

		if (val.usageTime == 0)
		{
			val.startTime = now;
			memset(val.name, 0, sizeof(val.name));
			sprintf(val.name, keyValue.data(), keyValue.length());
		}
		val.lastTime = now;
		val.usageTime = dt.m_time.time_t + val.usageTime;
		
		// Update info
		m_csMap.Enter();
		m_map[keyValue] = val;
		m_csMap.Leave();
	}

	void CMaintenanceUtility::OnTimer(void* p)
	{
		CMaintenanceUtility* pThis = reinterpret_cast<CMaintenanceUtility*>(p);
		std::map<std::string, Utility::StMaintenance> map;

		pThis->m_csMap.Enter();
		map = m_map;
		pThis->m_csMap.Leave();

		// Write maintenance info to temp file
		//FILE* fp = fopen("@maintenance.tmp", "wb");
		//if (fp)
		{
			std::map<std::string, Utility::StMaintenance>::iterator iter;
			for (iter = map.begin(); iter != map.end(); iter++)
			{
				//fwrite(fp, 1, sizeof(Utility::StMaintenance), fp);
				DevLib::Utility::Config::SetConfigInt("aaaa", iter->first.c_str(), iter->second.usageTime, "aaa.ini");
			}
			//fclose(fp);
		}
		//CopyFile("./@maintenance.tmp", "./maintenance.bin", FALSE);
	}
}