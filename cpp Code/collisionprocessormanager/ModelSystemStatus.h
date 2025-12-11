#pragma once

#include <direct.h>
#include <Routine/Include/Base/CTimer.h>
#include <Routine/Include/Base/RoutineUtility.h>

class CModelSystemStatus
{
public:
	CModelSystemStatus()
	{
		m_nGpsFrames = 0;
		m_nRouterFrames = 0;
		m_bGpsDataValid = false;

		memset(&m_infoPLC, 0, sizeof(SHI::Data::StPlcInfo));
		memset(&m_statusRotor, 0, sizeof(SHI::Data::StRotorStatus));
		memset(&m_statusSystem, 0, sizeof(SHI::Data::StSystemStatus));

		m_statusSystem.bUseAlarm = Routine::GetConfigInt("Alarm", "Used", 0) == 0 ? false : true;

		Routine::CTime time;
		char fname[256] = "";
		_mkdir("./AlarmControlLog");
		sprintf_s(fname, "./AlarmControlLog/%04d-%02d-%02d.txt", time.Year(), time.Month(), time.Day());
		FILE* fp = fopen(fname, "at");
		if (fp)
		{
			std::string t = time.GetTimeToString();
			if (m_statusSystem.bUseAlarm == 1)
			{
				fprintf(fp, "%s: 충돌방지 매니저 재시작 Alarm \"Use\"\n", t.c_str());
			}
			else
			{
				fprintf(fp, "%s: 충돌방지 매니저 재시작 Alarm \"Not use\"\n", t.c_str());
			}
			fclose(fp);
		}
	}

	// 타이머
	virtual void OnTimerReportSystemStatus() = 0;

	bool StartTimerReportSystemStatus()
	{
		return m_timerReportSystemStatus.StartTimer(1000, &CModelSystemStatus::OnTimerReportSystemStatus, this);
	}

	void StopTimerReportSystemStatus()
	{
		m_timerReportSystemStatus.StopTimer();
	}

	inline SHI::Data::StPlcInfo& GetPLCInfo() { return m_infoPLC; }
	inline SHI::Data::StRotorStatus& GetRotorStatus() { return m_statusRotor; }
	inline SHI::Data::StSystemStatus& GetSystemStatus() { return m_statusSystem;  }

	inline void UpdatePLCInfo(SHI::Data::StPlcInfo* info)
	{
		m_infoPLC = *info;
	}

	inline void UpdateRotorStatus(SHI::Data::StRotorStatus* pRotorStatus)
	{
		m_statusRotor = *pRotorStatus;
	}

	inline uint32_t& gpsFrames()
	{
		return m_nGpsFrames;
	}

	inline uint32_t& numZeroFrame()
	{
		return m_nZeroFrames;
	}

	inline uint32_t& routerFrames()
	{
		return m_nRouterFrames;
	}

	inline bool IsGpsValid()
	{
		return m_bGpsDataValid;
	}

	inline void SetGpsValid(bool bGpsDataValid)
	{
		m_bGpsDataValid = bGpsDataValid;
	}


private:
	SHI::Data::StPlcInfo		m_infoPLC;
	SHI::Data::StRotorStatus	m_statusRotor;
	SHI::Data::StSystemStatus	m_statusSystem;
	Routine::CTimer				m_timerReportSystemStatus;
	uint32_t m_nZeroFrames;
	uint32_t m_nRouterFrames;
	bool m_bGpsDataValid;

public:
	uint32_t m_nGpsFrames;
};
