
#include "ProcessDBPlayer.h"
#include <Routine/include/Base/CElapseTimer.h>
#include <Data/StDistanceSocketCompressed.h>
#include <Config/CraneInfo.h>
#include <3rd/pcl/3rdParty/Boost/include/boost-1_74/boost/shared_ptr.hpp>
#include <3rd/pcl/3rdParty/Boost/include/boost-1_74/boost/make_shared.hpp>
#include <windows.h>
#include <map>
#include <vector>

CProcessDBPlayer::CProcessDBPlayer(IntegratedRouter::CProcessDBConnector& dbConnector, CObserverDBPlayer* pObserver)
	: m_dbConnector(dbConnector)
{
	m_pObserver.reset(pObserver);

	for (unsigned int iPier = 0; iPier < SHI::GetNumPier(); iPier++)
	{
		for (unsigned int iCrane = 0; iCrane < SHI::GetNumCrane(iPier); iCrane++)
		{
			int id = SHI::GetCraneId(iPier, iCrane);
			m_decoders[id] = std::make_unique<SHI::Compressor::CCompressDistance>();
		}
	}

	m_bPlay = false;
}

CProcessDBPlayer::~CProcessDBPlayer()
{
}

bool CProcessDBPlayer::IsPlaying()
{
	return m_thread.IsRunThread();
}

bool CProcessDBPlayer::StartLoading(int pier, SHI::Data::StDateTime date)
{
	m_thread.StopThread();
	m_thread.WaitForEndThread();

	m_pier = pier;
	m_date = date;
	m_bPlay = false;
	bool ret = m_thread.StartThread(&CProcessDBPlayer::Run, this);
	return ret;
}

bool CProcessDBPlayer::StartPlay(int pier, SHI::Data::StDateTime date)
{
	bool ret = false;
	if (m_thread.IsRunThread() == false)
	{
		m_pier = pier;
		m_date = date;
		m_bPlay = true;
		ret = m_thread.StartThread(&CProcessDBPlayer::Run, this);
	}
	return ret;
}

void CProcessDBPlayer::StopPlay()
{
	m_bPlay = false;
	m_thread.StopThread();
}

SHI::Data::StDateTime CProcessDBPlayer::GetSelectedDate()
{
	return m_date;
}

void CProcessDBPlayer::Run()
{
	printf("====================================\n");
	std::vector<int> craneId;
	for (unsigned int iCrane = 0; iCrane < SHI::GetNumCrane(m_pier); iCrane++)
	{
		int id = SHI::GetCraneId(m_pier, iCrane);
		craneId.push_back(id);
	}
	
	if (m_thread.IsRunThread())
	{
		// Find initial index
		SHI::Data::StDateTime startTime(9999);
		SHI::Data::StDateTime endTime(0);
		for (const int& id : craneId)
		{
			int pier = SHI::GetID2Pier(id);
			int crane = SHI::GetID2Crane(id);

			unsigned int numDistance = m_dbConnector.ReadNumDistanceCompressed(id, m_date.year, m_date.month, m_date.date);

			if (numDistance == 0)
			{   
				printf("ERROR : Null DistanceCompressed, Pier : %d, CraneID : %d\n", pier, crane);
				continue; //pjh return;
			}
			std::shared_ptr<SHI::Data::StDistanceSocketCompressed> distance = std::make_shared<SHI::Data::StDistanceSocketCompressed>();
			SHI::Data::StDateTime curStartTime;
			SHI::Data::StDateTime curEndTime;
			bool bReadBegin = m_dbConnector.ReadDistanceCompressed(id, m_date.year, m_date.month, m_date.date, *distance, curStartTime, 0);
			bool bReadEnd = m_dbConnector.ReadDistanceCompressed(id, m_date.year, m_date.month, m_date.date, *distance, curEndTime, numDistance - 1);
			if (bReadBegin && bReadEnd)
			{
				if (curStartTime < startTime)
				{
					startTime = curStartTime;
				}
				
				if (endTime < curEndTime)
				{
					endTime = curEndTime;
				}
			}
		}

		m_startTime = startTime;
		m_endTime = endTime;
		m_syncTime = m_date;

		if (m_syncTime < m_startTime)
		{
			m_syncTime = m_startTime;
		}
	}

	m_pObserver->OnDBPlayerInfo(m_startTime, m_endTime, m_syncTime);
	if (m_endTime <= m_startTime)
	{
		m_bPlay = false;
	}

	// Run DB
	Routine::CElapseTimer elapsed;
	std::map<int, std::shared_ptr<SHI::Data::StDistanceSocketCompressed>> collisionInfo;
	std::map<int, std::shared_ptr<SHI::Data::StCraneStatus>> craneStatus;
	while (m_thread.IsRunThread() && m_bPlay)
	{
		printf("-----------------\n");
		printf("SyncTime %02d:%02d %02d\n", m_syncTime.hour, m_syncTime.min, m_syncTime.sec);
		for (const int& id : craneId)
		{
			std::shared_ptr<SHI::Data::StDistanceSocketCompressed> distance = std::make_shared<SHI::Data::StDistanceSocketCompressed>();

			if (m_dbConnector.FindDistanceCompressed(id, m_syncTime.year, m_syncTime.month, m_syncTime.date, *distance, m_syncTime))
			{
				//float time = distance->attitude.gpsTimestamp;
				//if (time > 0)
				//{
				//	int hour = (((((int)time) / 10000) % 100) + 9) % 24;
				//	int min = (((int)time) / 100) % 100;
				//	int sec = ((int)time) % 100;
				//	int msec = ((int)time * 100) % 100;

				//	SHI::Data::StDateTime t = m_syncTime;
				//	t.hour = hour;
				//	t.min = min;
				//	t.sec = sec;

				//	int pier = distance->attitude.pierId;
				//	int crane = distance->attitude.craneId;
				//	printf("Read %s read %02d:%02d %02d - sync %02d:%02d %02d\n",
				//		SHI::ConvCraneStr(pier, crane).c_str(),
				//		 t.hour, t.min, t.sec, m_syncTime.hour, m_syncTime.min, m_syncTime.sec);
				//}
				collisionInfo[id] = distance;
			}

			if (!m_bPlay) break;

			std::shared_ptr<SHI::Data::StCraneStatus> curCraneStatus = std::make_shared<SHI::Data::StCraneStatus>();

			if (m_dbConnector.FindStatus(id, m_syncTime.year, m_syncTime.month, m_syncTime.date, *curCraneStatus, m_syncTime))
			{
				//printf("find status\n");
				craneStatus[id] = curCraneStatus;
			}

			if (!m_bPlay) break;
		}

		if (!m_bPlay) break;
		m_pObserver->OnDBPlayerInfo(m_startTime, m_endTime, m_syncTime);
		
		for (const auto& collision : collisionInfo)
		{
			int pier = collision.second->attitude.pierId;
			int crane = collision.second->attitude.craneId;
			int id = SHI::GetCraneId(pier, crane);
			float time = collision.second->attitude.gpsTimestamp;

			int hour = (((((int)time) / 10000) % 100) + 9) % 24;
			int min = (((int)time) / 100) % 100;
			int sec = ((int)time) % 100;
			int msec = ((int)time * 100) % 100;

			SHI::Data::StDateTime curTime = m_syncTime;
			curTime.hour = hour;
			curTime.min = min;
			curTime.sec = sec;

			printf("Buffered %s (%02d:%02d %02d)\n", SHI::ConvCraneStr(pier, crane).c_str(), curTime.hour, curTime.min, curTime.sec);

			if (m_pObserver)
			{
				boost::shared_ptr<SHI::Data::StDistanceSocket> distanceData = boost::make_shared<SHI::Data::StDistanceSocket>();
				if (m_decoders[id]->Decode(*distanceData, *collision.second))
				{
					m_pObserver->OnDBPlayerDistance(pier, crane, distanceData.get());
				}

				if (craneStatus.find(id) != craneStatus.end())
				{
					m_pObserver->OnCraneStatus(pier, crane, craneStatus[id].get());
				}
			}
			if (!m_bPlay) break;
		}
		if (!m_bPlay) break;
		
		m_syncTime.sec++;

		if (m_syncTime.sec >= 60)
		{
			m_syncTime.sec = 0;
			m_syncTime.min++;
		}

		if (m_syncTime.min >= 60)
		{
			m_syncTime.min = 0;
			m_syncTime.hour++;
		}

		if (m_syncTime.hour >= 24)
		{
			m_syncTime.hour = 0;
			m_syncTime.date++;
		}

		int elapsedTimeMs = elapsed.GetElapseTime() * 0.001;
		int dTimeMs = 1000 - elapsedTimeMs;
		if (dTimeMs > 0)
		{
			Sleep(500);
		}
	}
}