#pragma once

#include <Routine/Include/Base/CTimer.h>
#include <Routine/Include/Base/CThread.h>
#include <Routine/Include/Base/CEvent.h>

// DB
#include <Object/DBConnector/DBConnector.h>

class CModelDBMSConnector
{
public:
	CModelDBMSConnector()
	{
		m_eUpdateCollisionInformation.Create();

		m_dataDistance = new SHI::Data::StDistance;
		m_dbConnector = new SHI::Object::CDBConnector;
		m_dataCollisionInformation = new SHI::Data::StCollisionInformation;

		m_dbConnector->Create();
	}

	bool StartTimerConnetDBMS()
	{
		m_threadDBMS.StartThread(&CModelDBMSConnector::OnUpdatedDistance, this);
		return m_timerDBMSConnector.StartTimer(60000, &CModelDBMSConnector::OnTimerConnetDBMS, this);
	}

	void StopTimerConnetDBMS()
	{
		m_timerDBMSConnector.StopTimer();
	}

	bool StartTimerUpdateHeading()
	{
		return m_timerUpdateHeading.StartTimer(1000, &CModelDBMSConnector::OnTimerUpdateCraneInfo, this);
	}

	void StopTimerUpdateHeading()
	{
		m_timerUpdateHeading.StopTimer();
	}

	bool StartTimerUpdateCooperation()
	{
		return m_timerUpdateCooperation.StartTimer(10000, &CModelDBMSConnector::OnTimerUpdateCooperation, this);
	}

	void StopTimerUpdateCooperation()
	{
		m_timerUpdateCooperation.StopTimer();
	}

	inline SHI::Data::StDistance *GetDistance()
	{
		return m_dataDistance;
	}

	void UpdateDistanceData(SHI::Data::StDistance* pDistance)
	{
		*m_dataDistance = *pDistance;
	}

	void UpdateCollisionInformation(SHI::Data::StCollisionInformation* pData)
	{
		memcpy(m_dataCollisionInformation, pData, pData->GetSize());
		m_eUpdateCollisionInformation.SetEvent();
	}

	virtual void OnTimerConnetDBMS() = 0;
	virtual void OnTimerUpdateCraneInfo() = 0;
	virtual void OnTimerUpdateCooperation() = 0;
	virtual void OnUpdatedDistance() = 0;

protected:
	Routine::CTimer						m_timerDBMSConnector;
	Routine::CTimer						m_timerUpdateHeading;
	Routine::CTimer						m_timerUpdateCooperation;
	SHI::Object::CDBConnector*			m_dbConnector;
	SHI::Data::StDistance*				m_dataDistance;
	SHI::Data::StCollisionInformation*	m_dataCollisionInformation;
	Routine::CThread					m_threadDBMS;
	Routine::CEvent						m_eUpdateCollisionInformation;


	//static void _OnTimerConnetDBMS(void* p)
	//{
	//	((CModelDBMSConnector*)p)->OnTimerConnetDBMS();
	//}
	//
	//static void _OnTimerUpdateHeading(void* p)
	//{
	//	((CModelDBMSConnector*)p)->OnTimerUpdateCraneInfo();
	//}
	//
	//static void _OnTimerUpdateCooperation(void* p)
	//{
	//	((CModelDBMSConnector*)p)->OnTimerUpdateCooperation();
	//}
	//
	//
	//static void _OnUpdatedCluster(void* p)
	//{
	//	while (((CModelDBMSConnector*)p)->m_threadDBMS.IsRunThread())
	//	{
	//		if(((CModelDBMSConnector*)p)->m_eUpdateCollisionInformation.WaitForEvent(100)) ((CModelDBMSConnector*)p)->OnUpdatedDistance();
	//	}
	//}
};