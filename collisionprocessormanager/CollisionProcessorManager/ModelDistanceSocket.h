#pragma once

#include <Routine/Include/Base/RoutineUtility.h>
#include <Routine/Include/Base/CCriticalSection.h>

#include <Data/StDistanceSocket.h>

class CModelDistanceSocket
{
public:
	CModelDistanceSocket()
	{
		m_pDistanceSocket = new SHI::Data::StDistanceSocket;
		memset(m_pDistanceSocket, 0, sizeof(SHI::Data::StDistanceSocket));		
	}
	~CModelDistanceSocket()
	{
		delete m_pDistanceSocket;
	}

	inline void LockDistanceSocket() { m_csLockDistance.Lock();  }
	inline void UnLockDistanceSocket() { m_csLockDistance.UnLock();  }

	inline SHI::Data::StDistanceSocket* GetDistanceSocket() { return m_pDistanceSocket;  }

private:
	Routine::CCriticalSection		m_csLockDistance;
	SHI::Data::StDistanceSocket*	m_pDistanceSocket;
};