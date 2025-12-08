#pragma once

#include <Routine/Include/Base/RoutineUtility.h>

class CModelCollisionZone
{
public:
	CModelCollisionZone()
	{
		m_collisionZoneLength = new SHI::Data::StCollisionZoneLength[10];
		m_collisionZoneDecelerationRate = new SHI::Data::StCollisionZoneDecelerationRate;
		memset(m_collisionZoneLength, 0, sizeof(SHI::Data::StCollisionZoneLength) * 10);
		memset(m_collisionZoneDecelerationRate, 0, sizeof(SHI::Data::StCollisionZoneDecelerationRate));

		m_collisionZoneDecelerationRate->Zone1 = (float)Routine::GetConfigDouble("CollisionZoneDecelerationRate", "DecelerationRate1", 50.0, "./CollisionZone.json");
		m_collisionZoneDecelerationRate->Zone2 = (float)Routine::GetConfigDouble("CollisionZoneDecelerationRate", "DecelerationRate2", 20.0, "./CollisionZone.json");
		m_collisionZoneDecelerationRate->Zone3 = (float)Routine::GetConfigDouble("CollisionZoneDecelerationRate", "DecelerationRate3", 0.0, "./CollisionZone.json");

		for (int32_t i = 0; i < 7; i++)
		{
			char StrZone[32] = "";
			sprintf_s(StrZone, "CollisionZone%d", i);
			m_collisionZoneLength[i].nZone = i;
			m_collisionZoneLength[i].Zone1 = (float)Routine::GetConfigDouble(StrZone, "Length1", 6.0, "./CollisionZone.json");
			m_collisionZoneLength[i].Zone2 = (float)Routine::GetConfigDouble(StrZone, "Length2", 4.0, "./CollisionZone.json");
			m_collisionZoneLength[i].Zone3 = (float)Routine::GetConfigDouble(StrZone, "Length3", 3.0, "./CollisionZone.json");
			m_collisionZoneLength[i].Zone4 = (float)Routine::GetConfigDouble(StrZone, "Length4", 2.0, "./CollisionZone.json");
		}
	}

	inline SHI::Data::StCollisionZoneLength& GetCollisionZoneLength(int32_t nZone) { return m_collisionZoneLength[nZone]; }
	inline SHI::Data::StCollisionZoneDecelerationRate& GetCollisionZoneDecelerationRate() { return *m_collisionZoneDecelerationRate; }

	inline void UpdateCollisionZoneLength(SHI::Data::StCollisionZoneLength* pCollisionZoneLength)
	{
		m_collisionZoneLength[pCollisionZoneLength->nZone] = *pCollisionZoneLength;

		char StrZone[32] = "";
		sprintf_s(StrZone, "CollisionZone%d", pCollisionZoneLength->nZone);
		Routine::SetConfigDouble(StrZone, "Length1", pCollisionZoneLength->Zone1, "./CollisionZone.json");
		Routine::SetConfigDouble(StrZone, "Length2", pCollisionZoneLength->Zone2, "./CollisionZone.json");
		Routine::SetConfigDouble(StrZone, "Length3", pCollisionZoneLength->Zone3, "./CollisionZone.json");
		Routine::SetConfigDouble(StrZone, "Length4", pCollisionZoneLength->Zone4, "./CollisionZone.json");

	}

	inline void UpdateCollisionZoneDecelerationRate(SHI::Data::StCollisionZoneDecelerationRate* pCollisionZoneDecelerationRate)
	{
		*m_collisionZoneDecelerationRate = *pCollisionZoneDecelerationRate;

		Routine::SetConfigDouble("CollisionZoneDecelerationRate", "DecelerationRate1", pCollisionZoneDecelerationRate->Zone1, "./CollisionZone.json");
		Routine::SetConfigDouble("CollisionZoneDecelerationRate", "DecelerationRate2", pCollisionZoneDecelerationRate->Zone2, "./CollisionZone.json");
		Routine::SetConfigDouble("CollisionZoneDecelerationRate", "DecelerationRate3", pCollisionZoneDecelerationRate->Zone3, "./CollisionZone.json");
	}


private:
	SHI::Data::StCollisionZoneLength*				m_collisionZoneLength;
	SHI::Data::StCollisionZoneDecelerationRate*		m_collisionZoneDecelerationRate;
};