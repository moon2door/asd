
#include "ModelCollisionZoneLength.h"
#include <Data/StCollisionZoneLength.h>
#include <Config/CollisionInfo.h>

CModelCollisionZoneLength::CModelCollisionZoneLength()
{
	//pjh
	m_zone[0].dists[0] = Routine::GetConfigDouble("CollisionZone0", "Length4", 2, "./CollisionZone.json");
	m_zone[0].dists[1] = Routine::GetConfigDouble("CollisionZone0", "Length3", 4, "./CollisionZone.json");
	m_zone[0].dists[2] = Routine::GetConfigDouble("CollisionZone0", "Length2", 8, "./CollisionZone.json");

	m_zone[1].dists[0] = Routine::GetConfigDouble("CollisionZone1", "Length4", 1, "./CollisionZone.json");
	m_zone[1].dists[1] = Routine::GetConfigDouble("CollisionZone1", "Length3", 2, "./CollisionZone.json");
	m_zone[1].dists[2] = Routine::GetConfigDouble("CollisionZone1", "Length2", 3, "./CollisionZone.json");

	m_zone[2].dists[0] = Routine::GetConfigDouble("CollisionZone2", "Length4", 2, "./CollisionZone.json");
	m_zone[2].dists[1] = Routine::GetConfigDouble("CollisionZone2", "Length3", 4, "./CollisionZone.json");
	m_zone[2].dists[2] = Routine::GetConfigDouble("CollisionZone2", "Length2", 8, "./CollisionZone.json");

	m_zone[3].dists[0] = Routine::GetConfigDouble("CollisionZone3", "Length4", 1, "./CollisionZone.json");
	m_zone[3].dists[1] = Routine::GetConfigDouble("CollisionZone3", "Length3", 2, "./CollisionZone.json");
	m_zone[3].dists[2] = Routine::GetConfigDouble("CollisionZone3", "Length2", 3, "./CollisionZone.json");

	m_zone[4].dists[0] = Routine::GetConfigDouble("CollisionZone4", "Length4", 2, "./CollisionZone.json");
	m_zone[4].dists[1] = Routine::GetConfigDouble("CollisionZone4", "Length3", 4, "./CollisionZone.json");
	m_zone[4].dists[2] = Routine::GetConfigDouble("CollisionZone4", "Length2", 8, "./CollisionZone.json");

	m_zone[5].dists[0] = Routine::GetConfigDouble("CollisionZone5", "Length4", 1, "./CollisionZone.json");
	m_zone[5].dists[1] = Routine::GetConfigDouble("CollisionZone5", "Length3", 2, "./CollisionZone.json");
	m_zone[5].dists[2] = Routine::GetConfigDouble("CollisionZone5", "Length2", 3, "./CollisionZone.json");

	m_zone[6].dists[0] = Routine::GetConfigDouble("CollisionZone6", "Length4", 1, "./CollisionZone.json");
	m_zone[6].dists[1] = Routine::GetConfigDouble("CollisionZone6", "Length3", 2, "./CollisionZone.json");
	m_zone[6].dists[2] = Routine::GetConfigDouble("CollisionZone6", "Length2", 3, "./CollisionZone.json");

	/*m_zone[6].dists[0] = Routine::GetConfigDouble("CollisionZone6", "Length3", 1, "./CollisionZone.json");
	m_zone[6].dists[1] = Routine::GetConfigDouble("CollisionZone6", "Length2", 2, "./CollisionZone.json");
	m_zone[6].dists[2] = Routine::GetConfigDouble("CollisionZone6", "Length1", 3, "./CollisionZone.json");*/
	//~pjh
}

void CModelCollisionZoneLength::UpdateCollisionZoneLength(SHI::Data::StCollisionZoneLength* pZoneLength)
{
	int32_t iZone = pZoneLength->nZone;
	if (0 <= iZone && iZone < 7)
	{
		m_zone[iZone].dists[2] = pZoneLength->Zone2;//pjh
		m_zone[iZone].dists[1] = pZoneLength->Zone3;//pjh
		m_zone[iZone].dists[0] = pZoneLength->Zone4;//pjh
	}
}

CModelCollisionZoneLength::StZone CModelCollisionZoneLength::GetCollisionZone(int32_t pier, int32_t crane, int32_t part)
{
	return m_zone[SHI::GetCollisionIndex(pier, crane, part)];
}

CModelCollisionZoneLength::StZone CModelCollisionZoneLength::GetCollisionZoneException(int32_t pier, int32_t crane, int32_t part)
{
	return m_zone[SHI::GetCollisionIndex(pier, crane, part) + 1];
}

uint32_t CModelCollisionZoneLength::GetCollisionIndex(int32_t pier, int32_t crane, int32_t part)
{
	return SHI::GetCollisionIndex(pier, crane, part);
}

uint32_t CModelCollisionZoneLength::GetCollisionIndexException(int32_t pier, int32_t crane, int32_t part)
{
	return SHI::GetCollisionIndex(pier, crane, part)+1;
}
