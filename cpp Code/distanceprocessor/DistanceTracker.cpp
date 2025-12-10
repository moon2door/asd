#include "DistanceTracker.h"

static unsigned int g_trackID = 0;

CDistanceTracker::CDistanceTracker()
	: m_distanceTolSq(0), m_trackTTL(0), m_maxTrack(0), m_nAverage(0)
{
}

CDistanceTracker::~CDistanceTracker()
{
}

void CDistanceTracker::Initalize(double distTol /*= 1.0*/, unsigned char trackTTL /*= 5*/, int nAverage/*=3*/, unsigned int maxTrack/*=256*/)
{
	m_distanceTolSq = distTol*distTol;
	m_trackTTL = trackTTL;
	m_maxTrack = maxTrack;
	m_nAverage = nAverage;
}

void CDistanceTracker::UpdateTrack(const SHI::DistanceInfoVectorPtr& vDistanceIn)
{
	// 갱신 상태 초기화
	for (std::list<StTrack>::iterator it = m_arTrack.begin(); it != m_arTrack.end(); ++it)
	{
		it->bUpdated = false;
	}

	// 거리 데이터 순회
	for (size_t i = 0; i < vDistanceIn->size(); i++)
	{
		std::list<StTrack>::iterator itor;
		if (ScanMatingTrack(vDistanceIn->at(i), itor, true)) // 트랙 정보가 있음
		{
			if (itor->bUpdated == false)
			{
				// 기존 트랙이 업데이트 되지 않았다면
				// 기존 트랙 갱신
				itor->bUpdated = true;
				itor->TTL = m_trackTTL;
				itor->Distance = vDistanceIn->at(i);
				itor->qDistance.push(vDistanceIn->at(i).Distance);
				if (itor->Status == STAT_NEW_TARGET)
				{
					itor->Status = STAT_NEW_UPDATED_TARGET;
				}
				if (itor->Status == STAT_NEW_UPDATED_TARGET)
				{
					itor->Status = STAT_NEW_UPDATED_TARGET2;
				}
				else
				{
					itor->Status = STAT_UPDATED_TARGET;
				}
				if (itor->qDistance.size() > m_nAverage) itor->qDistance.pop();

				// 누적 거리의 평균값 계산
				float average = 0;
				unsigned int sizeAccum = itor->qDistance.size();
				if (sizeAccum > 0)
				{
					for (unsigned int i = 0; i < sizeAccum; i++)
					{
						float d = itor->qDistance.front();
						average += d;
						itor->qDistance.pop();
						itor->qDistance.push(d);
					}
					average /= sizeAccum;
				}
				itor->Distance.Distance = average;
			}
			else
			{
				// 기존 트랙이 이미 업데이트 되었다면
				// 더 가까운 값을 사용한다.
				itor->Distance = vDistanceIn->at(i);
				itor->qDistance.back() = std::min(itor->qDistance.back(), vDistanceIn->at(i).Distance);
				
				// 누적 거리의 평균값 다시 계산
				float average = 0;
				unsigned int sizeAccum = itor->qDistance.size();
				if (sizeAccum > 0)
				{
					for (unsigned int i = 0; i < sizeAccum; i++)
					{
						float d = itor->qDistance.front();
						average += d;
						itor->qDistance.pop();
						itor->qDistance.push(d);
					}
					average /= sizeAccum;
				}
				itor->Distance.Distance = average;
			}
		}
		else
		{
			// 신규 트랙 생성
			if (m_arTrack.size() < m_maxTrack)
			{
				StTrack newTrack;
				newTrack.bUpdated = true;
				newTrack.TTL = m_trackTTL;
				newTrack.Status = STAT_NEW_TARGET;
				newTrack.Distance = vDistanceIn->at(i);
				newTrack.TrackID = g_trackID++;
				newTrack.qDistance.push(vDistanceIn->at(i).Distance);
				if (newTrack.qDistance.size() > m_nAverage) newTrack.qDistance.pop();

				m_arTrack.push_back(newTrack);
				//printf("## new track %d\n", newTrack.TrackID);
			}
		}
	}

	// 갱신되지 않은 트랙 업데이트
	int cnt = 0;
	int size = m_arTrack.size();
	std::list<StTrack>::iterator it = m_arTrack.begin();
	while (it != m_arTrack.end() )
	{
		if (it->bUpdated == false)
		{
			// 업데이트되지 않은 데이터 TTL 감소
			// 이전 상태가 STAT_NEW_TARGET이면서 업데이트되지 않은 데이터는 삭제됨
			it->TTL--;
			if (it->TTL > 0 && it->Status != STAT_NEW_TARGET)
			{
				it->bUpdated = true;
				it->Status = STAT_COASTED_TARGET;
				it->qDistance.push(it->qDistance.back());
				if (it->qDistance.size() > m_nAverage) it->qDistance.pop();
			}
			else
			{
				it = m_arTrack.erase(it);
				it--;
			}
		}
		else
		{
			//printf(" ## already updated. (%d) \n", it->TrackID);
			//printf(" ## already updated. \n");
		}
		it++;
	}
}

void CDistanceTracker::GetDistance(SHI::DistanceInfoVector& vDistanceOut)
{
	vDistanceOut.empty();
	for (std::list<StTrack>::iterator it = m_arTrack.begin(); it != m_arTrack.end(); ++it)
	{
		if (it->Status == STAT_UPDATED_TARGET
			|| it->Status == STAT_COASTED_TARGET )
		{
			//it->Distance.TrackID = it->TrackID;
			vDistanceOut.push_back(it->Distance);

			//printf("track %d, stat=%d, ttl=%d\n", it->TrackID, it->Status, it->TTL);
		}
		//printf("track %d, stat=%d, ttl=%d\n", it->TrackID, it->Status, it->TTL);
	}
}

float __SquaredDistFloat3__(float p1[3], float p2[3])
{
	return ((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]) + (p1[2] - p2[2])*(p1[2] - p2[2]));
}

bool CDistanceTracker::ScanMatingTrack(SHI::DistanceInfo& d, std::list<StTrack>::iterator &itor, bool bScanUpdatedTrack)
{
	bool ret = false;
	float minDist = FLT_MAX;
	std::list<StTrack>::iterator it, matingTrack;
	for (it= m_arTrack.begin(); it != m_arTrack.end(); ++it)
	{
		if (bScanUpdatedTrack == false)
		{
			// 이미 갱신되 트랙은 제외
			if (it->bUpdated == true) continue;
		}

		// 크레인 번호가 동일해야 함
		if (it->Distance.CraneIndex != d.CraneIndex) continue;

		// 시점, 종점의 위치 변화가 tolorance 이내여야 함
		if (__SquaredDistFloat3__(it->Distance.PosCrane.xyz, d.PosCrane.xyz) > m_distanceTolSq) continue;

		float distObject = __SquaredDistFloat3__(it->Distance.PosCluster.xyz, d.PosCluster.xyz);
		if (distObject < m_distanceTolSq)
		{
			// 앞의 조건을 충족하는 track 중, 이전 프레임의 물체 위치와 가장 가까운 값 사용
			if (distObject < minDist)
			{
				ret = true;
				minDist = distObject;
				matingTrack = it;
			}
		}
	}
	itor = matingTrack;
	return ret;
}
