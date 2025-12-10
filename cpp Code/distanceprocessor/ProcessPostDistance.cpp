#include "../DistanceProcessor.h"
#include <algorithm>


void CDistanceProcessor::ProcessPostDistance(SHI::DistanceInfoVector& distance, SHI::DistanceInfoVector& hookDistance, std::vector<unsigned char>& distanceLabel)
{
	// 거리 데이터 병합
	SHI::DistanceInfoVector result;
	result.insert(result.end(), distance.begin(), distance.end());
	result.insert(result.end(), hookDistance.begin(), hookDistance.end());

	// 거리 데이터 정렬
	std::sort(result.begin(), result.end());

	// 라벨 생성
	for (unsigned int i=0; i<result.size(); i++)
	{
		if (result.at(i).CraneIndex == 200) distanceLabel.push_back(SHI::DISTANCE_HOOK);
		else distanceLabel.push_back(SHI::DISTANCE_NORMAL);
	}
	distance = result;
}
