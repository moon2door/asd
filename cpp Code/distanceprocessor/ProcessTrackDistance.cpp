#include "../DistanceProcessor.h"

void CDistanceProcessor::ProcessTrackDistance(SHI::DistanceInfoVector& dst, const SHI::DistanceInfoVectorPtr& src, SHI::DistanceInfoVector& dstHook, const SHI::DistanceInfoVectorPtr& srcHook)
{
	m_tracker->UpdateTrack(src);
	m_tracker->GetDistance(dst);
	m_trackerHook->UpdateTrack(srcHook);
	m_trackerHook->GetDistance(dstHook);
}