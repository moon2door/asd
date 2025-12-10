#include "../DistanceProcessor.h"
#include <Utility/Filter.h>
#include <Utility/Transform.h>

bool IsInRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi);

bool IsInTriangularRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi, const SHI::CraneAttitudePtr& attitude);

bool IsInRartRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi, const SHI::CraneAttitudePtr& attitude);

bool IsInRartOnlyRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi, const SHI::CraneAttitudePtr& attitude);

void CDistanceProcessor::ProcessExceptionDistance(std::vector<unsigned char> &labels, const SHI::DistanceInfoVectorPtr& vDistance, const SHI::DistanceParamPtr& distanceParam, const SHI::CraneAttitudePtr& attitude)
{
	if (distanceParam)
	{
		SHI::CraneRoiPtr craneRoi = distanceParam->roiException;

		for (unsigned int i = 0; i < vDistance->size(); i++)
		{

			if( IsInRoi(vDistance->at(i), craneRoi) ) // 일반 ROI 필터 처리
			{
				if (labels[i] == SHI::DISTANCE_NORMAL) labels[i] = SHI::DISTANCE_EXCEPTION;
				else if (labels[i] == SHI::DISTANCE_HOOK) labels[i] = SHI::DISTANCE_HOOK_EXCEPTION;
				else {}
			}
			else if (IsInRartRoi(vDistance->at(i), craneRoi, attitude)) // 파트와 함께 움직이는 ROI
			{
				if (labels[i] == SHI::DISTANCE_NORMAL) labels[i] = SHI::DISTANCE_EXCEPTION_PART;
				else if (labels[i] == SHI::DISTANCE_HOOK) labels[i] = SHI::DISTANCE_HOOK_EXCEPTION_PART;
				else {}
			}
			else if (IsInRartOnlyRoi(vDistance->at(i), craneRoi, attitude)) // 특정 파트만 적용되는 ROI
			{
				if (labels[i] == SHI::DISTANCE_NORMAL) labels[i] = SHI::DISTANCE_EXCEPTION_PARTONLY;
				else if (labels[i] == SHI::DISTANCE_HOOK) labels[i] = SHI::DISTANCE_HOOK_EXCEPTION_PARTONLY;
				else {}
			}
			else if (IsInTriangularRoi(vDistance->at(i), craneRoi, attitude)) // 특수 ROI 필터 처리(삼각형 ROI)
			{
				if (labels[i] == SHI::DISTANCE_NORMAL) labels[i] = SHI::DISTANCE_EXCEPTION_TRIANGULAR;
				else if (labels[i] == SHI::DISTANCE_HOOK) labels[i] = SHI::DISTANCE_HOOK_EXCEPTION_TRIANGULAR;
				else {}
			}
			//else if (vDistance->at(i).Distance <distanceParam->minDistance // 근거리, 원거리 데이터 예외 처리
			//	  || vDistance->at(i).Distance >distanceParam->maxDistance)
			//{
			//	if (labels[i] == SHI::DISTANCE_NORMAL) labels[i] = SHI::DISTANCE_EXCEPTION_OUTOFRANGE;
			//	else {}
			//}
			else
			{
			}
		}
	}
}

bool IsInRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi)
{
	bool bInRoi = false;
	SHI::Point3D posCluster(distance.PosCluster.X, distance.PosCluster.Y, distance.PosCluster.Z);
	for (unsigned int iFilter = 0; iFilter < craneRoi->vRoi.size(); iFilter++)
	{
		SHI::StROI roi = craneRoi->vRoi.at(iFilter);
		bInRoi = bInRoi || roi.IsInlier(posCluster);
		if (bInRoi == true) break;
	}
	return bInRoi;
}

bool IsInTriangularRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi, const SHI::CraneAttitudePtr& attitude)
{
	bool bInRoi = false;
	SHI::Point3D posCluster(distance.PosCluster.X, distance.PosCluster.Y, distance.PosCluster.Z);
	for (unsigned int iFilter = 0; iFilter < craneRoi->triangular.size(); iFilter++)
	{
		SHI::StTriangularROI roi = craneRoi->triangular.at(iFilter);
		bInRoi = bInRoi || roi.IsInlier(posCluster, attitude);
		if (bInRoi == true) break;
	}
	return bInRoi;
}

bool IsInRartRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi, const SHI::CraneAttitudePtr& attitude)
{
	bool bInRoi = false;
	if (craneRoi && attitude)
	{
		// 모든 파트 ROI 확인
		for (unsigned int iPart = 0; iPart<attitude->numPart && bInRoi == false; iPart++)
		{
			// 파트 좌표 변환
			SHI::Point3D posCluster(distance.PosCluster.X, distance.PosCluster.Y, distance.PosCluster.Z);
			SHI::Point3D posTrans;
			SHI::Transform::TransformPointInv(posCluster, posTrans, *attitude, iPart);

			// 파트 ROI 필터 처리
			for (unsigned int iFilter = 0; iFilter < craneRoi->part[iPart].size() && bInRoi == false; iFilter++)
			{
				SHI::StROI roi = craneRoi->part[iPart].at(iFilter);
				bInRoi = bInRoi || roi.IsInlier(posTrans);
			}
		}
	}
	return bInRoi;
}

bool IsInRartOnlyRoi(const SHI::DistanceInfo &distance, const SHI::CraneRoiPtr& craneRoi, const SHI::CraneAttitudePtr& attitude)
{
	bool bInRoi = false;
	int part = distance.CraneIndex;
	if (distance.CraneIndex < 5)
	{
		// 파트 좌표 변환
		SHI::Point3D posCluster(distance.PosCluster.X, distance.PosCluster.Y, distance.PosCluster.Z);
		SHI::Point3D posTrans;
		SHI::Transform::TransformPointInv(posCluster, posTrans, *attitude, part);

		// 파트 ROI 필터 처리
		for (unsigned int iFilter = 0; iFilter < craneRoi->partOnly[part].size(); iFilter++)
		{
			SHI::StROI roi = craneRoi->partOnly[part].at(iFilter);
			bInRoi = bInRoi || roi.IsInlier(posTrans);
			if (bInRoi == true) break;
		}
	}
	return bInRoi;
}

