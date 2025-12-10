#pragma once

#include "CraneObject.h"

namespace SHI
{

	void CCraneObject::SetCraneAttitude(SHI::CraneAttitude& attitude)
	{
		SHI::CraneAttitudePtr tmp = SHI::CraneAttitudePtr(new SHI::CraneAttitude);
		*tmp = attitude;
		m_attitude = tmp;
	}

	void CCraneObject::SetClusterParam(SHI::ClusterParam& param)
	{
		SHI::ClusterParamPtr tmp = SHI::ClusterParamPtr(new SHI::ClusterParam);
		*tmp = param;
		m_clusterParam = tmp;
	}

	void CCraneObject::SetDistanceParam(SHI::DistanceParam& param)
	{
		SHI::DistanceParamPtr tmp = DistanceParamPtr(new SHI::DistanceParam);
		*tmp = param;
		m_distanceParam = tmp;
	}

	void CCraneObject::SetDBInfo(SHI::Data::StDBInfo& info)
	{
		std::shared_ptr<SHI::Data::StDBInfo> tmp = std::shared_ptr<SHI::Data::StDBInfo>(new SHI::Data::StDBInfo);
		*tmp = info;
		m_dbInfo = tmp;
	}

	void CCraneObject::SetPlcInfo(SHI::Data::StPlcInfo& info)
	{
		std::shared_ptr<SHI::Data::StPlcInfo> tmp = std::shared_ptr<SHI::Data::StPlcInfo>(new SHI::Data::StPlcInfo);
		*tmp = info;
		m_plcInfo = tmp;
	}

	void CCraneObject::SetRefPartPoints(std::vector<SHI::PointCloudPtr>& data)
	{
		std::shared_ptr<std::vector<SHI::PointCloudPtr>> tmp = std::shared_ptr<std::vector<SHI::PointCloudPtr>>(new std::vector<SHI::PointCloudPtr>);
		*tmp = data;
		m_vRefPartPoints = tmp;
	}

}