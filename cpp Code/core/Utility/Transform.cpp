
#include <Utility/Transform.h>
#include <Utility/UTM.h>
#include <Utility/mathematics.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <Config/CraneGps.h>
#include <Config/CraneInfo.h>
#include "Geometric/Geometric.h"

namespace SHI
{
	namespace Transform
	{
		void TransformPoint(const SHI::Point3D& src, SHI::Point3D& dst, const SHI::CraneAttitude& attitude, unsigned int idxPart)
		{
			SHI::PointCloudPtr pc(new SHI::PointCloud);
			pc->push_back(src);
			TransformPointCloud(pc, *pc, attitude, idxPart);
			dst = pc->at(0);
		}

		void TransformPointInv(const SHI::Point3D& src, SHI::Point3D& dst, const SHI::CraneAttitude& attitude, unsigned int idxPart)
		{
			SHI::PointCloudPtr pc(new SHI::PointCloud);
			pc->push_back(src);
			TransformPointCloudInv(pc, *pc, attitude, idxPart);
			dst = pc->at(0);
		}

		void TransformPointCloud(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::Point3D& center, const SHI::Point3D& rotation, const SHI::Point3D& translation)
		{
			SHI::PointCloud cloudTmp;

			Eigen::Affine3f trans1 = Eigen::Affine3f::Identity();
			trans1 = Eigen::Affine3f::Identity();
			trans1.translation() += Eigen::Vector3f(-center.x, -center.y, -center.z);
			pcl::transformPointCloud(*src, cloudTmp, trans1);

			Eigen::Affine3f trans2 = Eigen::Affine3f::Identity();
			trans2.rotate(Eigen::AngleAxisf(pcl::deg2rad(rotation.x), Eigen::Vector3f::UnitX()));
			trans2.rotate(Eigen::AngleAxisf(pcl::deg2rad(rotation.y), Eigen::Vector3f::UnitY()));
			trans2.rotate(Eigen::AngleAxisf(pcl::deg2rad(rotation.z), Eigen::Vector3f::UnitZ()));
			trans2.translation() += Eigen::Vector3f(center.x + translation.x, center.y + translation.y, center.z + translation.z);
			pcl::transformPointCloud(cloudTmp, dst, trans2);
		}

		void TransformPointCloud(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::CraneAttitude& attitude, unsigned int idxPart)
		{
			if (attitude.numPart > idxPart)
			{
				TransformPointCloud(src, dst, attitude.jointInfo[idxPart], attitude.pose[idxPart]);
			}
		}

		void TransformPointCloud(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::StJointInfo& joint, float_t f)
		{
			SHI::Point3D center(joint.cx, joint.cy, joint.cz);
			SHI::Point3D rotation(f * joint.rx, f * joint.ry, f * joint.rz);
			SHI::Point3D translation(f * joint.tx, f * joint.ty, f * joint.tz);
			TransformPointCloud(src, dst, center, rotation, translation);
		}

		void TransformPointCloudInv(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::Point3D& center, const SHI::Point3D& rotation, const SHI::Point3D& translation)
		{
			SHI::PointCloud cloudTmp;

			// Undo translation before rotating so the offset is not rotated as well.
			Eigen::Affine3f translate = Eigen::Affine3f::Identity();
			translate.translation() = Eigen::Vector3f(-(center.x + translation.x), -(center.y + translation.y), -(center.z + translation.z));
			pcl::transformPointCloud(*src, cloudTmp, translate);

			Eigen::Affine3f rotate = Eigen::Affine3f::Identity();
			rotate.rotate(Eigen::AngleAxisf(pcl::deg2rad(-rotation.x), Eigen::Vector3f::UnitX()));
			rotate.rotate(Eigen::AngleAxisf(pcl::deg2rad(-rotation.y), Eigen::Vector3f::UnitY()));
			rotate.rotate(Eigen::AngleAxisf(pcl::deg2rad(-rotation.z), Eigen::Vector3f::UnitZ()));
			rotate.translation() = Eigen::Vector3f(center.x, center.y, center.z);
			pcl::transformPointCloud(cloudTmp, dst, rotate);
		}

		void TransformPointCloudInv(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::CraneAttitude& attitude, unsigned int idxPart)
		{
			if (attitude.numPart > idxPart)
			{
				TransformPointCloudInv(src, dst, attitude.jointInfo[idxPart], attitude.pose[idxPart]);
			}
		}

		void TransformPointCloudInv(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::StJointInfo& joint, float_t f)
		{
			SHI::Point3D center(joint.cx, joint.cy, joint.cz);
			SHI::Point3D rotation(f * joint.rx, f * joint.ry, f * joint.rz);
			SHI::Point3D translation(f * joint.tx, f * joint.ty, f * joint.tz);
			TransformPointCloudInv(src, dst, center, rotation, translation);
		}

		double conv_ddmm2d(double ddmm)
		{
			double degree = floor(ddmm * 0.01);
			double min = ddmm - degree * 100;
			return degree + (min / 60.0);
		}

		void GetGlobalPositionTower(float_t& dx, float_t& dy, float_t& dz, float_t& rx, float_t& rz, const SHI::CraneAttitude& attitude)
		{
			// GPS ��ȯ; ����/�浵 ��ǥ(����dmm.mmmmmmmm) -> MGRS(UTM ������ ����)
			double gpsLatitude = conv_ddmm2d(attitude.gpsLatitude);
			double gpsLongitude = conv_ddmm2d(attitude.gpsLongitude);
			int zone = 0;
			char latitudeZone = 0;
			char squareZone1 = 0;
			char squareZone2 = 0;
			double_t east = 0;
			double_t north = 0;
			if (attitude.gpsLatitude2 > FLT_EPSILON) gpsLatitude = conv_ddmm2d(attitude.gpsLatitude2);
			if (attitude.gpsLongitude2 > FLT_EPSILON) gpsLongitude = conv_ddmm2d(attitude.gpsLongitude2);

			//zone = LatLonToUTMXY(gpsLatitude, gpsLongitude, 0, east, north);
			DevLib::Geometric::WGS84ToMGRS(gpsLatitude, gpsLongitude, &zone, &latitudeZone, &squareZone1, &squareZone2, &east, &north);

			GpsAttitude gpsAttitude = SHI::GetGpsParam(attitude.pierId, attitude.craneId);
			float_t angle = -(attitude.azimuth + gpsAttitude.gpsOffsetAzimuth);
			float_t c = cos(angle * Math::D2R);
			float_t s = sin(angle * Math::D2R);

			// Offset crane origin to crane
			int towerIdx = SHI::PierK::JIB_TOWER;
			if (attitude.craneId == SHI::PierK::TTC23 && attitude.pierId == SHI::PIERK)
			{
				towerIdx = SHI::PierK::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIERHAN)
			{
				towerIdx = SHI::PierHan::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIER6)
			{
				towerIdx = SHI::Pier6::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G2DOCK)
			{
				towerIdx = SHI::G2Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G3DOCK)
			{
				towerIdx = SHI::G3Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G4DOCK)
			{
				if (attitude.craneId == SHI::G4Dock::LLC25)
				{
					towerIdx = SHI::G4Dock::LLC_TOWER;
				}
			}
			//pjh
			if (attitude.pierId == SHI::PIERZ)
			{
				towerIdx = SHI::PierZ::LLC_TOWER;
			}//~pjh

			if (attitude.pierId == SHI::PIERY)
			{
				if (attitude.craneId == SHI::PierY::TC1)
				{
					towerIdx = SHI::PierY::TC_TOWER;
				}
				else
				{
					towerIdx = SHI::PierY::JIB_TOWER;
				}
			}

			float_t gps2towerX = -gpsAttitude.gpsOffsetX + attitude.jointInfo[towerIdx].cx;
			float_t gps2towerY = -gpsAttitude.gpsOffsetY + attitude.jointInfo[towerIdx].cy;
			float_t tower2gpsX = -gps2towerX;
			float_t tower2gpsY = -gps2towerY;

			float_t globalTower2GpsX = tower2gpsX * c - tower2gpsY * s;
			float_t globalTower2GpsY = tower2gpsX * s + tower2gpsY * c;
			float_t globalGps2TowerX = -globalTower2GpsX;
			float_t globalGps2TowerY = -globalTower2GpsY;

			east = east + globalGps2TowerX - gpsAttitude.pierOffsetX;
			north = north + globalGps2TowerY - gpsAttitude.pierOffsetY;

			dx = east;
			dy = north;
			dz = attitude.height + gpsAttitude.gpsOffsetZ;
			rz = -angle + gpsAttitude.craneOffsetAzimuth;
			rx = attitude.pose[SHI::Pier6::LLC_JIB];
		}

		void GetGlobalPositionCrane(float_t& dx, float_t& dy, float_t& dz, float_t& rx, float_t& rz, const SHI::CraneAttitude& attitude)
		{
			float_t _dx = 0, _dy = 0, _dz = 0, _rx = 0, _rz = 0;
			GetGlobalPositionTower(_dx, _dy, _dz, _rx, _rz, attitude);
			float_t c = cos(-_rz * Math::D2R);
			float_t s = sin(-_rz * Math::D2R);


			int towerIdx = SHI::PierK::JIB_TOWER;
			if (attitude.craneId == SHI::PierK::TTC23 && attitude.pierId == SHI::PIERK)
			{
				towerIdx = SHI::PierK::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIERHAN)
			{
				towerIdx = SHI::PierHan::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIER6)
			{
				towerIdx = SHI::Pier6::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G2DOCK)
			{
				towerIdx = SHI::G2Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G3DOCK)
			{
				towerIdx = SHI::G3Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G4DOCK)
			{
				towerIdx = SHI::G4Dock::LLC_TOWER;
			}
			//pjh
			if (attitude.pierId == SHI::PIERZ)
			{
				towerIdx = SHI::PierZ::LLC_TOWER;
			}//~pjh

			if (attitude.pierId == SHI::PIERY)
			{
				if (attitude.craneId == SHI::PierY::TC1)
				{
					towerIdx = SHI::PierY::TC_TOWER;
				}
				else
				{
					towerIdx = SHI::PierY::JIB_TOWER;
				}
			}


			float_t crane2towerX = attitude.jointInfo[towerIdx].cx;
			float_t crane2towerY = attitude.jointInfo[towerIdx].cy;
			float_t tower2craneX = -crane2towerX;
			float_t tower2craneY = -crane2towerY;

			float_t tower2crane_towerX = tower2craneX;// - attitude.jointInfo[towerIdx].cx;
			float_t tower2crane_towerY = tower2craneY;// - attitude.jointInfo[towerIdx].cy;
			float_t globalTower2craneX = tower2crane_towerX * c - tower2crane_towerY * s + _dx;
			float_t globalTower2craneY = tower2crane_towerX * s + tower2crane_towerY * c + _dy;

			dx = globalTower2craneX;
			dy = globalTower2craneY;
			dz = _dz;
			rx = _rx;
			rz = _rz;
		}

		void TransformPointCloudGps(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::CraneAttitude& attitude)
		{
			float_t dx = 0, dy = 0, dz = 0, rx = 0, rz = 0;
			GetGlobalPositionTower(dx, dy, dz, rx, rz, attitude);
			float_t c = cos(-rz * Math::D2R);
			float_t s = sin(-rz * Math::D2R);

			int towerIdx = SHI::PierK::JIB_TOWER;
			if (attitude.craneId == SHI::PierK::TTC23 && attitude.pierId == SHI::PIERK)
			{
				towerIdx = SHI::PierK::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIERHAN)
			{
				towerIdx = SHI::PierHan::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIER6)
			{
				towerIdx = SHI::Pier6::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G2DOCK)
			{
				towerIdx = SHI::G2Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G3DOCK)
			{
				towerIdx = SHI::G3Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G4DOCK)
			{
				towerIdx = SHI::G4Dock::LLC_TOWER;
			}
			//pjh
			if (attitude.pierId == SHI::PIERZ)
			{
				towerIdx = SHI::PierZ::LLC_TOWER;
			}//~pjh

			if (attitude.pierId == SHI::PIERY)
			{
				if (attitude.craneId == SHI::PierY::TC1)
				{
					towerIdx = SHI::PierY::TC_TOWER;
				}
				else
				{
					towerIdx = SHI::PierY::JIB_TOWER;
				}
			}

			float_t crane2towerX = attitude.jointInfo[towerIdx].cx;
			float_t crane2towerY = attitude.jointInfo[towerIdx].cy;

			SHI::PointCloud tmp = *src;
			for (unsigned int i = 0; i < src->size(); i++)
			{
				Eigen::Matrix<float_t, 3, 1> pt(src->points[i].x - crane2towerX, src->points[i].y - crane2towerY, src->points[i].z);
				tmp[i].x = static_cast<float_t> (c * pt.coeffRef(0) - s * pt.coeffRef(1) + dx);
				tmp[i].y = static_cast<float_t> (s * pt.coeffRef(0) + c * pt.coeffRef(1) + dy);
				tmp[i].z = static_cast<float_t> (pt.coeffRef(2) + dz);
			}
			dst = tmp;
		}

		void TransformPointCloudGpsInv(SHI::PointCloudPtr src, SHI::PointCloud& dst, const SHI::CraneAttitude& attitude)
		{
			float_t dx = 0, dy = 0, dz = 0, rx = 0, rz = 0;
			GetGlobalPositionTower(dx, dy, dz, rx, rz, attitude);
			float_t c = cos(rz * Math::D2R);
			float_t s = sin(rz * Math::D2R);

			int towerIdx = SHI::PierK::JIB_TOWER;
			if (attitude.craneId == SHI::PierK::TTC23 && attitude.pierId == SHI::PIERK)
			{
				towerIdx = SHI::PierK::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIERHAN)
			{
				towerIdx = SHI::PierHan::TTC_TOWER;
			}
			if (attitude.pierId == SHI::PIER6)
			{
				towerIdx = SHI::Pier6::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G2DOCK)
			{
				towerIdx = SHI::G2Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G3DOCK)
			{
				towerIdx = SHI::G3Dock::LLC_TOWER;
			}
			if (attitude.pierId == SHI::G4DOCK)
			{
				towerIdx = SHI::G4Dock::LLC_TOWER;
			}
			//pjh
			if (attitude.pierId == SHI::PIERZ)
			{
				towerIdx = SHI::PierZ::LLC_TOWER;
			}//~pjh

			if (attitude.pierId == SHI::PIERY)
			{
				if (attitude.craneId == SHI::PierY::TC1)
				{
					towerIdx = SHI::PierY::TC_TOWER;
				}
				else
				{
					towerIdx = SHI::PierY::JIB_TOWER;
				}
			}

			float_t crane2towerX = attitude.jointInfo[towerIdx].cx;
			float_t crane2towerY = attitude.jointInfo[towerIdx].cy;

			SHI::PointCloud tmp = *src;
			for (unsigned int i = 0; i < src->size(); i++)
			{
				Eigen::Matrix<float_t, 3, 1> pt(src->points[i].x - dx, src->points[i].y - dy, src->points[i].z - dz);
				tmp[i].x = static_cast<float_t> (c * pt.coeffRef(0) - s * pt.coeffRef(1) + crane2towerX);
				tmp[i].y = static_cast<float_t> (s * pt.coeffRef(0) + c * pt.coeffRef(1) + crane2towerY);
				tmp[i].z = static_cast<float_t> (pt.coeffRef(2));
			}
			dst = tmp;
		}

	}
}


