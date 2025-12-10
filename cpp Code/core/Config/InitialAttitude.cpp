#pragma once
#include "InitialAttitude.h"
#include "../Utility/Types.h"
#include "CraneInfo.h"

namespace SHI
{
	namespace Pier7
	{
		void GetInitialAttitudeGC(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 0;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -71.5, "ClusterProcessor.json");//pjh-71.5;
			attitude.numPart = 4;
			attitude.numHook = 3;
			attitude.hookWightThreshold[SHI::Pier7::GC_HOOK_XN] = 10;
			attitude.hookWightThreshold[SHI::Pier7::GC_HOOK_C] = 23;
			attitude.hookWightThreshold[SHI::Pier7::GC_HOOK_XP] = 9;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::GC_FIXED_LEG] = false;
			attitude.bUseEstimate[SHI::Pier7::GC_HINGED_LEG] = false;
			attitude.bUseEstimate[SHI::Pier7::GC_GIRDER] = false;
			attitude.bUseEstimate[SHI::Pier7::GC_ROOM] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::GC_FIXED_LEG] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::GC_HINGED_LEG] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::GC_GIRDER] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::GC_ROOM] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 1, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::GC_FIXED_LEG] = StROI(-26, 24, -9, 1.8, -71.5, 1);
			attitude.craneRoi[SHI::Pier7::GC_HINGED_LEG] = StROI(-26, 24, 132.5, 139, -71.5, 1);
			attitude.craneRoi[SHI::Pier7::GC_GIRDER] = StROI(-6.5, 6.5, -9, 139, -0.2, 10);
			attitude.craneRoi[SHI::Pier7::GC_ROOM] = StROI(-14.5, 17, -2.5, 15.5, -2.5, 18.0);
		}

		void GetInitialAttitudeLLC16(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 1;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -37, "ClusterProcessor.json");//pjh-37;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier7::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier7::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_JIB] = SHI::StJointInfo(0, -1.0, -1.25, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_TOWER] = SHI::StJointInfo(0.1217 + 0.1, -4.8561 + 0.1, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::LLC_BODY] = StROI(-5.5, 5.5, -14, 3.5, -2.5, 23.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier7::LLC_JIB] = StROI(-3.0, 3.5, -1, 64, -3.5, 1.4); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier7::LLC_TOWER] = StROI(-4.5, 4.7, -12.5, 1.4, -33.6, -2.2); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC17(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 2;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -37, "ClusterProcessor.json");//pjh-37;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier7::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier7::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_JIB] = SHI::StJointInfo(0, -1.0, -1.25, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_TOWER] = SHI::StJointInfo(0.1217, -4.8561, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::LLC_BODY] = StROI(-5.5, 5.5, -14, 3.5, -2.5, 23.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier7::LLC_JIB] = StROI(-3.0, 3.5, -1, 64, -3.5, 1.4); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier7::LLC_TOWER] = StROI(-4.5, 4.7, -12.5, 1.4, -33.6, -2.2); // LLC 타워 ROI
		}

		void GetInitialAttitudeTC1(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 3;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -52.4, "ClusterProcessor.json");//pjh-52.4;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::TC_JIB] = StROI(-2.3, 3.9, -24, 62, -2, 15); // TC 7-1 지브 ROI
		}

		void GetInitialAttitudeTC2(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 4;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -42.4, "ClusterProcessor.json");//pjh;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::TC_JIB] = StROI(-2.3, 3.8, -21, 62, -0.5, 7.3); // TC 7-2 지브 ROI
		}

		void GetInitialAttitudeTC4(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 5;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -51.5, "ClusterProcessor.json");//pjh-51.5;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::TC_JIB] = StROI(-2.3, 3.6, -23.2, 62.1, 0.5, 15); // TC 7-4 지브 ROI
		}

		void GetInitialAttitudeTC5(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 6;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -53.8, "ClusterProcessor.json");//pjh-53.8;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::TC_JIB] = StROI(-2.4, 2.6, -23.6, 61.3, -0.4, 13.2); // TC 7-5 지브 ROI
		}

		void GetInitialAttitudeTC6(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER7;
			attitude.craneId = 7;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -52.4, "ClusterProcessor.json");//pjh-52.4;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::TC_JIB] = StROI(-2.0, 4.1, -23.3, 66.7, -1.2, 14.5); // TC 7-6 지브 ROI

		}

	}

	namespace PierJ
	{
		void GetInitialAttitudeLLC24(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERJ;
			attitude.craneId = SHI::PierJ::LLC24;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -67.0, "ClusterProcessor.json");//pjh-67.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier7::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier7::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_TOWER] = SHI::StJointInfo(0, -5.9, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::LLC_BODY] = StROI(-7.0, 7.0, -26.0, 5.0, -5.5, 54.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier7::LLC_JIB] = StROI(-4.0, 5.0, -1, 135, -6.0, 3.0); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier7::LLC_TOWER] = StROI(-9.5, 9.5, -34, 24, -67.0, -2.0); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC11(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERJ;
			attitude.craneId = SHI::PierJ::LLC11;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -41.8, "ClusterProcessor.json");//pjh-41.8;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier7::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier7::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_TOWER] = SHI::StJointInfo(0, -4.82, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::LLC_BODY] = StROI(-7.0, 5.5, -24.0, 2.0, -4.5, 54.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier7::LLC_JIB] = StROI(-4.0, 4.0, -1, 117, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier7::LLC_TOWER] = StROI(-9.0, 10.0, -26.0, 13.5, -66.0, -3.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC8(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERJ;
			attitude.craneId = SHI::PierJ::LLC8;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -65.2, "ClusterProcessor.json");//pjh-65.2;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier7::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier7::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_TOWER] = SHI::StJointInfo(0, -5.90, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::LLC_BODY] = StROI(-7.0, 5.5, -24.0, 2.0, -4.5, 54.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier7::LLC_JIB] = StROI(-4.0, 4.0, -1, 117, -3.0, 5.0); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier7::LLC_TOWER] = StROI(-9.0, 10.0, -26.0, 13.5, -66.0, -3.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC9(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERJ;
			attitude.craneId = SHI::PierJ::LLC9;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -40.5, "ClusterProcessor.json");//pjh-40.5;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier7::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier7::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier7::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier7::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier7::LLC_TOWER] = SHI::StJointInfo(0, -2.47, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier7::LLC_BODY] = StROI(-7.0, 7.0, -24.0, 2.0, -4.5, 54.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier7::LLC_JIB] = StROI(-4.0, 4.0, -1, 117, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier7::LLC_TOWER] = StROI(-9.0, 10.0, -26.0, 13.5, -66.0, -3.5); // LLC 타워 ROI
		}
	}

	namespace PierK
	{
		void GetInitialAttitudeJib1(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERK;
			attitude.craneId = SHI::PierK::JIB1;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");//pjh-59.2;
			attitude.numPart = 3;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierK::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierK::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierK::JIB_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierK::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierK::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::PierK::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierK::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0); // LLC 타워 ROI
		}

		void GetInitialAttitudeJib2(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERK;
			attitude.craneId = SHI::PierK::JIB2;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");//pjh-59.2;
			attitude.numPart = 3;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierK::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierK::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierK::JIB_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierK::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierK::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::PierK::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierK::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0); // LLC 타워 ROI
		}

		void GetInitialAttitudeJib3(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERK;
			attitude.craneId = SHI::PierK::JIB3;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");//pjh-59.2;
			attitude.numPart = 3;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierK::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierK::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierK::JIB_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierK::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierK::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::PierK::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierK::JIB_TOWER] = StROI(-5.0, 5.0, -5.0, 5.0, -33.0, 0.0); // LLC 타워 ROI
		}

		void GetInitialAttitudeTTC23(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERK;
			attitude.craneId = SHI::PierK::TTC23;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -75.5, "ClusterProcessor.json");//pjh-75.5;
			attitude.numPart = 2;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierK::TTC_JIB] = false;
			attitude.bUseEstimate[SHI::PierK::TTC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierK::TTC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::TTC_TOWER] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierK::TTC_JIB] = StROI(-5.0, 7.0, -27.0, 85, -3.5, 14.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierK::TTC_TOWER] = StROI(-9.0, 9.0, -9.0, 9.0, -76.0, 0.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC18(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERK;
			attitude.craneId = SHI::PierK::LLC18;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -62.5, "ClusterProcessor.json");//pjh-62.5;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierK::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::PierK::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::PierK::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierK::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::LLC_TOWER] = SHI::StJointInfo(0, -5.47, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierK::LLC_BODY] = StROI(-7.0, 7.0, -24.0, 6.0, -3.5, 47.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::PierK::LLC_JIB] = StROI(-4.3, 4.3, 0, 120, -5.8, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierK::LLC_TOWER] = StROI(-10.0, 10.0, -26.0, 14.5, -66.0, -1.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC19(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERK;
			attitude.craneId = SHI::PierK::LLC19;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -62.5, "ClusterProcessor.json");//pjh-62.5;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierK::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::PierK::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::PierK::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierK::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierK::LLC_TOWER] = SHI::StJointInfo(0, -5.47, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierK::LLC_BODY] = StROI(-7.0, 7.0, -24.0, 6.0, -3.5, 47.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::PierK::LLC_JIB] = StROI(-4.3, 4.3, 0, 120, -5.8, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierK::LLC_TOWER] = StROI(-10.0, 10.0, -26.0, 14.5, -66.0, -1.5); // LLC 타워 ROI
		}
	}

	namespace PierHan
	{
		void GetInitialAttitudeGC1(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::GC1;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -85.0, "ClusterProcessor.json");//pjh-85.0;
			attitude.numPart = 4;
			attitude.numHook = 3;
			attitude.hookWightThreshold[SHI::PierHan::GC_HOOK_XN] = 3;
			attitude.hookWightThreshold[SHI::PierHan::GC_HOOK_C] = 3;
			attitude.hookWightThreshold[SHI::PierHan::GC_HOOK_XP] = 3;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::GC_FIXED_LEG] = false;
			attitude.bUseEstimate[SHI::PierHan::GC_HINGED_LEG] = false;
			attitude.bUseEstimate[SHI::PierHan::GC_GIRDER] = false;
			attitude.bUseEstimate[SHI::PierHan::GC_ROOM] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::GC_FIXED_LEG] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::GC_HINGED_LEG] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::GC_GIRDER] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::GC_ROOM] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 1, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::GC_FIXED_LEG] = StROI(-31, 31, -15, 5, -90.0, 10);
			attitude.craneRoi[SHI::PierHan::GC_HINGED_LEG] = StROI(-31, 31, 155, 170, -90.0, 10);
			attitude.craneRoi[SHI::PierHan::GC_GIRDER] = StROI(-6, 6, -15, 170, -1, 15);
			attitude.craneRoi[SHI::PierHan::GC_ROOM] = StROI(-19, 12, -7, 12, -12, 25);
		}

		void GetInitialAttitudeGC2(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::GC2;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -69.0, "ClusterProcessor.json");//pjh-69.0;
			attitude.numPart = 4;
			attitude.numHook = 3;
			attitude.hookWightThreshold[SHI::PierHan::GC_HOOK_XN] = 3;
			attitude.hookWightThreshold[SHI::PierHan::GC_HOOK_C] = 3;
			attitude.hookWightThreshold[SHI::PierHan::GC_HOOK_XP] = 3;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::GC_FIXED_LEG] = false;
			attitude.bUseEstimate[SHI::PierHan::GC_HINGED_LEG] = false;
			attitude.bUseEstimate[SHI::PierHan::GC_GIRDER] = false;
			attitude.bUseEstimate[SHI::PierHan::GC_ROOM] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::GC_FIXED_LEG] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::GC_HINGED_LEG] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::GC_GIRDER] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::GC_ROOM] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 1, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::GC_FIXED_LEG] = StROI(-20, 26, -10, 7, -69.0, 0);
			attitude.craneRoi[SHI::PierHan::GC_HINGED_LEG] = StROI(-20, 26, 160, 168, -69.0, 0);
			attitude.craneRoi[SHI::PierHan::GC_GIRDER] = StROI(-10, 5, -20, 166, -3, 21);
			attitude.craneRoi[SHI::PierHan::GC_ROOM] = StROI(-20, 10, -7, 12, -1, 16);
		}

		void GetInitialAttitudeTTC4(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::TTC4;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -60.0, "ClusterProcessor.json");//pjh-60.0;
			attitude.numPart = 2;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::TTC_JIB] = false;
			attitude.bUseEstimate[SHI::PierHan::TTC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::TTC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierHan::TTC_TOWER] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::TTC_JIB] = StROI(-3.0, 5.0, -30.0, 80, -1.0, 17.0); // LLC Jib ROI
			attitude.craneRoi[SHI::PierHan::TTC_TOWER] = StROI(-4.0, 4.0, -4.0, 4.0, -50.0, 1.0); // LLC 타워 ROI
		}

		void GetInitialAttitudeTC1(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::TC1;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -54.0, "ClusterProcessor.json");//pjh-54;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::TC_TOWER] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::TC_JIB] = StROI(-2.3, 3.9, -24, 72, -2, 15);
		}

		void GetInitialAttitudeTC2(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::TC2;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::TC_JIB] = StROI(-2.3, 3.0, -24, 62, -2, 12);
		}

		void GetInitialAttitudeTC3(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::TC3;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -45.0, "ClusterProcessor.json");//pjh-45.0;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::TC_TOWER] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::TC_JIB] = StROI(-2.0, 4.0, -24, 72, -2.5, 13);
		}

		void GetInitialAttitudeTC5(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::TC5;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -43.0, "ClusterProcessor.json");//pjh-43.0;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::TC_JIB] = StROI(-4.6, 3.3, -24, 75, -0.2, 13);
		}

		void GetInitialAttitudeTC6(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERHAN;
			attitude.craneId = SHI::PierHan::TC6;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -46.0, "ClusterProcessor.json");//pjh-46.0;
			attitude.numPart = 1;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierHan::TC_JIB] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierHan::TC_JIB] = SHI::StJointInfo(0, 0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierHan::TC_JIB] = StROI(-2.3, 3.9, -24, 72, -2, 15);
		}
	}

	namespace Pier6
	{
		void GetInitialAttitudeLLC7(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER6;
			attitude.craneId = SHI::Pier6::LLC7;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -29.0, "ClusterProcessor.json");//pjh-29.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier6::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier6::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier6::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier6::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier6::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier6::LLC_TOWER] = SHI::StJointInfo(0, -2.0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier6::LLC_BODY] = StROI(-7.0, 7.0, -24.0, 6.0, -3.5, 47.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier6::LLC_JIB] = StROI(-4.3, 4.3, 0, 120, -5.8, 6.0); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier6::LLC_TOWER] = StROI(-10.0, 10.0, -26.0, 14.5, -66.0, -1.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC23(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIER6;
			attitude.craneId = SHI::Pier6::LLC23;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -62.5, "ClusterProcessor.json");//pjh-62.5;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::Pier6::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::Pier6::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::Pier6::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::Pier6::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier6::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::Pier6::LLC_TOWER] = SHI::StJointInfo(0, -5.47, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::Pier6::LLC_BODY] = StROI(-7.0, 7.0, -24.0, 6.0, -3.5, 47.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::Pier6::LLC_JIB] = StROI(-4.3, 4.3, 0, 120, -5.8, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::Pier6::LLC_TOWER] = StROI(-10.0, 10.0, -26.0, 14.5, -66.0, -1.5); // LLC 타워 ROI
		}
	}

	namespace G2Dock
	{
		void GetInitialAttitudeLLC12(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::G2DOCK;
			attitude.craneId = SHI::G2Dock::LLC12;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::G2Dock::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::G2Dock::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::G2Dock::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::G2Dock::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G2Dock::LLC_JIB] = SHI::StJointInfo(0, 0.0, 0.0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G2Dock::LLC_TOWER] = SHI::StJointInfo(0, -4.0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::G2Dock::LLC_BODY] = StROI(-5.0, 5.0, -14.0, 2.5, -2.0, 16.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::G2Dock::LLC_JIB] = StROI(-4.0, 4.0, 0, 53.0, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::G2Dock::LLC_TOWER] = StROI(-4.5, 4.5, -13.0, 5.0, -30.0, -1.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC13(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::G2DOCK;
			attitude.craneId = SHI::G2Dock::LLC13;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::G2Dock::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::G2Dock::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::G2Dock::LLC_TOWER] = true;

			// 조인트 정보
			attitude.jointInfo[SHI::G2Dock::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G2Dock::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G2Dock::LLC_TOWER] = SHI::StJointInfo(0, -3.8, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::G2Dock::LLC_BODY] = StROI(-5.0, 5.0, -14.0, 2.5, -2.0, 16.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::G2Dock::LLC_JIB] = StROI(-4.0, 4.0, 0, 53.0, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::G2Dock::LLC_TOWER] = StROI(-4.5, 4.5, -13.0, 5.0, -30.0, -1.5); // LLC 타워 ROI
		}
	}

	namespace G3Dock
	{
		void GetInitialAttitudeLLC19(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::G3DOCK;
			attitude.craneId = SHI::G3Dock::LLC19;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::G3Dock::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::G3Dock::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::G3Dock::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::G3Dock::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G3Dock::LLC_JIB] = SHI::StJointInfo(0, 0.0, 0.0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G3Dock::LLC_TOWER] = SHI::StJointInfo(0, -3.82, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::G3Dock::LLC_BODY] = StROI(-6.0, 6.0, -18.0, 5.0, -2.0, 23.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::G3Dock::LLC_JIB] = StROI(-4.0, 4.0, 0, 58.0, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::G3Dock::LLC_TOWER] = StROI(-4.5, 4.5, -13.0, 5.0, -30.0, -1.4); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC20(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::G3DOCK;
			attitude.craneId = SHI::G3Dock::LLC20;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::G3Dock::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::G3Dock::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::G3Dock::LLC_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::G3Dock::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G3Dock::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G3Dock::LLC_TOWER] = SHI::StJointInfo(0, -3.75, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::G3Dock::LLC_BODY] = StROI(-6.0, 6.0, -18.0, 5.0, -2.0, 23.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::G3Dock::LLC_JIB] = StROI(-4.0, 4.0, 0, 58.0, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::G3Dock::LLC_TOWER] = StROI(-4.5, 4.5, -13.0, 5.0, -30.0, -1.4); // LLC 타워 ROI
		}
	}

	namespace G4Dock
	{
		void GetInitialAttitudeLLC25(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::G4DOCK;
			attitude.craneId = SHI::G4Dock::LLC25;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::G4Dock::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::G4Dock::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::G4Dock::LLC_TOWER] = false;

			attitude.jointInfo[SHI::G4Dock::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G4Dock::LLC_JIB] = SHI::StJointInfo(0, -1.2, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G4Dock::LLC_TOWER] = SHI::StJointInfo(0, -5.0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::G4Dock::LLC_BODY] = StROI(-5.5, 5.5, -14.0, 2.5, -2.0, 200.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::G4Dock::LLC_JIB] = StROI(-4.0, 4.0, 0, 53.0, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::G4Dock::LLC_TOWER] = StROI(-6.0, 6.0, -22.0, 8.0, -40.0, -1.5); // LLC 타워 ROI
		}

		void GetInitialAttitudeLLC26(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::G4DOCK;
			attitude.craneId = SHI::G4Dock::LLC26;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -50.0, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::G4Dock::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::G4Dock::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::G4Dock::LLC_TOWER] = false;

			//// 조인트 정보
			attitude.jointInfo[SHI::G4Dock::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G4Dock::LLC_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::G4Dock::LLC_TOWER] = SHI::StJointInfo(0, -3.0, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::G4Dock::LLC_BODY] = StROI(-5.5, 5.5, -14.0, 2.5, -2.0, 20.0); // LLC 몸체 ROI
			attitude.craneRoi[SHI::G4Dock::LLC_JIB] = StROI(-4.0, 4.0, 0, 53.0, -4.0, 4.0); // LLC Jib ROI
			attitude.craneRoi[SHI::G4Dock::LLC_TOWER] = StROI(-6.0, 6.0, -22.0, 8.0, -40.0, -1.5); // LLC 타워 ROI
		}
	}

	//pjh
	namespace PierZ
	{
		void GetInitialAttitudeLLC16(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(CraneAttitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERZ;
			attitude.craneId = SHI::PierZ::LLC16;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -55, "ClusterProcessor.json");//pjh-50.0;
			attitude.numPart = 3;
			attitude.numHook = 2;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierZ::LLC_BODY] = false;
			attitude.bUseEstimate[SHI::PierZ::LLC_JIB] = true;
			attitude.bUseEstimate[SHI::PierZ::LLC_TOWER] = false;

			//pjh
			auto roiSt = Routine::GetConfigString("ROI", "jib_JointInfo", "0, -1.2, 0, 1, 0, 0, 0, 0, 0", "ClusterProcessor.json");
			std::istringstream iss(roiSt);
			std::vector<std::float_t> tokens;
			std::string token;
			while (iss >> token) {
				tokens.push_back(std::stof(token));
			}
			
			attitude.jointInfo[SHI::PierZ::LLC_JIB] = SHI::StJointInfo(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7], tokens[8]);

			tokens.clear();
			roiSt = Routine::GetConfigString("ROI", "body_JointInfo", "0, 0, 0, 0, 0, 0, 0, 0, 0", "ClusterProcessor.json");
			iss.clear();
			iss.str(roiSt);
			while (iss >> token) {
				tokens.push_back(std::stof(token));
			}
			attitude.jointInfo[SHI::PierZ::LLC_BODY] = SHI::StJointInfo(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7], tokens[8]);
			
			tokens.clear();
			roiSt = Routine::GetConfigString("ROI", "tower_JointInfo", "0, -5.0, 0, 0, 0, 1, 0, 0, 0", "ClusterProcessor.json");
			iss.clear();
			iss.str(roiSt);
			while (iss >> token) {
				tokens.push_back(std::stof(token));
			}
			attitude.jointInfo[SHI::PierZ::LLC_TOWER] = SHI::StJointInfo(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7], tokens[8]);

			// 크레인 ROI
			tokens.clear();
			roiSt = Routine::GetConfigString("ROI", "jib_craneROI", "-4.0, 4.0, 0, 53.0, -4.0, 4.0", "ClusterProcessor.json");
			iss.clear();
			iss.str(roiSt);
			while (iss >> token) {
				tokens.push_back(std::stof(token));
			}
			attitude.craneRoi[SHI::PierZ::LLC_JIB] = StROI(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5]);

			tokens.clear();
			roiSt = Routine::GetConfigString("ROI", "body_craneROI", "-5.5, 5.5, -14.0, 2.5, -2.0, 200.0", "ClusterProcessor.json");
			iss.clear();
			iss.str(roiSt);
			while (iss >> token) {
				tokens.push_back(std::stof(token));
			}
			attitude.craneRoi[SHI::PierZ::LLC_BODY] = StROI(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5]);
			
			tokens.clear();
			roiSt = Routine::GetConfigString("ROI", "tower_craneROI", "-6.0, 6.0, -22.0, 8.0, -40.0, -1.5", "ClusterProcessor.json");
			iss.clear();
			iss.str(roiSt);
			while (iss >> token) {
				tokens.push_back(std::stof(token));
			}
			attitude.craneRoi[SHI::PierZ::LLC_TOWER] = StROI(tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5]);
			
			//attitude.jointInfo[SHI::PierZ::LLC_JIB] = SHI::StJointInfo(0, -1.2, 0, 1, 0, 0, 0, 0, 0);
			//attitude.jointInfo[SHI::PierZ::LLC_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			//attitude.jointInfo[SHI::PierZ::LLC_TOWER] = SHI::StJointInfo(0, -5.0, 0, 0, 0, 1, 0, 0, 0);
			//attitude.craneRoi[SHI::PierZ::LLC_JIB] = StROI(-4.0, 4.0, 0, 53.0, -4.0, 4.0); // LLC Jib ROI
			//attitude.craneRoi[SHI::PierZ::LLC_BODY] = StROI(-5.5, 5.5, -14.0, 2.5, -2.0, 200.0); // LLC 몸체 ROI
			//attitude.craneRoi[SHI::PierZ::LLC_TOWER] = StROI(-6.0, 6.0, -22.0, 8.0, -40.0, -1.5); // LLC 타워 ROI
			//
		}
	}
	//
	namespace PierY
	{
		/* ----------  Y‑TC1  ---------- */
		void GetInitialAttitudeTC1(CraneAttitude& a)
		{
			memset(&a, 0, sizeof(a));
			a.pierId = SHI::PIERY;
			a.craneId = SHI::PierY::TC1;
			a.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -52.0, "ClusterProcessor.json");
			a.numPart = 1;
			a.numHook = 1;
			a.bUseEstimate[SHI::PierY::TC_JIB] = false;
			a.jointInfo[SHI::PierY::TC_JIB] = StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			a.craneRoi[SHI::PierY::TC_JIB] = StROI(-2.5, 3.0, -22, 65, -2, 15);
		}

		/* ----------  Y‑JIB1  ---------- */
		void GetInitialAttitudeJIB1(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(attitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERY;
			attitude.craneId = SHI::PierY::JIB1;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");
			attitude.numPart = 3;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierY::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierY::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierY::JIB_TOWER] = false;
			
			// 조인트 정보
			attitude.jointInfo[SHI::PierY::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierY::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0);		 // 몸체 ROI
			attitude.craneRoi[SHI::PierY::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0);				 // Jib ROI
			attitude.craneRoi[SHI::PierY::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0);		 // 타워 ROI
		}

		/* ----------  Y‑JIB2  ---------- */
		void GetInitialAttitudeJIB2(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(attitude));

			// 크레인 정보
			attitude.pierId = SHI::PIERY;
			attitude.craneId = SHI::PierY::JIB2;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");
			attitude.numPart = 3;
			attitude.numHook = 1;

			// 자세 인식 여부
			attitude.bUseEstimate[SHI::PierY::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierY::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierY::JIB_TOWER] = false;

			// 조인트 정보
			attitude.jointInfo[SHI::PierY::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			// 크레인 ROI
			attitude.craneRoi[SHI::PierY::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0);	// 몸체 ROI
			attitude.craneRoi[SHI::PierY::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0);			// Jib ROI
			attitude.craneRoi[SHI::PierY::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0);	// 타워 ROI
		}

		/* ----------  Y?JIB3  ---------- */
		void GetInitialAttitudeJIB3(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(attitude));

			attitude.pierId = SHI::PIERY;
			attitude.craneId = SHI::PierY::JIB3;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");
			attitude.numPart = 3;
			attitude.numHook = 1;

			attitude.bUseEstimate[SHI::PierY::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierY::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierY::JIB_TOWER] = false;

			attitude.jointInfo[SHI::PierY::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			attitude.craneRoi[SHI::PierY::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0);
			attitude.craneRoi[SHI::PierY::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0);
			attitude.craneRoi[SHI::PierY::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0);
		}

		/* ----------  Y?JIB4  ---------- */
		void GetInitialAttitudeJIB4(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(attitude));

			attitude.pierId = SHI::PIERY;
			attitude.craneId = SHI::PierY::JIB4;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");
			attitude.numPart = 3;
			attitude.numHook = 1;

			attitude.bUseEstimate[SHI::PierY::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierY::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierY::JIB_TOWER] = false;

			attitude.jointInfo[SHI::PierY::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			attitude.craneRoi[SHI::PierY::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0);
			attitude.craneRoi[SHI::PierY::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0);
			attitude.craneRoi[SHI::PierY::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0);
		}

		/* ----------  Y-JIB5  ---------- */
		void GetInitialAttitudeJIB5(CraneAttitude& attitude)
		{
			memset(&attitude, 0, sizeof(attitude));

			attitude.pierId = SHI::PIERY;
			attitude.craneId = SHI::PierY::JIB5;
			attitude.groundHeight = Routine::GetConfigDouble("CraneData", "GroundHeight", -59.2, "ClusterProcessor.json");
			attitude.numPart = 3;
			attitude.numHook = 1;

			attitude.bUseEstimate[SHI::PierY::JIB_BODY] = false;
			attitude.bUseEstimate[SHI::PierY::JIB_JIB] = true;
			attitude.bUseEstimate[SHI::PierY::JIB_TOWER] = false;

			attitude.jointInfo[SHI::PierY::JIB_BODY] = SHI::StJointInfo(0, 0, 0, 0, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_JIB] = SHI::StJointInfo(0, 0, 0, 1, 0, 0, 0, 0, 0);
			attitude.jointInfo[SHI::PierY::JIB_TOWER] = SHI::StJointInfo(0, -0.38, 0, 0, 0, 1, 0, 0, 0);

			attitude.craneRoi[SHI::PierY::JIB_BODY] = StROI(-3.5, 4.0, -8.5, 2.5, -3.5, 15.0);
			attitude.craneRoi[SHI::PierY::JIB_JIB] = StROI(-1.5, 1.5, -1, 56, -1.0, 3.0);
			attitude.craneRoi[SHI::PierY::JIB_TOWER] = StROI(-2.0, 2.0, -2.0, 2.0, -60.0, 0.0);
		}
	}

	bool GetInitialAttitude(int32_t pier, int32_t id, CraneAttitude& attitude)
	{
		bool ret = false;
		if (pier == SHI::PIER7)
		{
			switch (id)
			{
			case SHI::Pier7::GC:
				SHI::Pier7::GetInitialAttitudeGC(attitude);
				ret = true;
				break;
			case SHI::Pier7::LLC16:
				SHI::Pier7::GetInitialAttitudeLLC16(attitude);
				ret = true;
				break;
			case SHI::Pier7::LLC17:
				SHI::Pier7::GetInitialAttitudeLLC17(attitude);
				ret = true;
				break;
			case SHI::Pier7::TC1:
				SHI::Pier7::GetInitialAttitudeTC1(attitude);
				ret = true;
				break;
			case SHI::Pier7::TC2:
				SHI::Pier7::GetInitialAttitudeTC2(attitude);
				ret = true;
				break;
			case SHI::Pier7::TC4:
				SHI::Pier7::GetInitialAttitudeTC4(attitude);
				ret = true;
				break;
			case SHI::Pier7::TC5:
				SHI::Pier7::GetInitialAttitudeTC5(attitude);
				ret = true;
				break;
			case SHI::Pier7::TC6:
				SHI::Pier7::GetInitialAttitudeTC6(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::PIERJ)
		{
			switch (id)
			{
			case SHI::PierJ::LLC24:
				SHI::PierJ::GetInitialAttitudeLLC24(attitude);
				ret = true;
				break;
			case SHI::PierJ::LLC11:
				SHI::PierJ::GetInitialAttitudeLLC11(attitude);
				ret = true;
				break;
			case SHI::PierJ::LLC8:
				SHI::PierJ::GetInitialAttitudeLLC8(attitude);
				ret = true;
				break;
			case SHI::PierJ::LLC9:
				SHI::PierJ::GetInitialAttitudeLLC9(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::PIERK)
		{
			switch (id)
			{
			case SHI::PierK::JIB1:
				SHI::PierK::GetInitialAttitudeJib1(attitude);
				ret = true;
				break;
			case SHI::PierK::JIB2:
				SHI::PierK::GetInitialAttitudeJib2(attitude);
				ret = true;
				break;
			case SHI::PierK::JIB3:
				SHI::PierK::GetInitialAttitudeJib3(attitude);
				ret = true;
				break;
			case SHI::PierK::TTC23:
				SHI::PierK::GetInitialAttitudeTTC23(attitude);
				ret = true;
				break;
			case SHI::PierK::LLC18:
				SHI::PierK::GetInitialAttitudeLLC18(attitude);
				ret = true;
				break;
			case SHI::PierK::LLC19:
				SHI::PierK::GetInitialAttitudeLLC19(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::PIERHAN)
		{
			switch (id)
			{
			case SHI::PierHan::GC1:
				SHI::PierHan::GetInitialAttitudeGC1(attitude);
				ret = true;
				break;
			case SHI::PierHan::GC2:
				SHI::PierHan::GetInitialAttitudeGC2(attitude);
				ret = true;
				break;
			case SHI::PierHan::TTC4:
				SHI::PierHan::GetInitialAttitudeTTC4(attitude);
				ret = true;
				break;
			case SHI::PierHan::TC1:
				SHI::PierHan::GetInitialAttitudeTC1(attitude);
				ret = true;
				break;
			case SHI::PierHan::TC2:
				SHI::PierHan::GetInitialAttitudeTC2(attitude);
				ret = true;
				break;
			case SHI::PierHan::TC3:
				SHI::PierHan::GetInitialAttitudeTC3(attitude);
				ret = true;
				break;
			case SHI::PierHan::TC5:
				SHI::PierHan::GetInitialAttitudeTC5(attitude);
				ret = true;
				break;
			case SHI::PierHan::TC6:
				SHI::PierHan::GetInitialAttitudeTC6(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::PIER6)
		{
			switch (id)
			{
			case SHI::Pier6::LLC7:
				SHI::Pier6::GetInitialAttitudeLLC7(attitude);
				ret = true;
				break;
			case SHI::Pier6::LLC23:
				SHI::Pier6::GetInitialAttitudeLLC23(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::G2DOCK)
		{
			switch (id)
			{
			case SHI::G2Dock::LLC12:
				SHI::G2Dock::GetInitialAttitudeLLC12(attitude);
				ret = true;
				break;
			case SHI::G2Dock::LLC13:
				SHI::G2Dock::GetInitialAttitudeLLC13(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::G3DOCK)
		{
			switch (id)
			{
			case SHI::G3Dock::LLC19:
				SHI::G3Dock::GetInitialAttitudeLLC19(attitude);
				ret = true;
				break;
			case SHI::G3Dock::LLC20:
				SHI::G3Dock::GetInitialAttitudeLLC20(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		else if (pier == SHI::G4DOCK)
		{
			switch (id)
			{
			case SHI::G4Dock::LLC25:
				SHI::G4Dock::GetInitialAttitudeLLC25(attitude);
				ret = true;
				break;
			case SHI::G4Dock::LLC26:
				SHI::G4Dock::GetInitialAttitudeLLC26(attitude);
				ret = true;
				break;
			default:
				break;
			}
		}
		//pjh
		else if (pier == SHI::PIERZ)
		{
			switch (id)
			{
				case SHI::PierZ::LLC16:
					SHI::PierZ::GetInitialAttitudeLLC16(attitude);
					ret = true;
					break;
			}
		}
		//
		else if (pier == SHI::PIERY)
		{
			switch (id)
			{
				case SHI::PierY::TC1:  
					SHI::PierY::GetInitialAttitudeTC1(attitude);
					ret = true; break;
				case SHI::PierY::JIB1: 
					SHI::PierY::GetInitialAttitudeJIB1(attitude);
					ret = true; break;
				case SHI::PierY::JIB2: 
					SHI::PierY::GetInitialAttitudeJIB2(attitude);
					ret = true; break;
				case SHI::PierY::JIB3: 
					SHI::PierY::GetInitialAttitudeJIB3(attitude);
					ret = true; break;
				case SHI::PierY::JIB4: 
					SHI::PierY::GetInitialAttitudeJIB4(attitude);
					ret = true; break;
				case SHI::PierY::JIB5: 
					SHI::PierY::GetInitialAttitudeJIB5(attitude);
					ret = true; break;
				
				default: 
					break;
			}
		}
		return ret;
	}
}
