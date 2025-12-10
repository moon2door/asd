#include "CraneGps.h"
#include "InitialAttitude.h"
#include "CraneInfo.h"
#include "../Utility/Types.h"
#include "../3rd/Routine/include/Base/RoutineUtility.h"

namespace SHI
{
	GpsAttitude GetGpsParam(int32_t pier, int32_t crane)
	{
		GpsAttitude ret(0, 0, 0, 0, 0, 0, 0, 0, 0);
		CraneAttitude attitude;
		GetInitialAttitude(pier, crane, attitude);

		float_t offsetX = 63490;
		float_t offsetY = 62868;
		if (pier == SHI::PIERHAN)
		{
			offsetX += 10; // Adjusted total 63500
			offsetY -= 150; // Adjusted total 62718
		}
		else if (pier == SHI::PIER6)
		{
			offsetX += 0; // Keep 63490
			offsetY += 170; // Adjusted total 63038
		}
		else if (pier == SHI::PIERJ)
		{
			offsetX -= 2; // Adjusted total 63488
			offsetY += 170; // Adjusted total 63038
		}
		else if (pier == SHI::G2DOCK)
		{
			offsetX += 57; // Adjusted total 63547
			offsetY -= 10; // Adjusted total 62858
		}
		else if (pier == SHI::G3DOCK)
		{
			offsetX += -242;
			offsetY += -30;
		}
		else if (pier == SHI::G4DOCK)
		{
			offsetX = 63143;
			offsetY = 62718;
		}
		else if(pier != SHI::PIERK)
		{
			// TODO: load pier-specific offsets from GPS_Offset.json

			offsetX = 0;
			offsetY = 0;
		}

		switch (pier)
		{
			case SHI::PIER7:
				// No GPS available
				break;
			case SHI::PIERJ:
				if (crane == SHI::PierJ::LLC24)
				{
					ret = GpsAttitude(0, 0, -49, 0, 0, offsetX, offsetY, attitude.groundHeight, 270);
				}
				else if (crane == SHI::PierJ::LLC11)
				{
					// Result = 3.000000, -3.000000, 170.000000, error = 0.000000
					ret = GpsAttitude(3.000000, -3.000000, -32, 0, 170.000000, offsetX, offsetY, attitude.groundHeight, 100);
				}
				else if (crane == SHI::PierJ::LLC8)
				{
					// Result = 0.960000, -2.980000, 200.200073, error = 0.000038
					ret = GpsAttitude(0.960000, -2.980000, -57, 0, 200.200073, offsetX, offsetY, attitude.groundHeight, 70);
				}
				else if (crane == SHI::PierJ::LLC9)
				{
					// Result = -4.100000, -7.100000, 340.400085, error = 0.000297
					ret = GpsAttitude(-4.100000, -7.100000, -32, 0, 340.400085, offsetX, offsetY, attitude.groundHeight, -70);
				}
				break;
			case SHI::PIERK:
				if (crane == SHI::PierK::JIB1)
				{
					ret = GpsAttitude(6.799999, 1.600000, -11.3 - 4, 0, 205.000000, offsetX, offsetY, attitude.groundHeight, -116.2);
				}
				else if (crane == SHI::PierK::JIB2)
				{
					ret = GpsAttitude(3.3, 5.5, -15.3, 0, 56, offsetX, offsetY, attitude.groundHeight, 217.1);
				}
				else if (crane == SHI::PierK::JIB3)
				{
					ret = GpsAttitude(-1, -6.9, -15.3, 0, 252, offsetX, offsetY, attitude.groundHeight, 19.56);
				}
				else if (crane == SHI::PierK::TTC23)
				{
					ret = GpsAttitude(-3.9, -5.4, -5.76, 0, -10, offsetX, offsetY, attitude.groundHeight, 10.85);
				}
				else if (crane == SHI::PierK::LLC18)
				{
					ret = GpsAttitude(5.899999, -3.6, -48.62, 0, 334, offsetX, offsetY, attitude.groundHeight, 104.9);
				}
				else if (crane == SHI::PierK::LLC19)
				{
					ret = GpsAttitude(-4.6, -8.9, -48.62, 0, 59, offsetX, offsetY, attitude.groundHeight, 201.7);
				}
				break;
			case SHI::PIERHAN:
				if (crane == SHI::PierHan::GC1)
				{
					// Calibrated
					ret = GpsAttitude(-6.2, -1.4, -29.6 - 1.08, 0, 202.8, offsetX, offsetY, attitude.groundHeight, 250);
				}
				else if (crane == SHI::PierHan::GC2)
				{
					// Calibrated
					ret = GpsAttitude(6.8, 1.6, -24.6+0.5, 0, 205.0, offsetX, offsetY, attitude.groundHeight, 65);
				}
				else if (crane == SHI::PierHan::TC1)
				{
					// Calibrated
					// Result = 19.100004, 8.080004, 20.700006, error = 0.002423
					ret = GpsAttitude(19.1, 8.08, -5.31 - 4.4, 0, 20.7, offsetX - 13, offsetY - 12, attitude.groundHeight, 252);
				}
				else if (crane == SHI::PierHan::TC2)
				{
					// Calibrated
					// Result = 20.070004, 5.010003, 10.300005, error = 0.000116
					ret = GpsAttitude(20.07, 5.01, -4.9 - 2.98, 0, 10.3, offsetX - 13, offsetY - 12, attitude.groundHeight, 260);
				}
				else if (crane == SHI::PierHan::TC3)
				{
					// Calibrated
					ret = GpsAttitude(-13, -19, -6.12 - 3.1 + 2.65, 0, 59.899986, offsetX - 13, offsetY - 12, attitude.groundHeight, 34);
				}
				else if (crane == SHI::PierHan::TTC4)
				{
					// Pending calibration
					// Result = -20.100000, -19.100000, 219.900055
					ret = GpsAttitude(-20.1, -19.1, 5 - 14 - 1.9, 0, 219, offsetX - 10, offsetY - 12, attitude.groundHeight, 51 - 2.85);
				}
				else if (crane == SHI::PierHan::TC5)
				{
					// Pending calibration
					// Result = 6.900000, 16.920000, 60.099983, error = 0.047569
					ret = GpsAttitude(9.6, 18.4, -11.3 - 4 + 9.23 - 5.2 + 1.9, 0, 60.10, offsetX - 13, offsetY - 12, attitude.groundHeight, 210 + 1.77);
				}
				else if (crane == SHI::PierHan::TC6)
				{
					// Result = -18.009998, -9.039999, 29.700003, error = 0.001335
					// Result = -16.899996, -11.899996, 39.799988, error = 0.001900
					ret = GpsAttitude(-16, -10, -5 - 2.5 - 1.64, 0, 39.799988, offsetX - 14, offsetY - 11.7, attitude.groundHeight, 51 + 1.24);
				}
				break;
			case SHI::PIER6:
				if (crane == SHI::Pier6::LLC7)
				{
					float_t angleOffset = 260;
					float_t gpsOffsetX = -0.930000;
					float_t gpsOffsetY = -4.069999;
					float_t gpsOffsetZ = -33;
					float_t gpsOffsetAzimuth = 10.300005;
					// Result = -0.930000, -4.069999, 10.300005, error = 0.000000
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
				}
				else if (crane == SHI::Pier6::LLC23)
				{
					float_t angleOffset = 60;
					float_t gpsOffsetX = -16.100000;
					float_t gpsOffsetY = -12.100000;
					float_t gpsOffsetZ = -17;
					float_t gpsOffsetAzimuth = 210.200073;
					// Result = -16.100000, -12.100000, 210.200073, error = 0.000000
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
				}
				break;
			case SHI::G2DOCK:
				if (crane == SHI::G2Dock::LLC12)
				{
					double_t gpsOffsetX = 1.920000;
					double_t gpsOffsetY = -7.009998;
					double_t gpsOffsetZ = -23.5;
					double_t gpsOffsetAzimuth = 81;
					double_t angleOffset = -169;
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
					// Result = -1.910000, -7.009998, 30.000004, error = 0.000011
					// Result = 1.920000, -7.019998, 80.999969, error = 0.000173
					// Result = -2.000000, -7.000000, 30.000000, error = 0.000206
				}
				else if (crane == SHI::G2Dock::LLC13)
				{
					double_t gpsOffsetX = 0.900000;
					double_t gpsOffsetY = -0.1000;
					double_t gpsOffsetZ = -21;
					double_t gpsOffsetAzimuth = 50.499977;
					double_t angleOffset = 40;
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
					// Result = 0.900000, -0.100000, 50.499977, error = 0.000899
				}
				break;
			case SHI::G3DOCK:
				if (crane == SHI::G3Dock::LLC19)
				{

					double_t gpsOffsetX = 2.920000;
					double_t gpsOffsetY = -5.039999;
					double_t gpsOffsetZ = -30;
					double_t gpsOffsetAzimuth = 99.599991;
					double_t angleOffset = -190;

					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
					// Result = 1.000000, -4.999998, 50.599976, error = 0.000159
					// Result = 2.920000, -5.039999, 99.599991, error = 0.000084
				}
				else if (crane == SHI::G3Dock::LLC20)
				{

					double_t gpsOffsetX = -3.060000;
					double_t gpsOffsetY = -2.990000;
					double_t gpsOffsetZ = -30;
					double_t gpsOffsetAzimuth = -289.800049;
					double_t angleOffset = -160;
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
					// Result = -3.060000, -2.990000, 289.800049, error = 0.000068
				}
				break;
			case SHI::G4DOCK:
				if (crane == SHI::G4Dock::LLC25)
				{

					double_t gpsOffsetX = 1.9;
					double_t gpsOffsetY = -3;
					double_t gpsOffsetZ = -20;
					double_t gpsOffsetAzimuth = -79.599991;
					double_t angleOffset = -10.0;
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
				}
				else if (crane == SHI::G4Dock::LLC26)
				{
					double_t gpsOffsetX = -0.060000;
					double_t gpsOffsetY = -6.949996;
					double_t gpsOffsetZ = -20;
					double_t gpsOffsetAzimuth = 39.299995;
					float_t angleOffset = -130.0;
					ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
					
				}
				break;
			// Default offsets fallback
			default:
				// Example: per-crane config key (disabled)

				double_t gpsOffsetX = 0;// Routine::GetConfigInt(craneConfigKey, "gpsOffsetX", 1.9, "GPS_Offset.json");
				double_t gpsOffsetY = 0;//Routine::GetConfigInt(craneConfigKey, "gpsOffsetY", -3, "GPS_Offset.json");
				double_t gpsOffsetZ = 0;//Routine::GetConfigInt(craneConfigKey, "gpsOffsetZ", -20, "GPS_Offset.json");
				double_t gpsOffsetAzimuth = 0;//Routine::GetConfigInt(craneConfigKey, "gpsOffsetAzimuth", -79.599991, "GPS_Offset.json");
				double_t angleOffset = 0;//Routine::GetConfigInt(craneConfigKey, "angleOffset", -10, "GPS_Offset.json");

				ret = GpsAttitude(gpsOffsetX, gpsOffsetY, gpsOffsetZ, 0, gpsOffsetAzimuth, offsetX, offsetY, attitude.groundHeight, angleOffset);
				break;
			/* default case disabled (pjh 240509) */ 
		}

		return ret;
	}
}
