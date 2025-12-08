#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"

#include <cmath>

namespace
{
	float g_lastValidAzimuth = 0.0f;
	uint32_t g_invalidHeadingCount = 0;
}

bool IsValidGps(SHI::Device::TDR2000::StGpsData& gps)
{
	bool ret = false;
	if (gps.time > 0 &&
		gps.latitude > FLT_EPSILON &&
		gps.longitude > FLT_EPSILON //&&
		//gps.quality >= 4 && pjh
		//abs(gps.azimuth) > FLT_EPSILON pjh
		)
	{
		ret = true;
	}
#ifdef _DEBUG
	//printf("IsValidGps : %s\n", ret ? "True" : "False");
#endif
	return ret;
}

void CCollisionProcessorManagerDlg::OnGpsData(SHI::Device::TDR2000::StGpsData& gpsData)
{
	bool bHeadingValid = std::isfinite(gpsData.azimuth);
	if (bHeadingValid)
	{
		g_lastValidAzimuth = gpsData.azimuth;
		g_invalidHeadingCount = 0;
	}
	else
	{
		g_invalidHeadingCount++;
		if (g_invalidHeadingCount <= 20)
		{
			gpsData.azimuth = g_lastValidAzimuth;
		}
		else
		{
			gpsData.azimuth = g_lastValidAzimuth;
			gpsData.quality = -1;
		}
	}

	SHI::Data::StDBInfo d = { 0, };
	d.hook.gps.time = gpsData.time;
	d.hook.gps.latitude = gpsData.latitude;
	d.hook.gps.longitude = gpsData.longitude;
	d.hook.gps.hdop = gpsData.hdop;
	d.hook.gps.height = gpsData.height;
	d.hook.gps.azimuth = gpsData.azimuth;
	d.hook.gps.quality = m_bGpsConnected ? gpsData.quality : -1;
	if (m_bGpsConnected && g_invalidHeadingCount > 20)
	{
		d.hook.gps.quality = -1;
	}
	d.hook.gps.numOfSatellites = gpsData.numOfSatellites;
	d.hook.gps.latitude2 = gpsData.latitude;
	d.hook.gps.longitude2 = gpsData.longitude;

	d.latitude = gpsData.latitude;
	d.logitude = gpsData.longitude;

	GetStubDBInfo()->WriteData(&d);

	SetGpsValid(IsValidGps(gpsData));
//#ifdef _DEBUG
	//float Latitude;
	//float Lat_decimalPart = std::modf(d.latitude, &Latitude);
	//Lat_decimalPart *= 60;
	//float Longitude;
	//float Long_decimalPart = std::modf(d.logitude, &Longitude);
	//Long_decimalPart *= 60;
	//printf("latitude: %d.%d, logitude: %d.%d, height: %f\n", (int)Latitude, (int)std::round(Lat_decimalPart), (int)Longitude, (int)std::round(Long_decimalPart), gpsData.height);
	//printf("OnGpsData latitude: %.3f, logitude: %.3f, height: %f, Quality : %d\n", d.latitude, d.logitude, gpsData.height, gpsData.quality > 0 ? gpsData.quality : 0);
//#endif

	gpsFrames()++;
}
void CCollisionProcessorManagerDlg::OnGpsConnected(const std::string& ip, int32_t port)
{
	m_bGpsConnected = true;//pjh
	SetDlgItemText(IDC_TXT_DBMS, "GPS :");
	SetDlgItemText(IDC_TXT_DB_CONNECTION, "Connected");
}

void CCollisionProcessorManagerDlg::OnGpsDisConnected(const std::string& ip, int32_t port)
{
	m_bGpsConnected = false; //pjh
	SetDlgItemText(IDC_TXT_DBMS, "GPS :");
	SetDlgItemText(IDC_TXT_DB_CONNECTION, "Disconnected");	
}
