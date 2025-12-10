#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"
#include <Routine/Include/Base/RoutineUtility.h>
#include <Routine/include/Base/CTime.h>
#include <Config/CraneInfo.h>
#include <Data/StMaintenanceInfo.h>

void CCollisionProcessorManagerDlg::OnTimerReportSystemStatus()
{
	//////////////////////////////////////////////////////////////////////////
	// Reset the status snapshot before populating new values
	GetSystemStatus().InitBuffer();

	if (m_objSensorManager->GetSocketTCP()->GetClientInfo().size() > 0)
	{
		// Allocate rotor status entries for each sensor
		GetSystemStatus().AllocRotorStatus(GetRotorStatus().NumSensor);

		for (uint32_t i = 0; i < GetRotorStatus().NumSensor; i++)
		{
			GetSystemStatus().GetRotorStatus()[i] = GetRotorStatus().RotorStatus[i];
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Timestamp
	static Routine::CElapseTimer elapsedTimer;
	GetSystemStatus().timeStamp = elapsedTimer.GetElapseTime() * 0.000001;

	//////////////////////////////////////////////////////////////////////////
	// Update network state flags
	const size_t sensorClients = m_objSensorManager->GetSocketTCP()->GetClientInfo().size();
	int32_t numConnections = m_craneMonitoringInterface->GetSocketTCP()->GetClientInfo().size();
	GetSystemStatus().StatusNetwork.SensorCom = sensorClients == 0 ? 0 : 1;
	GetSystemStatus().StatusNetwork.DBMS = (routerFrames() > 0 || numConnections > 0) ? 1 : 0;
		// pjh 240509 m_dbConnector->IsConneted() == false ? 0 : 1;
	GetSystemStatus().StatusNetwork.PLC = GetPLCInfo().plcConnected == 0 ? 0 : 1;
	GetSystemStatus().StatusNetwork.GPS = gpsFrames() > 0 ? 1 : 0; // 1 means healthy, 0 means no data
	GetSystemStatus().StatusNetwork.Router = (routerFrames() > 0 || numConnections > 0) ? 1 : 0; // 1 means healthy, 0 means no data
	GetSystemStatus().StatusNetwork.GPSData = IsGpsValid() ? 1 : 0;

	static bool s_prevHadConnections = true;
	static bool s_noClientWarningPrinted = false;
	static Routine::CElapseTimer s_noClientElapsed;

	const bool hasConnections = numConnections > 0;
	if (hasConnections)
	{
		if (!s_prevHadConnections)
		{
			const double downTimeSec = static_cast<double>(s_noClientElapsed.GetElapseTimeContinue()) * 0.000001;
			printf("\n######## MONITORING RECONNECTED ######## (downtime %.1f sec, sensorClients=%zu)\n\n", downTimeSec, sensorClients);
		}
		s_prevHadConnections = true;
		s_noClientWarningPrinted = false;
	}
	else
	{
		if (s_prevHadConnections)
		{
			s_noClientElapsed.Reset();
			printf("\n######## WARNING: MONITORING CLIENT COUNT DROPPED TO ZERO ######## (sensorClients=%zu)\n\n", sensorClients);
			s_prevHadConnections = false;
			s_noClientWarningPrinted = false;
		}
		else if (!s_noClientWarningPrinted)
		{
			const uint32_t elapsedNoClientUs = s_noClientElapsed.GetElapseTimeContinue();
			if (elapsedNoClientUs >= 10000000)
			{
				const double elapsedSec = static_cast<double>(elapsedNoClientUs) * 0.000001;
				printf("\n######## WARNING: MONITORING CLIENTS STILL DISCONNECTED ######## (%.1f sec elapsed)\n\n", elapsedSec);
				s_noClientWarningPrinted = true;
			}
		}
	}

	if (numConnections == 0) return; // No connected clients

	routerFrames() = 0;
	//printf("%d\n", GetSystemStatus().StatusNetwork.Router);

	//////////////////////////////////////////////////////////////////////////
	// Clear emergency stop flag
	GetSystemStatus().ModeEmergencyStop		= 0;

	//////////////////////////////////////////////////////////////////////////
	// Process version information
	GetSystemStatus().Version.Interface = GetRotorStatus().VersionInterfacecRotor;

	// Version
	GetSystemStatus().Version.ManagerSensor = GetRotorStatus().VersionSensorManager;
	GetSystemStatus().Version.ManagerCollision = SHI::BuildInfo::GetBuildDate();

	//////////////////////////////////////////////////////////////////////////
	// Cooperative mode from DB (disabled)
	//GetSystemStatus().StatusCooperationMode = m_dbConnector->GetCooperationInfo();

	// Update wind information
	GetSystemStatus().WindSpeed = m_windInfo.windSpeed;
	GetSystemStatus().WindDirection = m_windInfo.windDirection;

	// Send updated status
	m_craneMonitoringInterface->SendSystemStatus(&GetSystemStatus());

	//////////////////////////////////////////////////////////////////////////
	// GPS health watchdog

	const bool bEnableReset = Routine::GetConfigInt("Gps Setting", "Enable", 1) == 1;
	const int32_t resetTime = Routine::GetConfigInt("Gps Setting", "Time", 10);
	//
	if (bEnableReset)
	{
		if(gpsFrames() == 0)
		{
			numZeroFrame()++;

			if (numZeroFrame() > resetTime)
			{
				//SHI::Data::StDBInfo d = { 0, };
				//GetStubDBInfo()->WriteData(&d);

				//pjh
				/*int32_t gpsPort = Routine::GetConfigInt("GPS", "Port", 5019);
				std::string gpsAddr = Routine::GetConfigString("GPS", "Address", "192.168.1.12");*/

				int32_t gpsPort = Routine::GetConfigInt("GPS", "Port", 5019, "IPConfig.ini");
				std::string gpsAddr = Routine::GetConfigString("GPS", "Address", "192.168.1.12", "IPConfig.ini");
				//

				printf("GPS is not connected. reset connection(%s,%d)\n", gpsAddr.c_str(), gpsPort);

				// Reset
				m_gps.Destroy();
				m_gps.Create(gpsAddr, gpsPort);
				numZeroFrame() = 0;

				// Send empty gps data
				SHI::Device::TDR2000::StGpsData gpsData;
				OnGpsData(gpsData);
			}
		}
		else
		{
			numZeroFrame() = 0;
		}
	}
	
	char text[256] = "";
	sprintf_s(text, "%d fps", gpsFrames());
	SetDlgItemText(IDC_TXT_DB_FPS, text);
	gpsFrames() = 0;

	///////////////////////////////////////////////////////////////////////////
	// Send viewer maximum distance information
	SHI::Data::StMaxDistance *maxDistance = new (std::nothrow) SHI::Data::StMaxDistance();
	maxDistance->maxDistance = Routine::GetConfigDouble("Viewer Distance", "Max(m)", 50.0f);
	int ret = m_craneMonitoringInterface->SendMaxDistance(maxDistance);

	//////////////////////////////////////////////////////////////////////////
	// Send maintenance data
	SHI::Data::StMaintenanceInfo *maintenanceInfo = new (std::nothrow) SHI::Data::StMaintenanceInfo();
	if (maintenanceInfo)
	{
		memset(maintenanceInfo, 0, sizeof(SHI::Data::StMaintenanceInfo));
		std::string szPier = Routine::GetConfigString("CraneType", "Pier", "Y", "CraneType.json");
		int32_t pier = SHI::ConvPierInt(szPier);
		int32_t crane = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
		int32_t numSensor = SHI::GetNumSensor(pier, crane);
		for (int32_t i = 0; i < numSensor; i++)
		{
			char lidarSerial[32] = "123456";
			char rotorSerial[32] = "123456";
			std::string tempLidarSerial;
			std::string tempRotorSerial;
			std::string lidarOperationStart = "2022-07-05 12:00";
			std::string rotorOperationStart = "2022-07-05 12:00";
			char field[256] = "";
			sprintf(field, "Rotor%d", i+1);
			tempLidarSerial = Routine::GetConfigString(field, "LidarSerial", "os1-1","interfacerotor.json");
			tempRotorSerial = Routine::GetConfigString(field, "RotorSerial", "os1-2", "interfacerotor.json");
			lidarOperationStart = Routine::GetConfigString(field, "LidarOperationStart",  "2020-12-01 00:00", "interfacerotor.json");
			rotorOperationStart = Routine::GetConfigString(field, "RotorOperationStart",  "2020-12-01 00:00", "interfacerotor.json");

			memcpy(maintenanceInfo->info[i].LidarSerial, tempLidarSerial.c_str(), sizeof(lidarSerial));
			memcpy(maintenanceInfo->info[i].RotorSerial, tempRotorSerial.c_str(), sizeof(rotorSerial));

			maintenanceInfo->info[i].LidarStartDate.year = std::stoi(lidarOperationStart.substr(0, 4));
			maintenanceInfo->info[i].LidarStartDate.month = std::stoi(lidarOperationStart.substr(5, 2));
			maintenanceInfo->info[i].LidarStartDate.date = std::stoi(lidarOperationStart.substr(8, 2));
			maintenanceInfo->info[i].LidarStartDate.hour = std::stoi(lidarOperationStart.substr(11, 2));
			maintenanceInfo->info[i].LidarStartDate.min = std::stoi(lidarOperationStart.substr(14, 2));

			maintenanceInfo->info[i].RotorStartDate.year = std::stoi(rotorOperationStart.substr(0, 4));
			maintenanceInfo->info[i].RotorStartDate.month = std::stoi(rotorOperationStart.substr(5, 2));
			maintenanceInfo->info[i].RotorStartDate.date = std::stoi(rotorOperationStart.substr(8, 2));
			maintenanceInfo->info[i].RotorStartDate.hour = std::stoi(rotorOperationStart.substr(11, 2));
			maintenanceInfo->info[i].RotorStartDate.min = std::stoi(rotorOperationStart.substr(14, 2));
		}

		m_craneMonitoringInterface->SendMaintenanceInfo(maintenanceInfo);
		delete maintenanceInfo;
	}
}
