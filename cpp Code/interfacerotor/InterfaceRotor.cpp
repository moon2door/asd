// ReSharper disable CppClangTidyCertErr33C
#include <Routine/include/Base/CTime.h>
#include <Config/CraneInfo.h>
#include <Utility/BuildInfo.h>
#include <utility/Config.h>
#include <algorithm>
#include <cctype>
#include "InterfaceRotor.h"

namespace SHI
{
	//cms
	const char* GetLidarName(uint8_t code)
	{
		switch (code)
		{
		case Device::LIDAR_VLP16:          return "Velodyne VLP-16";
		case Device::LIDAR_OUSTER_16_OLD:  return "Ouster OS1-16 (old)";
		case Device::LIDAR_OUSTER_32_OLD:  return "Ouster OS1-32 (old)";
		case Device::LIDAR_OUSTER_64_OLD:  return "Ouster OS1-64 (old)";
		case Device::LIDAR_OUSTER_16:      return "Ouster OS1-16";
		case Device::LIDAR_OUSTER_32:      return "Ouster OS1-32";
		case Device::LIDAR_OUSTER_64:      return "Ouster OS1-64";
		case Device::LIDAR_OUSTER_32_rev7:   return "Ouster OS1-32 (rev7)";
		case Device::LIDAR_OUSTER_OS2_64:   return "Ouster OS2-64";
		case Device::LIDAR_HESAI_20A:      return "Hesai Pandar20A";
		case Device::LIDAR_HESAI_40P:      return "Hesai Pandar40P";
		default:                           return "Unknown LiDAR";
		}
	}
	//

	//pjh
	CInterfaceRotor::CInterfaceRotor()
		:m_viewer(nullptr)
	{
		
	}
	//

	void CInterfaceRotor::Create(int32_t argc)
	{
		const uint32_t buildDate = BuildInfo::GetBuildDate();
		printf("[InterfaceRotor] ver %u\n", buildDate);

		const auto& pierName = Routine::GetConfigString("CraneType", "Pier", "Y", "CraneType.json");
		const int32_t pierId = ConvPierInt(pierName);
		const int32_t craneId = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
		m_fileCount = argc;
		const int32_t numSensor = GetNumSensor(pierId, craneId);
		
		//cms
		// 배열로 저장
		std::vector<uint8_t> revisions;  revisions.reserve(numSensor);
		for (int32_t i = 0; i < numSensor; ++i) {
			std::string section = "Rotor" + std::to_string(i + 1);          // "Rotor0", "Rotor1", …
			const std::string& prodLine = Routine::GetConfigString(section.c_str(), "prodLine", "OS-1-32-0");	// OS-2-64-0, OS-1-32-7 등
			const std::string& lidarProfile = Routine::GetConfigString(section.c_str(), "lidarProfile", "");
			int rev = -1;
			uint8_t lidar_id = Device::LidarIdFromProdLine(prodLine, &rev);
			
			if (!lidarProfile.empty() && (lidar_id == Device::LIDAR_OUSTER_32 || lidar_id == Device::LIDAR_OUSTER_32_rev7))
			{
				std::string profileLower = lidarProfile;
				std::transform(profileLower.begin(), profileLower.end(), profileLower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			
				if (profileLower.rfind("rng", 0) == 0)
				{
					lidar_id = Device::LIDAR_OUSTER_32_rev7;
				}
				else if (profileLower.rfind("legacy", 0) == 0)
				{
					lidar_id = Device::LIDAR_OUSTER_32;
				}
			}
			
			printf("lidar prodLine is : %s\n", prodLine);

			revisions.push_back(lidar_id);
		}
		//

		m_pierId = pierId;
		m_craneId = craneId;
		printf("Pier %s, Crane %s, Num sensor = %d\n", ConvPierDisplayStr(pierId).c_str(), ConvCraneStr(pierId, craneId).c_str(), numSensor);

		const bool bCreateStubXYZPoint = GetStubXYZPoint()->Create(1);
		printf("Create StubXYZPoint : %s\n", bCreateStubXYZPoint ? "succeed" : "failed");
		const bool bCreateStubRotorStatus = GetStubRotorStatus()->Create(1);
		printf("Create StubRotorStatus : %s\n", bCreateStubRotorStatus ? "succeed" : "failed");
		const bool bCreateStubLogSyncTime = GetStubLogSyncTime()->Create(1);
		printf("Create StubLogSyncTime : %s\n", bCreateStubLogSyncTime ? "succeed" : "failed");
		const bool bCreateProxyControl = GetProxyRotorControl()->Create(1);
		printf("Create Proxy Control : %s\n", bCreateProxyControl ? "succeed" : "failed");
		const bool bCreateProxyParameter = GetProxyRotorParameter()->Create(1);
		printf("Create ProxyParameter : %s\n", bCreateProxyParameter ? "succeed" : "failed");
		const bool bCreateStubRotorControl = GetStubRotorControl()->Create(2);
		printf("Create StubRotorControl : %s\n", bCreateStubRotorControl ? "succeed" : "failed");

		const float_t filterDistanceThreshold = static_cast<float_t>(Routine::GetConfigDouble("Options", "DistanceFilterThreshold", 20));
		const size_t maxLogFiles = static_cast<size_t>(Routine::GetConfigInt("Options", "maxLogFiles", 1200));
		
		m_pointCloud.get() = std::make_shared<Routine::CPointCloud<>>();

		for(uint8_t i =0; i < numSensor; i++)
		{
			const uint8_t id = i + 1;

			std::vector<float_t> altitudeAngles;
			std::vector<float_t> azimuthAngles;
			std::vector<float_t> intrinsicParameters;

			char szFieldName[256] = "";
			sprintf_s(szFieldName, "Rotor%d", id);
			Utility::GetConfigVector(szFieldName, "altitudeAngles", altitudeAngles);
			Utility::GetConfigVector(szFieldName, "azimuthAngles", azimuthAngles);
			Routine::CMatrix44f intrinsicMatrix = Utility::GetConfigMatrix44f(szFieldName, "intrinsicParameters");
			Routine::CMatrix44f extrinsicMatrix = Utility::GetConfigMatrix44f(szFieldName, "extrinsicParameters");
			const float_t targetRpm = static_cast<float_t>(Routine::GetConfigDouble(szFieldName, "targetRpm", 37.0));
			const float_t rpmTolerance = static_cast<float_t>(Routine::GetConfigDouble(szFieldName, "rpmTolerance", 2.0));
			
			char szDefaultAddressLidar[256] = "";
			sprintf_s(szDefaultAddressLidar, "192.168.10%d.101", id);
			const auto& addressLidar = Routine::GetConfigString(szFieldName, "addrLidar", szDefaultAddressLidar);

			char szDefaultAddressDsp[256] = "";
			sprintf_s(szDefaultAddressDsp, "192.168.10%d.102", id);
			const auto& addressDsp = Routine::GetConfigString(szFieldName, "addrDsp", szDefaultAddressDsp);

			char szDefaultAddressBind[256] = "";
			sprintf_s(szDefaultAddressBind, "192.168.10%d.103", id);
			const auto& addressBind = Routine::GetConfigString(szFieldName, "addrBind", szDefaultAddressBind);
			
			SensorNetConfig netConfig;
			netConfig.addrBind = addressBind;
			netConfig.addrLidar = addressLidar;
			netConfig.addrDsp = addressDsp;
			netConfig.lidarPort = 7501;
			netConfig.lidarBindPort = 7502;
			netConfig.dspPort = 6001;
			netConfig.dspCmdPort = 7001;
			netConfig.lidarRevision = revisions[i];
			m_sensorConfigs[i] = netConfig;
			m_targetRpm[i] = targetRpm;
			
			m_lidar[i].RegisterCallbackLidarPacket(&CInterfaceRotor::OnLidarPacket, this);
			m_lidar[i].SetAltitudeAngles(altitudeAngles);
			m_lidar[i].SetAzimuthAngles(azimuthAngles);
			//cms
			const bool bCreateLidar = m_lidar[i].CreateLidarDevice(i, revisions[i], addressBind, 7502, addressLidar, 7501);
			//const bool bCreateLidar = m_lidar[i].CreateLidarDevice(i, Device::LIDAR_OUSTER_32, addressBind, 7502, addressLidar, 7501);

			// Per-rotor scan start index (scan width rotation)
			const int32_t scanStart = Routine::GetConfigInt(szFieldName, "scanStart", 0);
			uint16_t scanWidth = 1024; // default for OS1_rev7/OS2
			m_workXYZ[i].SetScanStart(scanStart, scanWidth);

			m_dsp[i].RegisterCallbackRotorData(&CInterfaceRotor::OnRotorData, this);
			const bool bCreateDsp = m_dsp[i].CreatePala720Dsp(i, 6001, 7001, addressDsp, addressBind);
			
			m_workXYZ[i].SetMaxLogFiles(maxLogFiles);
			m_workXYZ[i].SetIntrinsicMatrix(intrinsicMatrix);
			m_workXYZ[i].SetExtrinsicMatrix(extrinsicMatrix);
			m_workXYZ[i].SetFilterDistance(filterDistanceThreshold);
			m_workXYZ[i].SetTargetRpm(targetRpm);
			m_workXYZ[i].SetRpmTolerance(rpmTolerance);
			const uint8_t sensorIndex = i;
			m_workXYZ[i].SetRotorCommandStateProvider([this, sensorIndex]() { return IsRotorCommandStart(sensorIndex); });
			m_workXYZ[i].RegisterCallbackWorkDone(&CInterfaceRotor::OnWorkXYZDone, this);
			printf("Create lidar(%s): %s %s\n", addressLidar.c_str(), GetLidarName(revisions[i]), bCreateLidar? "succeed" : "failed");
			printf("Create rotor(%s): %s\n", addressDsp.c_str(), bCreateDsp ? "succeed" : "failed");
		}

		m_timerProcessPoints.StartTimer(1000, &CInterfaceRotor::OnTimerProcessPoints, this);
		m_timerProcessResult.StartTimer(1000, &CInterfaceRotor::OnTimerProcessResult, this);
	}

	void CInterfaceRotor::SetRotation(uint8_t id, bool bRotate)
	{		
		const auto rotorControl = std::make_shared<SHI::Data::StRotorControl>();
		
		if(m_dsp.find(id) != m_dsp.end())
		{
			UpdateRotorCommandState(id, bRotate);
			printf("---------- [%d]InterfaceRotor Order = %s\n", id, bRotate > 0 ? "Start" : "Stop");//pjh
			if(bRotate)
			{
				m_dsp[id].CommandModeChange();
				rotorControl->SensorNumber = id;
				rotorControl->ControlMotor.bStart = true;
				// for SHM
				GetStubRotorControl()->WriteData(rotorControl.get());

			}
			else
			{
				rotorControl->SensorNumber = id;
				rotorControl->ControlMotor.bStart = false;
				// for SHM
				GetStubRotorControl()->WriteData(rotorControl.get());
				m_dsp[id].CommandStop();
			}
		}
	}
	
	void CInterfaceRotor::OnRotorParameter(SHI::Data::StRotorParameter* pData)
	{
#ifdef TEMP_KSG
		UpdateRotorParameter(pData);
#endif
	}

	void CInterfaceRotor::OnRotorControl(SHI::Data::StRotorControl* pData)
	{
		printf("%d:%s\n", pData->SensorNumber, pData->ControlMotor.bStart ? "True" : "False");
		if(m_dsp.find(pData->SensorNumber) != m_dsp.end())
		{
			UpdateRotorCommandState(pData->SensorNumber, pData->ControlMotor.bStart);
			if(pData->ControlMotor.bStart)
			{
				m_dsp[pData->SensorNumber].CommandModeChange();
			}
			else
			{
				m_dsp[pData->SensorNumber].CommandStop();
			}
		}
	}

	void CInterfaceRotor::OnLidarPacket(uint8_t id, const Device::LidarPacket& packet)
	{
		m_rotorData[id].LockRead();
		auto rotorData = m_rotorData[id].get();
		m_rotorData[id].UnLockRead();

		m_packetBuffer[id].Lock();
		if(m_packetBuffer[id].get() == nullptr	)
		{
			m_packetBuffer[id].get() = std::make_shared<std::deque<Pala720Packet>>();
		}

		m_packetBuffer[id].get()->emplace_back(id, packet, rotorData);
		if (m_packetBuffer[id].get()->size() >= m_queueSize)
		{
			m_packetBuffer[id].get()->pop_front();
		}
#ifdef _DEBUG
		if (m_packetBuffer[id].get()->empty() == false)
		{
			uint8_t tmp_id = m_packetBuffer[id].get()->back().id;
			if (m_packetBuffer[id].get()->back().lidarPacket.lidarSignals.empty() == false)
			{
				float_t az = m_packetBuffer[id].get()->back().lidarPacket.lidarSignals.back().azimuth_deg;
				printf("Read log id: %d, azimuth: %f\n", (int)tmp_id, az);
			}
		}
#endif
		m_packetBuffer[id].UnLock();

		const auto nowTime = std::chrono::steady_clock::now();
		{
			std::lock_guard<std::mutex> lock(m_packetMetaMutex);
			m_lastPacketTime[id] = nowTime;
			m_packetSequence[id]++;
		}
	}

	void CInterfaceRotor::OnRotorData(uint8_t id, const Device::Pala720dsp::StRotorData* rotorData)
	{
		m_rotorData[id].Lock();
		m_rotorData[id].get() = *rotorData;
		m_rotorData[id].UnLock();
	}

	void CInterfaceRotor::SetVisualize(bool bShow)
	{
		std::lock_guard<std::mutex> lock(m_viewerMutex);

		if (bShow)
		{
			if (!m_viewer)
			{
				m_viewer = std::make_shared<Routine::Visualization::CPointCloudViewer>();
				m_viewer->SetZoom(100);
			}

			m_viewer->Show();
			// Ensure immediate first paint without extra interaction
			m_viewer->RedrawWindow();
			m_visualizeEnabled.store(true, std::memory_order_release);
		}
		else
		{
			m_visualizeEnabled.store(false, std::memory_order_release);
			if (m_viewer)
			{
				m_viewer->Hide();
			}
		}
	}


	void CInterfaceRotor::OnTimerProcessPoints()
	{
		const auto now = std::chrono::steady_clock::now();

		for(auto& pair : m_packetBuffer)
		{
			const uint8_t sensorId = pair.first;
			uint64_t currentSeq = 0;
			uint64_t lastSeq = 0;
			std::chrono::steady_clock::time_point lastTime{};
			bool hasLastTime = false;

			{
				std::lock_guard<std::mutex> lock(m_packetMetaMutex);
				currentSeq = m_packetSequence[sensorId];
				lastSeq = m_lastProcessedSequence[sensorId];
				const auto timeIt = m_lastPacketTime.find(sensorId);
				if (timeIt != m_lastPacketTime.end())
				{
					lastTime = timeIt->second;
					hasLastTime = true;
				}
			}

			const bool hasNewData = currentSeq != lastSeq;
			const bool timedOut = !hasLastTime || (now - lastTime > m_packetTimeout);

			if (!hasNewData)
			{
				if (timedOut)
				{
					pair.second.Lock();
					if (pair.second.get())
					{
						pair.second.get()->clear();
					}
					pair.second.UnLock();

					const auto visIt = m_pointCloudVisualize.find(sensorId);
					if (visIt != m_pointCloudVisualize.end())
					{
						visIt->second.Lock();
						visIt->second.get() = std::make_shared<Routine::CPointCloud<>>();
						visIt->second.UnLock();
					}
					TryRecoverSensor(sensorId);
				}
				continue;
			}

			const auto packetBuffer = std::make_shared<std::deque<Pala720Packet>>();

			pair.second.Lock();
			if(pair.second.get())
			{
				std::copy(pair.second.get()->begin(), pair.second.get()->end(), std::back_inserter(*packetBuffer));
			}
			pair.second.UnLock();

			if (packetBuffer->empty())
			{
				continue;
			}

			{
				std::lock_guard<std::mutex> lock(m_packetMetaMutex);
				m_lastProcessedSequence[sensorId] = currentSeq;
			}

			m_workXYZ[sensorId].StartWorkXYZ(packetBuffer);
		}
	}

void CInterfaceRotor::TryRecoverSensor(uint8_t sensorId)
	{
		const auto now = std::chrono::steady_clock::now();
		{
			std::lock_guard<std::mutex> lock(m_packetMetaMutex);
			const auto it = m_lastRecoverAttempt.find(sensorId);
			if (it != m_lastRecoverAttempt.end() && (now - it->second) < m_recoverCooldown)
			{
				return;
			}
			m_lastRecoverAttempt[sensorId] = now;
		}

		const auto cfgIt = m_sensorConfigs.find(sensorId);
		if (cfgIt == m_sensorConfigs.end())
		{
			return;
		}
		const auto& cfg = cfgIt->second;

		printf("[InterfaceRotor] sensor %u timeout -> recreate DSP sockets\n", static_cast<unsigned int>(sensorId));

		auto dspIt = m_dsp.find(sensorId);
		if (dspIt != m_dsp.end())
		{
			dspIt->second.RegisterCallbackRotorData(&CInterfaceRotor::OnRotorData, this);
			const bool recreated = dspIt->second.Reconnect(sensorId, cfg.dspPort, cfg.dspCmdPort, cfg.addrDsp, cfg.addrBind);
			if (recreated)
			{
				const auto rpmIt = m_targetRpm.find(sensorId);
				if (rpmIt != m_targetRpm.end())
				{
					dspIt->second.CommandRpmSet(rpmIt->second);
				}
				if (IsRotorCommandStart(sensorId))
				{
					dspIt->second.CommandModeChange();
				}
			}
		}
	}

void CInterfaceRotor::OnTimerProcessResult()
	{
		// Write rotorStatus
		const auto rotorStatus = std::make_shared<SHI::Data::StRotorStatus>();

		rotorStatus->NumSensor = static_cast<uint32_t>(m_lidar.size());
		rotorStatus->Pier = m_pierId;
		rotorStatus->Crane = m_craneId;
		rotorStatus->VersionInterfacecRotor = 0;
		rotorStatus->VersionSensorManager = BuildInfo::GetBuildDate();

		for (uint8_t i = 0; i < 5; i++)
		{
			if (m_lidar.find(i) == m_lidar.end())
			{
				rotorStatus->RotorStatus[i].LidarFps = 0;
				rotorStatus->RotorStatus[i].LidarTempTop = 0;
				rotorStatus->RotorStatus[i].LidarTempBottom = 0;
				rotorStatus->RotorStatus[i].LidarNetworkStatus = 0;	// 0 : 비정상, 1 : 정상
				rotorStatus->RotorStatus[i].LidarNetworkRangeData = 0;	// 0 : 비정상, 1 : 정상
				rotorStatus->RotorStatus[i].LidarErrorCode = 1; // 0 : 정상, 1 : RPM 비정상
			}
			else
			{
				rotorStatus->RotorStatus[i].LidarFps = static_cast<int16_t>(m_lidar[i].fps());
				rotorStatus->RotorStatus[i].LidarTempTop = 0;
				rotorStatus->RotorStatus[i].LidarTempBottom = 0;
				rotorStatus->RotorStatus[i].LidarNetworkStatus = m_lidar[i].fps() > 0 ? (1) : (0);	// 0 : 비정상, 1 : 정상
				rotorStatus->RotorStatus[i].LidarNetworkRangeData = m_lidar[i].maxDistance() > 100 ? (1) : (0);	// 0 : 비정상, 1 : 정상
				rotorStatus->RotorStatus[i].LidarErrorCode = 0; // 0 : 정상, 1 : RPM 비정상	
				if(m_lidar[i].fps() == 0)
				{
					rotorStatus->RotorStatus[i].LidarErrorCode = 1; // 0 : 정상, 1 : RPM 비정상	
				}

				for (int nSize= 0; nSize < 16; nSize++)
				{
					rotorStatus->RotorParameter[i].intrinsicsMatrix[nSize] = m_workXYZ[i].GetIntrinsicsMaxtrix().data()[nSize];
					rotorStatus->RotorParameter[i].extrinsicsMatrix[nSize] = m_workXYZ[i].GetExtrinsicsMaxtrix().data()[nSize];
				}
				
			}

			if (m_dsp.find(i) == m_dsp.end())
			{
				rotorStatus->RotorStatus[i].ActuratorRpm = 0;
				rotorStatus->RotorStatus[i].reserved = 0;
				rotorStatus->RotorStatus[i].ActuratorFPS = 0;
				rotorStatus->RotorStatus[i].ActuratorPacket = 1; // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorRotation = 0; // 0 : ?��?, 1 : ?�전
				rotorStatus->RotorStatus[i].ActuratorProxyMeter = 1; // 근접?�서 ?�태, 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorZeroSet = 1; // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorEncoder = 1; // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorNetwork = 1; // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ErrorCode = 0;
			}
			else
			{
				rotorStatus->RotorStatus[i].ActuratorRpm = static_cast<float>(m_dsp[i].rpm());
				rotorStatus->RotorStatus[i].reserved = 0;
				rotorStatus->RotorStatus[i].ActuratorFPS = static_cast<uint16_t>(m_dsp[i].fps());
				rotorStatus->RotorStatus[i].ActuratorPacket = m_dsp[i].fps() > 0 ? (0) : (1); // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorRotation = m_dsp[i].rpm() > 0 ? (1) : (0); // 0 : ?��?, 1 : ?�전
				rotorStatus->RotorStatus[i].ActuratorProxyMeter = 0; // 근접?�서 ?�태, 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorZeroSet = 0; // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorEncoder = m_dsp[i].rpm() > 0 ? (0) : (1); // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ActuratorNetwork = 0; // 0 : ?�상, 1 : 비정??
				rotorStatus->RotorStatus[i].ErrorCode = 0;
				if (m_dsp[i].fps() < 1.0)
				{
					rotorStatus->RotorStatus[i].ErrorCode = 0x07;
				}
			}

		}

		GetStubRotorStatus()->WriteData(rotorStatus.get());

		// Write PointCloud
		auto pointCloud = std::make_shared<Routine::CPointCloud<>>();
		m_pointCloud.Lock();
		m_pointCloud.get().swap(pointCloud);
		m_pointCloud.UnLock();
		
		if (pointCloud && pointCloud->size() > 0)
		{
			// Write xyzPoints
			const auto xyzPoints = std::make_shared<SHI::Data::StXYZPoints>();
			xyzPoints->InitBuffer();

			const size_t pointSize = (std::min)(pointCloud->size(), static_cast<size_t>(500000));
			if (xyzPoints->AllocXYZ(pointSize))
			{
				const Routine::CTime t;
				xyzPoints->timeStamp = t.Hour() * 3600.0 + t.Min() * 60.0 + t.Second() * 1.0 + t.MilliSecond() * 0.001;

				for (size_t i = 0; i < pointSize; i++)
				{
					xyzPoints->GetXYZ()[i].X = pointCloud->GetPoints()[i].x();
					xyzPoints->GetXYZ()[i].Y = pointCloud->GetPoints()[i].y();
					xyzPoints->GetXYZ()[i].Z = pointCloud->GetPoints()[i].z();
					xyzPoints->GetXYZ()[i].W = 0;
				}
				if(m_fileCount > 1)
				{
					GetStubXYZPoint()->WriteData(xyzPoints.get());
				}
				else
				{
					if (rotorStatus->RotorStatus[0].ActuratorRpm > 35.0f || rotorStatus->RotorStatus[1].ActuratorRpm > 35.0f)
					{
						GetStubXYZPoint()->WriteData(xyzPoints.get());
					}
				}

				
			}
			

			bool shouldVisualize = m_visualizeEnabled.load(std::memory_order_acquire);
			std::shared_ptr<Routine::Visualization::CPointCloudViewer> viewer;
			if (shouldVisualize)
			{
				std::lock_guard<std::mutex> viewerLock(m_viewerMutex);
				if (!m_visualizeEnabled.load(std::memory_order_relaxed) || !m_viewer)
				{
					shouldVisualize = false;
				}
				else
				{
					viewer = m_viewer;
				}
			}

			if (shouldVisualize && viewer)
			{
				// Show
				const auto pclCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
				for(const auto &pointCloudVis : m_pointCloudVisualize)
				{
					const auto keyCode = pointCloudVis.first;
					auto points = pointCloudVis.second;

					points.Lock();
					const PointCloudPtr pointCloudVisPtr = points.get();
					points.UnLock();

					pclCloud->reserve(pointCloudVisPtr->size());
					for (const auto& p : pointCloudVisPtr->GetPoints())
					{
						pclCloud->emplace_back(pcl::PointXYZI(p.x(), p.y(), p.z(), keyCode));
					}
				}

				if (pclCloud->size() != 0)
				{
					UpdateShowXyz(pclCloud);
				}

				Routine::GL::CGlModel glModel;
				for(auto& pointCloudBuffer : m_pointCloudVisualize)
				{
					const auto& keyCode = pointCloudBuffer.first;
					pointCloudBuffer.second.Lock();
					const auto pointCloudShow = pointCloudBuffer.second.get();
					pointCloudBuffer.second.UnLock();
			
					Routine::GL::CGlObject glPoints;
					float_t r, g, b;
					switch(keyCode)
					{
					case 0:
						r = 1; g = 0; b = 0;
						break;
					case 1:
						r = 0; g = 1; b = 0;
						break;
					case 2:
						r = 0; g = 0; b = 1;
						break;
					case 3:
						r = 1; g = 1; b = 0;
						break;
					case 4:
						r = 1; g = 0; b = 1;
						break;
					case 5:
						r = 0; g = 1; b = 1;
						break;
					default:
						r = 1; g = 1; b = 1;
						break;
					}
			
					glPoints.SetDrawColor(r, g, b);
					for(const auto& pt : pointCloudShow->GetPoints())
					{
						glPoints.AddPoint(pt.x(), pt.y(), pt.z());
					}
					glModel.AddObject(glPoints);
				}
			
				Routine::GL::CGlObject glArrow;
				glArrow.SetType(Routine::GL::ObjectType::GlLines);
				glArrow.AddPoint(0, 0, 0);
				glArrow.AddPoint(10, 0, 0);
				glArrow.AddColor(1, 0, 0);
				glArrow.AddColor(1, 0, 0);
			
				glArrow.AddPoint(0, 0, 0);
				glArrow.AddPoint(0, 10, 0);
				glArrow.AddColor(0, 1, 0);
				glArrow.AddColor(0, 1, 0);
			
				glArrow.AddPoint(0, 0, 0);
				glArrow.AddPoint(0, 0, 10);
				glArrow.AddColor(0, 0, 1);
				glArrow.AddColor(0, 0, 1);
				glModel.AddObject(glArrow);
			
				// Render to viewer immediately if available (avoid requiring extra mouse input)
				if (m_visualizeEnabled.load(std::memory_order_acquire))
				{
					viewer->ShowModel(glModel);
				}
			}
		}
		
		std::string strStatus;
		for(auto& pair : m_lidar)
		{
			char lidarStatus[256] = "";
			sprintf_s(lidarStatus, "[%d] %.1lffps(%s) ", pair.first, pair.second.fps(), pair.second.sensorInfo().c_str());

			char rotorStatus[256] = "";
			try
			{
				auto& dsp = m_dsp[pair.first];
				sprintf_s(rotorStatus, "dsp %.1lffps, %.1lfrpm ", dsp.fps(), dsp.rpm());
			}
			catch (...)
			{
			}
			strStatus += lidarStatus;
			strStatus += rotorStatus;
			strStatus += "| ";
		}

		char outputStatus[256] = "";
		size_t pointCloudSize = 0;
		if (pointCloud) pointCloudSize = pointCloud->size();
		sprintf_s(outputStatus, "xyz %llupoints \n", pointCloudSize);
		strStatus += outputStatus;

		printf("%s", strStatus.c_str());
	}

	void CInterfaceRotor::OnWorkXYZDone(const Routine::CWorker& worker)
	{
		const auto workXYZ = reinterpret_cast<SHI::WorkXYZ::CWorkXYZ*>(&const_cast<Routine::CWorker&>(worker));

		if(const auto outBuffer = workXYZ->GetOutput())
		{
			// Copy to process buffer
			m_pointCloud.Lock();
			m_pointCloud.get()->insert(m_pointCloud.get()->end(), outBuffer->begin(), outBuffer->end());
			m_pointCloud.UnLock();

			// Copy to visualize buffer
			for (auto& w : m_workXYZ)
			{
				if (&w.second == &worker)
				{
					const auto& keyCode = w.first;

					m_pointCloudVisualize[keyCode].Lock();
					m_pointCloudVisualize[keyCode].get() = outBuffer;
					m_pointCloudVisualize[keyCode].UnLock();
				}
			}
		}		
	}

	void CInterfaceRotor::UpdateRotorCommandState(uint8_t id, bool bRotate)
	{
		std::lock_guard<std::mutex> lock(m_rotorCommandMutex);
		m_rotorCommandStates[id] = bRotate;
	}

	bool CInterfaceRotor::IsRotorCommandStart(uint8_t id) const
	{
		std::lock_guard<std::mutex> lock(m_rotorCommandMutex);
		const auto it = m_rotorCommandStates.find(id);
		if (it != m_rotorCommandStates.end())
		{
			return it->second;
		}
		return false;
	}
}

