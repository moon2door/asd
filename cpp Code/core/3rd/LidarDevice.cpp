#include "LidarDevice.h"

namespace Device
{
	CLidarDevice::CLidarDevice()
	{
		RegisterCallbackLidarPacket(&CLidarDevice::OnLidarPacket, this);
	}

	bool CLidarDevice::CreateLidarDevice(uint8_t lidarId, uint8_t lidarCode, const std::string& bindIp, uint16_t bindPort, const std::string& lidarIp, uint16_t lidarPort)
	{
		bool ret = false;
		m_lidarId = lidarId;
		if(lidarCode == Device::LIDAR_OUSTER_16)
		{
			auto lidar = new OS1_16::COS1_16();
			lidar->RegisterCallbackRecieveLidar(&CLidarDevice::OnReceiveOS1_16, this);
			lidar->RegisterCallbackRecieveStatus(&CLidarDevice::OnReceiveStatusOuster, this);
			if (lidar->Create(lidarPort, bindPort, 7503, lidarIp, bindIp))  // NOLINT(readability-suspicious-call-argument)
			{
				m_hLidar = std::make_unique<Routine::handle_t>(lidar);
				m_lidarCode = lidarCode;
				ret = true;
			}
		}
		else if (lidarCode == Device::LIDAR_OUSTER_32)
		{
			auto lidar = new OS1_32::COS1_32();
			lidar->RegisterCallbackRecieveLidar(&CLidarDevice::OnReceiveOS1_32, this);
			lidar->RegisterCallbackRecieveStatus(&CLidarDevice::OnReceiveStatusOuster, this);
			if (lidar->Create(lidarPort, bindPort, 7503, lidarIp, bindIp))  // NOLINT(readability-suspicious-call-argument)
			{
				m_hLidar = std::make_unique<Routine::handle_t>(lidar);
				m_lidarCode = lidarCode;
				ret = true;
			}
		}
		else if (lidarCode == Device::LIDAR_OUSTER_32_rev7)
		{
			auto lidar = new OS1_32_rev7::COS1_32_rev7();
			lidar->RegisterCallbackRecieveLidar(&CLidarDevice::OnReceiveOS1_32_rev7, this);
			lidar->RegisterCallbackRecieveStatus(&CLidarDevice::OnReceiveStatusOuster, this);
			if (lidar->Create(lidarPort, bindPort, 7503, lidarIp, bindIp))  // NOLINT(readability-suspicious-call-argument)
			{
				m_hLidar = std::make_unique<Routine::handle_t>(lidar);
				m_lidarCode = lidarCode;
				ret = true;
			}

		}

		else if (lidarCode == Device::LIDAR_OUSTER_64)
		{
			auto lidar = new OS1_64::COS1_64();
			lidar->RegisterCallbackRecieveLidar(&CLidarDevice::OnReceiveOS1_64, this);
			lidar->RegisterCallbackRecieveStatus(&CLidarDevice::OnReceiveStatusOuster, this);
			if (lidar->Create(lidarPort, bindPort, 7503, lidarIp, bindIp))  // NOLINT(readability-suspicious-call-argument)
			{
				m_hLidar = std::make_unique<Routine::handle_t>(lidar);
				m_lidarCode = lidarCode;
				ret = true;
			}
		}

		else if (lidarCode == Device::LIDAR_OUSTER_OS2_64)
		{
			auto lidar = new OS2_64::COS2_64();
			lidar->RegisterCallbackRecieveLidar(&CLidarDevice::OnReceiveOS2_64, this);
			lidar->RegisterCallbackRecieveStatus(&CLidarDevice::OnReceiveStatusOuster, this);

			if (lidar->Create(lidarPort, bindPort, 7503, lidarIp, bindIp))  // NOLINT(readability-suspicious-call-argument)
			{
				m_hLidar = std::make_unique<Routine::handle_t>(lidar);
				m_lidarCode = lidarCode;
				ret = true;
			}
		}
		else
		{
		}

		return ret;
	}


	void CLidarDevice::OnReceiveOS1_16(const Device::OS1_16::StLidarData* packet)
	{
		constexpr float_t div90112 = 1.0f / 90112;
		LidarPacket lidarPacket;
		lidarPacket.lidarCode = Device::LIDAR_OUSTER_16;
		lidarPacket.timestamp = static_cast<double_t>(packet->azimuthBlock[0].timestamp) * 0.000000001;
		for (const auto& azimuthBlock : packet->azimuthBlock)
		{
			for (uint32_t iLayer = 0; iLayer < 16; iLayer++)
			{
				if (azimuthBlock.dataBlock[iLayer].range == 0) continue;

				float_t range = static_cast<float_t>(azimuthBlock.dataBlock[iLayer].range) * 0.001f;
				float_t azimuth = 360.0f * static_cast<float_t>(azimuthBlock.encoderCount) * div90112 + GetAzimuthAngles()[iLayer];
				float_t altitude = GetAltitudeAngles()[iLayer];
				const auto intensity = static_cast<float_t>(azimuthBlock.dataBlock[iLayer].reflectivity);
				azimuth = 180 - azimuth;
				if (azimuth < 0) azimuth += 360;
				lidarPacket.lidarSignals.emplace_back(range, azimuth, altitude, intensity);
			}
		}

		m_callbackLidarPacket(m_lidarId, lidarPacket);

		double_t maxDistance = 0;
		for (const auto& signal : lidarPacket.lidarSignals)
		{
			if (const auto range = static_cast<double_t>(signal.range_m); maxDistance < range)
			{
				maxDistance = range;
			}
		}
		UpdateMaxDistance(maxDistance);
		UpdateFrameCount();
		UpdateTimestamp(lidarPacket.timestamp);
	}

	void CLidarDevice::OnReceiveOS1_32(const Device::OS1_32::StLidarData* packet)
	{
		constexpr float_t div90112 = 1.0f / 90112;
		LidarPacket lidarPacket;
		lidarPacket.lidarCode = Device::LIDAR_OUSTER_32;
		lidarPacket.timestamp = static_cast<double_t>(packet->azimuthBlock[0].timestamp) * 0.000000001;
		for (const auto& azimuthBlock : packet->azimuthBlock)
		{
			for (uint32_t iLayer = 0; iLayer < 32; iLayer++)
			{
				if (azimuthBlock.dataBlock[iLayer].range == 0) continue;

				float_t range = static_cast<float_t>(azimuthBlock.dataBlock[iLayer].range) * 0.001f;
				float_t azimuth = 360.0f * static_cast<float_t>(azimuthBlock.encoderCount) * div90112 + GetAzimuthAngles()[iLayer];
				float_t altitude = GetAltitudeAngles()[iLayer];
				auto intensity = static_cast<float_t>(azimuthBlock.dataBlock[iLayer].reflectivity);
				azimuth = 180 - azimuth;
				if (azimuth < 0) azimuth += 360;
				lidarPacket.lidarSignals.emplace_back(range, azimuth, altitude, intensity);
			}
		}

		m_callbackLidarPacket(m_lidarId, lidarPacket);

		double_t maxDistance = 0;
		for (const auto& signal : lidarPacket.lidarSignals)
		{
			if (const auto range = static_cast<double_t>(signal.range_m); maxDistance < range)
			{
				maxDistance = range;
			}
		}
		UpdateMaxDistance(maxDistance);
		UpdateFrameCount();
		UpdateTimestamp(lidarPacket.timestamp);
	}

	void CLidarDevice::OnReceiveOS1_32_rev7(const Device::OS1_32_rev7::StLidarData* packet)
	{
		LidarPacket lidarPacket;
		lidarPacket.lidarCode = Device::LIDAR_OUSTER_32_rev7;
		lidarPacket.timestamp = static_cast<double_t>(packet->columnBlock[0].columnHeaderBlock.timestamp) * 0.000000001;
		for (const auto& azimuthBlock : packet->columnBlock)
		{
			for (uint32_t iLayer = 0; iLayer < 32; iLayer++)
			{
				if (azimuthBlock.columnDataBlock[iLayer].range == 0) continue;

				float_t range = static_cast<float_t>(azimuthBlock.columnDataBlock[iLayer].range) * 0.001f;
				float_t azimuth = Device::OS1_32_rev7::GetAzimuthAngleDegrees(azimuthBlock.columnHeaderBlock.measurementID) + GetAzimuthAngles()[iLayer];
				float_t altitude = GetAltitudeAngles()[iLayer];
				auto intensity = static_cast<float_t>(azimuthBlock.columnDataBlock[iLayer].reflectivity);
				azimuth = 180 - azimuth;
				if (azimuth < 0) azimuth += 360;
				lidarPacket.lidarSignals.emplace_back(range, azimuth, altitude, intensity);
			}
		}

		m_callbackLidarPacket(m_lidarId, lidarPacket);

		double_t maxDistance = 0;
		for (const auto& signal : lidarPacket.lidarSignals)
		{
			if (const auto range = static_cast<double_t>(signal.range_m); maxDistance < range)
			{
				maxDistance = range;
			}
		}
		UpdateMaxDistance(maxDistance);
		UpdateFrameCount();
		UpdateTimestamp(lidarPacket.timestamp);
	}

	void CLidarDevice::OnReceiveOS1_64(const Device::OS1_64::StLidarData* packet)
	{
		constexpr float_t div90112 = 1.0f / 90112;
		LidarPacket lidarPacket;
		lidarPacket.lidarCode = Device::LIDAR_OUSTER_64;
		lidarPacket.timestamp = static_cast<double_t>(packet->azimuthBlock[0].timestamp) * 0.000000001;
		for (const auto& azimuthBlock : packet->azimuthBlock)
		{
			for(uint32_t iLayer = 0; iLayer < 64; iLayer++)
			{
				if (azimuthBlock.dataBlock[iLayer].range == 0) continue;

				float_t range = static_cast<float_t>(azimuthBlock.dataBlock[iLayer].range) * 0.001f;
				float_t azimuth = 360.0f * static_cast<float_t>(azimuthBlock.encoderCount) * div90112 + GetAzimuthAngles()[iLayer];
				float_t altitude = GetAltitudeAngles()[iLayer];
				const auto intensity = static_cast<float_t>(azimuthBlock.dataBlock[iLayer].reflectivity);
				azimuth = 180 - azimuth;
				if (azimuth < 0) azimuth += 360;
				lidarPacket.lidarSignals.emplace_back(range, azimuth, altitude, intensity);
			}
		}

		m_callbackLidarPacket(m_lidarId, lidarPacket);

		double_t maxDistance = 0;
		for (const auto& signal : lidarPacket.lidarSignals)
		{
			if (const auto range = static_cast<double_t>(signal.range_m); maxDistance < range)
			{
				maxDistance = range;
			}
		}
		UpdateMaxDistance(maxDistance);
		UpdateFrameCount();
		UpdateTimestamp(lidarPacket.timestamp);
	}

	void CLidarDevice::OnReceiveOS2_64(const Device::OS2_64::StLidarData* packet)
	{
		LidarPacket lidarPacket;
		lidarPacket.lidarCode = Device::LIDAR_OUSTER_OS2_64;
		lidarPacket.timestamp = static_cast<double_t>(
			packet->columnBlock[0].columnHeaderBlock.timestamp) * 1e-9;

		for (const auto& col : packet->columnBlock)
		{
			// OS2: measurementID 기반 (ScanWidth 2048 가정; OS2_64::GetAzimuthAngleDegrees 사용)
			const float_t baseAz =
				Device::OS2_64::GetAzimuthAngleDegrees(col.columnHeaderBlock.measurementID);

			for (uint32_t iLayer = 0; iLayer < 64; ++iLayer)
			{
				const auto& px = col.columnDataBlock[iLayer];
				if (px.range == 0) continue;

				// OS2 RNG19는 mm → m
				float_t range = static_cast<float_t>(px.range) * 0.001f;
				float_t azimuth = baseAz + GetAzimuthAngles()[iLayer];
				float_t altitude = GetAltitudeAngles()[iLayer];
				float_t intensity = static_cast<float_t>(px.reflectivity);

				azimuth = 180.0f - azimuth;
				if (azimuth < 0) azimuth += 360.0f;

				lidarPacket.lidarSignals.emplace_back(range, azimuth, altitude, intensity);
			}
		}

		m_callbackLidarPacket(m_lidarId, lidarPacket);

		// 공통 후처리
		double_t maxDistance = 0;
		for (const auto& s : lidarPacket.lidarSignals)
			if (maxDistance < s.range_m) maxDistance = s.range_m;

		UpdateMaxDistance(maxDistance);
		UpdateFrameCount();
		UpdateTimestamp(lidarPacket.timestamp);
	}

	void CLidarDevice::OnReceiveStatusOuster(const std::string& status)
	{
		UpdateSensorInfo(status);
	}
}
