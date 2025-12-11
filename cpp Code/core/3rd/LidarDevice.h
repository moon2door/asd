#pragma once
#include <Routine/include/Base/RoutineCallback.h>
#include "../os1_16/OS1_16.h"
#include "../os1_32/OS1_32.h"
#include "../os1_64/OS1_64.h"
#include "../os1_32_rev7/OS1_32_rev7.h"
#include "../os2_64/OS2_64.h"
#include "LidarStatus.h"
#include "LidarPacket.h"

namespace Device
{
	class CLidarDevice
		: public CLidarStatus
	{
	public:
		CLidarDevice();

		bool CreateLidarDevice(uint8_t lidarId, uint8_t lidarCode, const std::string& bindIp, uint16_t bindPort, const std::string& lidarIp, uint16_t lidarPort);
				
		EnableCallback(LidarPacket, uint8_t id, const LidarPacket&)

	private:
		uint8_t m_lidarCode = LIDAR_UNKNOWN;
		uint8_t m_lidarId = 0;
		std::unique_ptr<Routine::handle_t> m_hLidar;

		void OnReceiveOS1_16(const Device::OS1_16::StLidarData* packet);

		void OnReceiveOS1_32(const Device::OS1_32::StLidarData* packet);

		void OnReceiveOS1_32_rev7(const Device::OS1_32_rev7::StLidarData* packet);

		void OnReceiveOS1_64(const Device::OS1_64::StLidarData* packet);

		void OnReceiveStatusOuster(const std::string& status);

		void OnReceiveOS2_64(const Device::OS2_64::StLidarData* packet);
	};

}