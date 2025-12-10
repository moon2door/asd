#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <Utility/mathematics.h>
#include "WorkXYZ.h"
#include <pcl/filters/radius_outlier_removal.h>

namespace SHI
{
	namespace WorkXYZ
	{
		CWorkXYZ::CWorkXYZ()
		{
			m_intrinsicMatrix.get() = Routine::CMatrix44f::Identity();
			m_extrinsicMatrix.get() = Routine::CMatrix44f::Identity();

			CreateAsync("workXYZ", &CWorkXYZ::ProcessWorkXYZ, this);

			auto &instanceLog = Object::CLogDynamic::Instance();
			instanceLog.Initialize("./log", "log", 409600);
		}

		void CWorkXYZ::StartWorkXYZ(const std::shared_ptr<std::deque<Pala720Packet>>& packet)
		{
			m_inputPacket.Lock();
			m_inputPacket.get() = packet;
			m_inputPacket.UnLock();

			Work();
		}

		PointCloudPtr CWorkXYZ::GetOutput()
		{
			m_outputCloud.Lock();
			auto outputCloud = m_outputCloud.get();
			m_outputCloud.UnLock();

			return outputCloud;
		}

		// ReSharper disable once CppMemberFunctionMayBeStatic
		void CWorkXYZ::SetMaxLogFiles(size_t maxFiles)
		{
			auto& instanceLog = Object::CLogDynamic::Instance();
			instanceLog.SetMaxFiles(maxFiles);
		}

		void CWorkXYZ::SetIntrinsicMatrix(const Routine::CMatrix44f& matrix)
		{
			m_intrinsicMatrix.Lock();
			m_intrinsicMatrix.get() = matrix;
			m_intrinsicMatrix.UnLock();
		}

		void CWorkXYZ::SetExtrinsicMatrix(const Routine::CMatrix44f& matrix)
		{
			m_extrinsicMatrix.Lock();
			m_extrinsicMatrix.get() = matrix;
			m_extrinsicMatrix.UnLock();
		}

		void CWorkXYZ::ProcessWorkXYZ()
		{
			m_inputPacket.Lock();
			const auto packetBuffer = m_inputPacket.get();
			m_inputPacket.UnLock();

			m_intrinsicMatrix.Lock();
			const auto &intrinsicMatrix = m_intrinsicMatrix.get();
			m_intrinsicMatrix.UnLock();

			m_extrinsicMatrix.Lock();
			const auto &extrinsicMatrix = m_extrinsicMatrix.get();
			m_extrinsicMatrix.UnLock();

			ProcessWriteLog(packetBuffer);

			PointCloudPtr pointCloud;
			ProcessConvertXYZ(pointCloud, packetBuffer, intrinsicMatrix);

			PointCloudPtr pointCloudFiltered;
			ProcessFilterXYZ(pointCloudFiltered, pointCloud, m_filterDistance);  // NOLINT(readability-suspicious-call-argument)

			PointCloudPtr pointCloudExtrinsic;
			ProcessExtrinsicXYZ(pointCloudExtrinsic, pointCloudFiltered, extrinsicMatrix);

			m_outputCloud.Lock();
			m_outputCloud.get() = pointCloudExtrinsic;
			m_outputCloud.UnLock();
		}

		void CWorkXYZ::ProcessConvertXYZ(PointCloudPtr &outCloud, const PacketBufferPtr& packetBuffer, const Routine::CMatrix44f& intrinsic)
		{
			outCloud = std::make_shared<Routine::CPointCloud<>>();
			outCloud->reserve(500000);

			if (!packetBuffer || packetBuffer->empty())
			{
				return;
			}

			std::vector<const Pala720Packet*> packetPtrs;
			packetPtrs.reserve(packetBuffer->size());
			for (const auto& packet : *packetBuffer)
			{
				packetPtrs.push_back(&packet);
			}

			const size_t packetCount = packetPtrs.size();
			if (packetCount == 0)
			{
				return;
			}

			const bool enableCorrection = ShouldEnableCorrection(packetBuffer);
			const bool gateFirstSample = enableCorrection && packetCount > 1;

			const auto wrapAngle = [](float_t angleDeg)
			{
				float_t wrapped = std::fmod(angleDeg, 360.0f);
				if (wrapped < 0.0f) wrapped += 360.0f;
				return wrapped;
			};

			std::vector<float_t> correctedAngles;
			if (enableCorrection)
			{
				const float_t rpmLower = std::max(0.0f, m_targetRpm - m_rpmTolerance);
				const float_t rpmUpper = m_targetRpm + m_rpmTolerance;
				const float_t degPerSecLower = rpmLower * 6.0f;
				const float_t degPerSecUpper = rpmUpper * 6.0f;
				const float_t degPerSecTarget = m_targetRpm * 6.0f;
				correctedAngles.assign(packetCount, 0.0f);
				std::vector<double> times(packetCount, 0.0);
				std::vector<float_t> unwrapped(packetCount, 0.0f);
				std::vector<bool> valid(packetCount, false);

				float_t revolutionOffset = 0.0f;
				float_t prevRawAngle = std::numeric_limits<float_t>::quiet_NaN();
				const auto unwrapAngle = [&](float_t rawAngle)
				{
					if (!std::isnan(prevRawAngle))
					{
						float_t diff = rawAngle - prevRawAngle;
						if (diff > 180.0f)
						{
							revolutionOffset -= 360.0f;
						}
						else if (diff < -180.0f)
						{
							revolutionOffset += 360.0f;
						}
					}
					prevRawAngle = rawAngle;
					return rawAngle + revolutionOffset;
				};

				for (size_t idx = 0; idx < packetCount; ++idx)
				{
					const auto& rotor = packetPtrs[idx]->rotorPacket;
					times[idx] = rotor.time;
					unwrapped[idx] = unwrapAngle(static_cast<float_t>(rotor.angle));
				}

				valid[0] = true;
				correctedAngles[0] = unwrapped[0];
				size_t lastValidIdx = 0;
				for (size_t idx = 1; idx < packetCount; ++idx)
				{
					if (lastValidIdx >= packetCount)
					{
						break;
					}

					const double deltaTime = times[idx] - times[lastValidIdx];
					if (deltaTime <= 0.0)
					{
						continue;
					}

					const float_t rate = static_cast<float_t>((unwrapped[idx] - correctedAngles[lastValidIdx]) / deltaTime);
					if (rate >= degPerSecLower && rate <= degPerSecUpper)
					{
						valid[idx] = true;
						correctedAngles[idx] = unwrapped[idx];
						lastValidIdx = idx;
					}
				}

				size_t idx = 0;
				size_t prevValidIdx = std::numeric_limits<size_t>::max();
				while (idx < packetCount)
				{
					if (valid[idx])
					{
						prevValidIdx = idx;
						++idx;
						continue;
					}

					const size_t left = prevValidIdx;
					size_t right = idx;
					while (right < packetCount && !valid[right])
					{
						++right;
					}

					if (left == std::numeric_limits<size_t>::max())
					{
						if (right >= packetCount)
						{
							break;
						}

						const double tEnd = times[right];
						const float_t angleEnd = correctedAngles[right];
						for (size_t k = 0; k < right; ++k)
						{
							const double delta = tEnd - times[k];
							correctedAngles[k] = angleEnd - degPerSecTarget * static_cast<float_t>(delta);
							valid[k] = true;
						}
						prevValidIdx = right;
						idx = right;
						continue;
					}

					if (right >= packetCount)
					{
						const double tStart = times[left];
						const float_t angleStart = correctedAngles[left];
						for (size_t k = idx; k < packetCount; ++k)
						{
							const double delta = times[k] - tStart;
							correctedAngles[k] = angleStart + degPerSecTarget * static_cast<float_t>(delta);
							valid[k] = true;
						}
						break;
					}

					const double tStart = times[left];
					const double tEnd = times[right];
					const double duration = tEnd - tStart;
					const float_t angleStart = correctedAngles[left];
					const float_t angleEnd = correctedAngles[right];

					for (size_t k = idx; k < right; ++k)
					{
						float_t interpolated;
						if (duration > 0.0)
						{
							const double alpha = (times[k] - tStart) / duration;
							interpolated = angleStart + static_cast<float_t>(alpha) * (angleEnd - angleStart);
						}
						else
						{
							const double delta = times[k] - tStart;
							interpolated = angleStart + degPerSecTarget * static_cast<float_t>(delta);
						}
						correctedAngles[k] = interpolated;
						valid[k] = true;
					}

					idx = right;
				}
			}

			for (size_t packetIndex = 0; packetIndex < packetCount; ++packetIndex)
			{
				if (gateFirstSample && packetIndex == 0)
				{
					continue;
				}

				const auto* packet = packetPtrs[packetIndex];
				const auto& lidar = packet->lidarPacket;
				float_t rotorAngle = static_cast<float_t>(packet->rotorPacket.angle);
				if (enableCorrection)
				{
					rotorAngle = wrapAngle(correctedAngles[packetIndex]);
				}

				for (const auto& laser : lidar.lidarSignals)
				{
					// Apply scan-start based azimuth rotation
					float_t azimuthAdj = laser.azimuth_deg - m_azimuthOffsetDeg;
					if (azimuthAdj < 0) azimuthAdj += 360.0f;
					const float_t cosTheta = cos(azimuthAdj * Math::D2Rf);
					const float_t sinTheta = sin(azimuthAdj * Math::D2Rf);
					const float_t cosPhi = cos(laser.altitude_deg * Math::D2Rf);
					const float_t sinPhi = sin(laser.altitude_deg * Math::D2Rf);

					const float_t x = laser.range_m * sinTheta * cosPhi;
					const float_t y = laser.range_m * cosTheta * cosPhi;
					const float_t z = laser.range_m * sinPhi;

					const float_t xBase = intrinsic(0, 0) * x + intrinsic(0, 1) * (-z) + intrinsic(0, 2) * y + intrinsic(0, 3);
					const float_t yBase = intrinsic(1, 0) * x + intrinsic(1, 1) * (-z) + intrinsic(1, 2) * y + intrinsic(1, 3);
					const float_t zBase = intrinsic(2, 0) * x + intrinsic(2, 1) * (-z) + intrinsic(2, 2) * y + intrinsic(2, 3);

					const float_t cosRotor = cos(rotorAngle * Math::D2Rf);
					const float_t sinRotor = sin(rotorAngle * Math::D2Rf);
					const float_t xRotor = cosRotor * xBase - sinRotor * yBase;
					const float_t yRotor = sinRotor * xBase + cosRotor * yBase;
					const float_t zRotor = zBase;
					outCloud->push_back(Routine::CPoint3Df(-xRotor, yRotor, zRotor));
				}
			}
		}

		void ProcessFilterXYZ(PointCloudPtr &outCloud, const PointCloudPtr& cloud, float_t distance)
		{
			const float_t distanceSq = distance * distance;
			const auto pclCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			const auto pclCloudFar = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			pclCloud->reserve(cloud->size());
			pclCloudFar->reserve(cloud->size());

			uint32_t count = 0;
			for (const auto &p : cloud->GetPoints())
			{
				if (const float norm = p.squaredNorm(); norm < distanceSq)
				{
					if (count >= 10)
					{
						pclCloud->emplace_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
						count = 0;
					}
				}
				else
				{
					pclCloudFar->emplace_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
				}
			}

			if(!pclCloudFar->empty())
			{
				pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
				filter.setMinNeighborsInRadius(1);
				filter.setRadiusSearch(1);
				filter.setInputCloud(pclCloudFar);
				filter.filter(*pclCloudFar);
			}

			outCloud = std::make_shared<Routine::CPointCloud<>>();
			outCloud->reserve(pclCloudFar->size() + pclCloud->size());
			for (const auto& p : pclCloudFar->points)
			{
				outCloud->GetPoints().emplace_back(p.x, p.y, p.z);
			}
			for (const auto& p : pclCloud->points)
			{
				outCloud->GetPoints().emplace_back(p.x, p.y, p.z);
			}
		}		
		
		void ProcessExtrinsicXYZ(PointCloudPtr& outCloud, const PointCloudPtr& cloud, const Routine::CMatrix44f& extrinsic)
		{
			outCloud = std::make_shared<Routine::CPointCloud<>>();
			outCloud->reserve(cloud->size());
			for (const auto& pt : cloud->GetPoints())
			{
				float_t x = extrinsic(0, 0) * pt.coeffRef(0) + extrinsic(0, 1) * pt.coeffRef(1) + extrinsic(0, 2) * pt.coeffRef(2) + extrinsic(0, 3);
				float_t y = extrinsic(1, 0) * pt.coeffRef(0) + extrinsic(1, 1) * pt.coeffRef(1) + extrinsic(1, 2) * pt.coeffRef(2) + extrinsic(1, 3);
				float_t z = extrinsic(2, 0) * pt.coeffRef(0) + extrinsic(2, 1) * pt.coeffRef(1) + extrinsic(2, 2) * pt.coeffRef(2) + extrinsic(2, 3);
				outCloud->GetPoints().emplace_back(x, y, z);
			}
		}

		void ProcessWriteLog(const PacketBufferPtr& packetBuffer)
		{
			auto& instanceLog = Object::CLogDynamic::Instance();

			for (auto& packet : *packetBuffer)
			{
				Routine::CSerializer serializer;
				serializer << packet;
				instanceLog.Write(serializer.data(), serializer.size());
			}
			instanceLog.Write(nullptr, 0);
		}

		bool CWorkXYZ::ShouldEnableCorrection(const PacketBufferPtr& packetBuffer) const
		{
			if (!packetBuffer || packetBuffer->empty())
			{
				return false;
			}

			if (!IsRotorCommandActive())
			{
				return false;
			}

			const float_t estimatedRpm = EstimateRotorRpm(packetBuffer);
			if (estimatedRpm <= 0.0f)
			{
				return true;
			}

			return IsRpmWithinTolerance(estimatedRpm);
		}

		float_t CWorkXYZ::EstimateRotorRpm(const PacketBufferPtr& packetBuffer) const
		{
			if (!packetBuffer || packetBuffer->size() < 2)
			{
				return 0.0f;
			}

			std::vector<float_t> rpmSamples;
			rpmSamples.reserve(packetBuffer->size());

			double prevTime = 0.0;
			float_t prevAngle = 0.0f;
			bool hasPrev = false;

			for (const auto& packet : *packetBuffer)
			{
				const auto& rotor = packet.rotorPacket;
				const double currentTime = rotor.time;
				const float_t currentAngle = static_cast<float_t>(rotor.angle);

				if (!hasPrev)
				{
					prevTime = currentTime;
					prevAngle = currentAngle;
					hasPrev = true;
					continue;
				}

				const double deltaTime = currentTime - prevTime;
				if (deltaTime <= 0.0)
				{
					prevTime = currentTime;
					prevAngle = currentAngle;
					continue;
				}

				float_t deltaAngle = currentAngle - prevAngle;
				while (deltaAngle < 0.0f)
				{
					deltaAngle += 360.0f;
				}

				if (deltaAngle <= 0.0f)
				{
					prevTime = currentTime;
					prevAngle = currentAngle;
					continue;
				}

				const float_t rpm = deltaAngle / static_cast<float_t>(6.0 * deltaTime);
				rpmSamples.push_back(rpm);

				prevTime = currentTime;
				prevAngle = currentAngle;
			}

			if (rpmSamples.empty())
			{
				return 0.0f;
			}

			const size_t mid = rpmSamples.size() / 2;
			std::nth_element(rpmSamples.begin(), rpmSamples.begin() + mid, rpmSamples.end());
			if (rpmSamples.size() % 2 == 1)
			{
				return rpmSamples[mid];
			}

			const float_t lower = *std::max_element(rpmSamples.begin(), rpmSamples.begin() + mid);
			return (lower + rpmSamples[mid]) * 0.5f;
		}

		bool CWorkXYZ::IsRpmWithinTolerance(float_t rpm) const
		{
			return std::fabs(rpm - m_targetRpm) <= m_rpmTolerance;
		}

		bool CWorkXYZ::IsRotorCommandActive() const
		{
			if (!m_rotorCommandStateProvider)
			{
				return true;
			}
			return m_rotorCommandStateProvider();
		}
	}
}
