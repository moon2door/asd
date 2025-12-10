
#include "Compressor.h"
#include <Utility/types.h>
#include <pcl/io/pcd_io.h>

SHI::Compressor::CCompressDistance::CCompressDistance()
	: m_pointEncoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(pcl::io::MANUAL_CONFIGURATION, false, 0.25, 0.25, true, 0, false, 4))
	, m_hookEncoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(pcl::io::MANUAL_CONFIGURATION, false, 0.25, 0.25, true, 0, false, 4))
	, m_groundEncoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(pcl::io::MANUAL_CONFIGURATION, false, 0.25, 0.25, true, 0, false, 4))
	, m_PointCloudDecoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>())
	, m_PointCloudHookDecoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>())
	, m_PointCloudGroundDecoder(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>())
{
	//m_compressor.Create(sizeof(SHI::Data::StDistanceSocket), 9);
	//m_uncompressor.Create(sizeof(SHI::Data::StDistanceSocket));
}

SHI::Compressor::CCompressDistance::~CCompressDistance()
{
	delete m_pointEncoder;
	delete m_hookEncoder;
	delete m_groundEncoder;
	delete m_PointCloudDecoder;
	delete m_PointCloudHookDecoder;
	delete m_PointCloudGroundDecoder;
	//m_compressor.Destroy();
	//m_uncompressor.Destroy();
}

bool SHI::Compressor::CCompressDistance::Encode(SHI::Data::StDistanceSocket& dst, SHI::Data::StDistanceSocket& src)
{
	bool bAllocXYZ = false;
	bool bAllocClusterInfo = false;
	bool bAllocClusterIndices = false;
	bool bAllocLabel = false;
	bool bAllocDistanceInfo = false;
	bool bAllocDistanceLabel = false;
	bool bAllocCollisionInfo = false;
	bool bAllocReason = false;

	m_csProcess.Lock();
	dst.InitBuffer();

	dst.timeStamp = src.timeStamp;
	dst.elapsedTimeClustering = src.elapsedTimeClustering;
	dst.elapsedTimeDistance = src.elapsedTimeDistance;
	dst.elapsedTimeCollision = src.elapsedTimeCollision;
	dst.attitude = src.attitude;
	dst.collisionTotal = src.collisionTotal;
	dst.gpsLat = 0;
	dst.gpsLon = 0;

	// Label인 점의 index list
	std::vector<unsigned int> iPointsObject, iPointsHook, iPointsGround;
	iPointsObject.reserve(src.GetXYZSize());
	iPointsHook.reserve(src.GetXYZSize());
	iPointsGround.reserve(src.GetXYZSize());
	for (unsigned int iCluster = 0; iCluster < src.GetClusterInfoSize(); iCluster++)
	{
		if (src.GetLabel()[iCluster] == SHI::LABEL_OBJECT)
		{
			for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
			{
				unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
				iPointsObject.push_back(idx);
			}
		}

		if (src.GetLabel()[iCluster] == SHI::LABEL_HOOK)
		{
			for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
			{
				unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
				iPointsHook.push_back(idx);
			}
		}
		if (src.GetLabel()[iCluster] == SHI::LABEL_GROUND)
		{
			for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
			{
				unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
				iPointsGround.push_back(idx);
			}
		}
	}

	// Copy Point Cloud
	bAllocXYZ = dst.AllocXYZ(iPointsObject.size() + iPointsHook.size() + iPointsGround.size());
	bAllocClusterInfo = dst.AllocClusterInfo(2);
	bAllocClusterIndices = dst.AllocClusterIndices(iPointsHook.size() + iPointsGround.size());
	bAllocLabel = dst.AllocLabel(2);

	if (bAllocXYZ && bAllocClusterInfo && bAllocClusterIndices && bAllocLabel)
	{
		for (unsigned int i = 0; i < iPointsObject.size(); i++)
		{
			dst.GetXYZ()[i].X = src.GetXYZ()[iPointsObject[i]].X;
			dst.GetXYZ()[i].Y = src.GetXYZ()[iPointsObject[i]].Y;
			dst.GetXYZ()[i].Z = src.GetXYZ()[iPointsObject[i]].Z;
		}
		for (unsigned int i = 0; i < iPointsHook.size(); i++)
		{
			unsigned int offset = iPointsObject.size();
			dst.GetXYZ()[offset + i].X = src.GetXYZ()[iPointsHook[i]].X;
			dst.GetXYZ()[offset + i].Y = src.GetXYZ()[iPointsHook[i]].Y;
			dst.GetXYZ()[offset + i].Z = src.GetXYZ()[iPointsHook[i]].Z;
			dst.GetClusterIndices()[i] = offset + i;
		}
		for (unsigned int i = 0; i < iPointsGround.size(); i++)
		{
			unsigned int offset = iPointsHook.size() + iPointsObject.size();
			dst.GetXYZ()[offset + i].X = src.GetXYZ()[iPointsGround[i]].X;
			dst.GetXYZ()[offset + i].Y = src.GetXYZ()[iPointsGround[i]].Y;
			dst.GetXYZ()[offset + i].Z = src.GetXYZ()[iPointsGround[i]].Z;
			dst.GetClusterIndices()[i] = offset + i;
		}

		dst.GetClusterInfo()[0].Index = 0;
		dst.GetClusterInfo()[0].Size = iPointsHook.size();
		dst.GetLabel()[0] = SHI::LABEL_HOOK;

		dst.GetClusterInfo()[1].Index = iPointsHook.size();
		dst.GetClusterInfo()[1].Size = iPointsGround.size();
		dst.GetLabel()[1] = SHI::LABEL_GROUND;

		// Decode Distance Info

		bAllocDistanceInfo = dst.AllocDistanceInfo(src.GetDistanceInfoSize());
		if (bAllocDistanceInfo) memcpy(dst.GetDistanceInfo(), src.GetDistanceInfo(), src.GetDistanceInfoSize() * sizeof(SHI::Data::StDistanceInfo));

		// Decode DistanceLabel
		bAllocDistanceLabel = dst.AllocDistanceLabel(src.GetDistanceLabelSize());
		if (bAllocDistanceLabel) memcpy(dst.GetDistanceLabel(), src.GetDistanceLabel(), src.GetDistanceLabelSize());

		// Decode Collision Info
		bAllocCollisionInfo = dst.AllocCollisionInfo(src.GetCollisionInfoSize());
		if (bAllocCollisionInfo) memcpy(dst.GetCollisionInfo(), src.GetCollisionInfo(), src.GetCollisionInfoSize());

		// Encode Reason Info
		bAllocReason = dst.AllocReasonInfo(src.GetReasonInfoSize());
		if (bAllocReason) memcpy(dst.GetReasonInfo(), src.GetReasonInfo(), src.GetReasonInfoSize());
	}
	m_csProcess.UnLock();
	return bAllocXYZ && bAllocClusterInfo && bAllocClusterIndices && bAllocLabel && bAllocDistanceInfo && bAllocDistanceLabel && bAllocCollisionInfo && bAllocReason;
}

bool SHI::Compressor::CCompressDistance::Encode(SHI::Data::StDistanceSocketCompressed& dst, SHI::Data::StDistanceSocket& src)
{
	bool bAllocXYZ = false;
	bool bAllocHook = false;
	bool bAllocGround = false;
	bool bAllocDistanceInfo = false;
	bool bAllocCollisionInfo = false;
	bool bAllocDistanceLabel = false;
	bool bAllicReason = false;
	m_csProcess.Lock();
	try
	{
		dst.InitBuffer();
		// 무압축 데이터 복사
		dst.timeStamp = src.timeStamp;
		dst.elapsedTimeClustering = src.elapsedTimeClustering;
		dst.elapsedTimeDistance = src.elapsedTimeDistance;
		dst.elapsedTimeCollision = src.elapsedTimeCollision;
		dst.attitude = src.attitude;
		dst.collisionTotal = src.collisionTotal;

		// Label인 점의 index list
		std::vector<unsigned int> iPointsObject, iPointsHook, iPointsGround;
		iPointsObject.reserve(src.GetXYZSize());
		iPointsHook.reserve(src.GetXYZSize());
		iPointsGround.reserve(src.GetXYZSize());
		std::vector<bool> labeled;
		labeled.resize(src.GetXYZSize());
		for (unsigned int iPoint = 0; iPoint < src.GetXYZSize(); iPoint++)
		{
			labeled[iPoint] = false;
		}

		for (unsigned int iCluster = 0; iCluster < src.GetClusterInfoSize(); iCluster++)
		{
			if (src.GetLabel()[iCluster] == SHI::LABEL_OBJECT)
			{
				for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
				{
					unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
					iPointsObject.push_back(idx);

					labeled[idx] = true;
				}
			}

			if (src.GetLabel()[iCluster] == SHI::LABEL_HOOK)
			{
				for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
				{
					unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
					iPointsHook.push_back(idx);

					labeled[idx] = true;
				}
			}
			if (src.GetLabel()[iCluster] == SHI::LABEL_GROUND)
			{
				for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
				{
					unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
					iPointsGround.push_back(idx);

					labeled[idx] = true;
				}
			}

			if (src.GetLabel()[iCluster] == SHI::LABEL_BODY)
			{
				for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
				{
					unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
					labeled[idx] = true;
				}
			}
			if (src.GetLabel()[iCluster] == SHI::LABEL_EXCEPTION)
			{
				for (unsigned int iPoint = 0; iPoint < src.GetClusterInfo()[iCluster].Size; iPoint++)
				{
					unsigned int idx = src.GetClusterIndices()[iPoint + src.GetClusterInfo()[iCluster].Index];
					labeled[idx] = true;
				}
			}

		}

		for (unsigned int iPoint = 0; iPoint < src.GetXYZSize(); iPoint++)
		{
			if (labeled[iPoint] == false)
			{
				iPointsObject.push_back(iPoint);
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->reserve(iPointsObject.size());
		for (unsigned int i = 0; i < iPointsObject.size(); i++)
		{
			unsigned int idx = iPointsObject[i];
			cloud->push_back(pcl::PointXYZ(src.GetXYZ()[idx].X * 0.01f, src.GetXYZ()[idx].Y * 0.01f, src.GetXYZ()[idx].Z * 0.01f));
		}

		// 후크 포인트
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHook(new pcl::PointCloud<pcl::PointXYZ>);
		cloudHook->reserve(iPointsHook.size());
		for (unsigned int i = 0; i < iPointsHook.size(); i++)
		{
			unsigned int idx = iPointsHook[i];
			cloudHook->push_back(pcl::PointXYZ(src.GetXYZ()[idx].X * 0.01f, src.GetXYZ()[idx].Y * 0.01f, src.GetXYZ()[idx].Z * 0.01f));
		}

		// 지면 포인트
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudGround(new pcl::PointCloud<pcl::PointXYZ>);
		cloudGround->reserve(iPointsGround.size());
		for (unsigned int i = 0; i < iPointsGround.size(); i++)
		{
			unsigned int idx = iPointsGround[i];
			cloudGround->push_back(pcl::PointXYZ(src.GetXYZ()[idx].X * 0.01f, src.GetXYZ()[idx].Y * 0.01f, src.GetXYZ()[idx].Z * 0.01f));
		}

		// Encode point cloud
		std::stringstream compressedXYZ;
		m_pointEncoder->encodePointCloud(cloud, compressedXYZ);
		bAllocXYZ = dst.AllocXYZCompressed(compressedXYZ.str().size());
		if (bAllocXYZ) memcpy(dst.GetXYZCompressed(), compressedXYZ.str().c_str(), compressedXYZ.str().size());

		// Encode hook point
		std::stringstream compressedHook;
		m_hookEncoder->encodePointCloud(cloudHook, compressedHook);
		bAllocHook = dst.AllocHookXYZCompressed(compressedHook.str().size());
		if (bAllocHook) memcpy(dst.GetHookXYZCompressed(), compressedHook.str().c_str(), compressedHook.str().size());

		// Encode ground point
		std::stringstream compressedGround;
		m_groundEncoder->encodePointCloud(cloudGround, compressedGround);
		bAllocGround = dst.AllocGroundXYZCompressed(compressedGround.str().size());
		if (bAllocGround) memcpy(dst.GetGroundXYZCompressed(), compressedGround.str().c_str(), compressedGround.str().size());

		// Encode Distance Info
		Routine::CCompressor compressDistance(src.GetDistanceInfo(), src.GetDistanceInfoSize() * sizeof(Data::StDistanceInfo));
		compressDistance.Compress();
		bAllocDistanceInfo = dst.AllocDistanceInfo(compressDistance.GetData()->size());
		if (bAllocDistanceInfo) memcpy(dst.GetDistanceInfo(), compressDistance.GetData()->data(), compressDistance.GetData()->size());

		// Encode DistanceLabel
		Routine::CCompressor compressDistanceLabel(src.GetDistanceLabel(), src.GetDistanceLabelSize());
		compressDistanceLabel.Compress();
		bAllocDistanceLabel = dst.AllocDistanceLabel(compressDistanceLabel.GetData()->size());
		if (bAllocDistanceLabel) memcpy(dst.GetDistanceLabel(), compressDistanceLabel.GetData()->data(), compressDistanceLabel.GetData()->size());

		// Encode Collision Info
		Routine::CCompressor compressCollisionInfo (src.GetCollisionInfo(), src.GetCollisionInfoSize());
		compressCollisionInfo.Compress();
		bAllocCollisionInfo = dst.AllocCollisionInfo(compressCollisionInfo.GetData()->size());
		if (bAllocCollisionInfo) memcpy(dst.GetCollisionInfo(), compressCollisionInfo.GetData()->data(), compressCollisionInfo.GetData()->size());

		// Encode Reason Info
		Routine::CCompressor compressReasonInfo(src.GetReasonInfo(), src.GetReasonInfoSize());
		compressReasonInfo.Compress();
		bAllicReason = dst.AllocReasonInfo(compressReasonInfo.GetData()->size());
		if (bAllicReason) memcpy(dst.GetReasonInfo(), compressReasonInfo.GetData()->data(), compressReasonInfo.GetData()->size());
	}
	catch (...)
	{
	}
	m_csProcess.UnLock();
	return (bAllocXYZ && bAllocHook && bAllocGround && bAllocDistanceInfo && bAllocCollisionInfo && bAllocDistanceLabel && bAllicReason);
}

bool SHI::Compressor::CCompressDistance::Decode(SHI::Data::StDistanceSocket& dst, SHI::Data::StDistanceSocketCompressed& src)
{
	m_csProcess.Lock();
	dst.InitBuffer();

	// 무압축 데이터 복사
	dst.timeStamp = src.timeStamp;
	dst.elapsedTimeClustering = src.elapsedTimeClustering;
	dst.elapsedTimeDistance = src.elapsedTimeDistance;
	dst.elapsedTimeCollision = src.elapsedTimeCollision;
	dst.attitude = src.attitude;
	dst.collisionTotal = src.collisionTotal;
	dst.gpsLat = 0;
	dst.gpsLon = 0;


	bool bAllocXYZ = false;
	bool bAllocClusterInfo = false;
	bool bAllocClusterIndices = false;
	bool bAllocLabel = false;
	try
	{
		//Decode point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (src.GetXYZCompressedSize())
		{

			std::stringstream compressedXYZ;
			compressedXYZ.write((char*)src.GetXYZCompressed(), src.GetXYZCompressedSize());
			m_PointCloudDecoder->decodePointCloud(compressedXYZ, cloud);
		}

		// decode cluster
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHook(new pcl::PointCloud<pcl::PointXYZ>);
		if (src.GetHookXYZCompressedSize())
		{
			std::stringstream compressedHookXYZ;
			compressedHookXYZ.write((char*)src.GetHookXYZCompressed(), src.GetHookXYZCompressedSize());
			m_PointCloudHookDecoder->decodePointCloud(compressedHookXYZ, cloudHook);
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudGround(new pcl::PointCloud<pcl::PointXYZ>);
		if (src.GetGroundXYZCompressedSize())
		{
			std::stringstream compressedGroundXYZ;
			compressedGroundXYZ.write((char*)src.GetGroundXYZCompressed(), src.GetGroundXYZCompressedSize());
			m_PointCloudGroundDecoder->decodePointCloud(compressedGroundXYZ, cloudGround);
		}

		bAllocXYZ = dst.AllocXYZ(cloud->size() + cloudHook->size() + cloudGround->size());
		bAllocClusterInfo = dst.AllocClusterInfo(2);
		bAllocClusterIndices = dst.AllocClusterIndices(cloudHook->size() + cloudGround->size());
		bAllocLabel = dst.AllocLabel(2);

		if (bAllocXYZ && bAllocClusterInfo && bAllocClusterIndices && bAllocLabel)
		{
			for (unsigned int i = 0; i < cloud->size(); i++)
			{
				dst.GetXYZ()[i].X = short(cloud->points[i].x * 100);
				dst.GetXYZ()[i].Y = short(cloud->points[i].y * 100);
				dst.GetXYZ()[i].Z = short(cloud->points[i].z * 100);
			}
			for (unsigned int i = 0; i < cloudHook->size(); i++)
			{
				unsigned int offset = cloud->size();
				dst.GetXYZ()[offset + i].X = short(cloudHook->points[i].x * 100);
				dst.GetXYZ()[offset + i].Y = short(cloudHook->points[i].y * 100);
				dst.GetXYZ()[offset + i].Z = short(cloudHook->points[i].z * 100);
				dst.GetClusterIndices()[i] = offset + i;
			}
			for (unsigned int i = 0; i < cloudGround->size(); i++)
			{
				unsigned int offset = cloud->size() + cloudHook->size();
				dst.GetXYZ()[offset + i].X = short(cloudGround->points[i].x * 100);
				dst.GetXYZ()[offset + i].Y = short(cloudGround->points[i].y * 100);
				dst.GetXYZ()[offset + i].Z = short(cloudGround->points[i].z * 100);
				dst.GetClusterIndices()[i] = offset + i;
			}

			dst.GetClusterInfo()[0].Index = 0;
			dst.GetClusterInfo()[0].Size = cloudHook->size();
			dst.GetLabel()[0] = SHI::LABEL_HOOK;

			dst.GetClusterInfo()[1].Index = cloudHook->size();
			dst.GetClusterInfo()[1].Size = cloudGround->size();
			dst.GetLabel()[1] = SHI::LABEL_GROUND;
		}
	}
	catch (...)
	{
	}


	bool bAllocDistanceInfo = false;
	bool bAllocDistanceLabel = false;
	bool bAllocCollisionInfo = false;
	bool bAllocReason = false;
	try
	{
		// Decode Distance Info
		Routine::CDeCompressor deCompressorDistanceInfo(src.GetDistanceInfo(), src.GetDistanceInfoSize());
		deCompressorDistanceInfo.DeCompress();
		bAllocDistanceInfo = dst.AllocDistanceInfo(deCompressorDistanceInfo.GetData()->size() / sizeof(SHI::Data::StDistanceInfo));
		auto aa = deCompressorDistanceInfo.GetData();
		if (bAllocDistanceInfo) memcpy(dst.GetDistanceInfo(), deCompressorDistanceInfo.GetData()->data(), deCompressorDistanceInfo.GetData()->size());

		// Decode DistanceLabel
		Routine::CDeCompressor deCompressorDistanceLabel(src.GetDistanceLabel(), src.GetDistanceLabelSize());
		deCompressorDistanceLabel.DeCompress();
		bAllocDistanceLabel = dst.AllocDistanceLabel(deCompressorDistanceLabel.GetData()->size());
		if (bAllocDistanceLabel) memcpy(dst.GetDistanceLabel(), deCompressorDistanceLabel.GetData()->data(), deCompressorDistanceLabel.GetData()->size());

		// Decode Collision Info
		Routine::CDeCompressor deCompressorCollisionInfo(src.GetCollisionInfo(), src.GetCollisionInfoSize());
		deCompressorCollisionInfo.DeCompress();
		bAllocCollisionInfo = dst.AllocCollisionInfo(deCompressorCollisionInfo.GetData()->size());
		if (bAllocCollisionInfo) memcpy(dst.GetCollisionInfo(), deCompressorCollisionInfo.GetData()->data(), deCompressorCollisionInfo.GetData()->size());

		// Encode Reason Info
		Routine::CDeCompressor deCompressorReasonInfo(src.GetReasonInfo(), src.GetReasonInfoSize());
		deCompressorReasonInfo.DeCompress();
		bAllocReason = dst.AllocReasonInfo(deCompressorReasonInfo.GetData()->size());
		if (bAllocReason) memcpy(dst.GetReasonInfo(), deCompressorReasonInfo.GetData()->data(), deCompressorReasonInfo.GetData()->size());
	}
	catch (...)
	{
	}
	m_csProcess.UnLock();
	return (bAllocXYZ && bAllocClusterInfo && bAllocClusterIndices && bAllocLabel && bAllocDistanceInfo && bAllocDistanceLabel && bAllocCollisionInfo && bAllocReason);
}