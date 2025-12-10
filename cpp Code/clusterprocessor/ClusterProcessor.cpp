
#include "ClusterProcessor.h"
#include <Config/CraneInfo.h>
#include <Routine/Include/Base/CElapseTimer.h>
#include <Routine/Include/Base/CTime.h>
#include <cstdio>
#include <sys/stat.h>
#include <algorithm>
#include <chrono>
#include <limits>

CClusterProcessor::CClusterProcessor()
{	
	// 초기화
	if (ProcessInitialize())
	{
		SHI::CraneAttitudePtr attitude = GetCraneAttitude();
		if (attitude)
		{
			int pier = attitude->pierId;
			int id = attitude->craneId;
			printf("Crane pier : %s, Type : %s \n", SHI::ConvPierDisplayStr(pier).c_str(), SHI::ConvCraneStr(pier, id).c_str());
		}
		else
		{
			printf("Crane pier : ??, Type : ?? \n");
		}
	}
	else
	{
		printf("ProcessInitialize failed. \n");
	}
}

CClusterProcessor::~CClusterProcessor()
{
}

void CClusterProcessor::OnXYZPoint(SHI::Data::StXYZPoints* pData)
{
	std::shared_ptr<SHI::Data::StCluster> dataCluster(new SHI::Data::StCluster);

	printf("On update XYZPoint : %d points\n", pData->GetXYZSize());
	Routine::CElapseTimer elapsedTotal;

	SHI::Data::StDBInfo dbinfo;
	GetProxyDBInfo()->ReadBuffer(&dbinfo);
	SetDBInfo(dbinfo);

	SHI::Data::StPlcInfo plcInfo;
	GetProxyPlcInfo()->ReadBuffer(&plcInfo);
	SetPlcInfo(plcInfo);

	// 포인트 클라우드 전처리: 포인트 클라우드로 변환 후 다운샘플
	SHI::PointCloudPtr cloudPreProcessed(new SHI::PointCloud);
	ProcessPreProcessing(*cloudPreProcessed, pData);

	// 크레인 자세 처리
	bool bAttitudeSucceed = ProcessCraneAttitude(cloudPreProcessed);
	printf(" > ProcessCraneAttitude : %s \n", bAttitudeSucceed ? "succeed" : "failed");
	SHI::CraneAttitudePtr attitude = GetCraneAttitude();
	if (attitude)
	{
		for (unsigned int i = 0; i < attitude->numPart; i++)
		{
			if (attitude->bUseEstimate[i])
			{
				printf("   - %.2f(%s, %s) \n", attitude->pose[i], attitude->bEstimateSucceed[i] ? "Succeed" : "failed", attitude->bMoving[i] ? "Moving" : "Halt");
			}
		}
	}

	// 몸체 포인트 제거 처리
	SHI::PointCloudPtr cloudFiltered(new SHI::PointCloud);
	SHI::PointCloudPtr cloudBody(new SHI::PointCloud);
	SHI::PointCloudPtr cloudExceptionRoi(new SHI::PointCloud);
	ProcessRemoveCranePoints(*cloudFiltered, *cloudBody, *cloudExceptionRoi, cloudPreProcessed);

	//  클러스터 전처리
	SHI::PointCloudPtr cloudObject(new SHI::PointCloud);
	SHI::PointCloudPtr cloudGround(new SHI::PointCloud);
	ProcessPreClustering(*cloudObject, *cloudGround, cloudFiltered);

	// 클러스터 처리
	SHI::PCLIndicesVectorPtr clusterIndices(new SHI::PCLIndicesVector);
	SHI::PointCloudPtr cloudCluster(new SHI::PointCloud);
	bool bClustered = ProcessClustering(*clusterIndices, *cloudCluster, cloudObject);

	// 클러스터 라벨링
	std::vector<unsigned char> labels;
	SHI::PCLIndicesVectorPtr clusterLabeled(new SHI::PCLIndicesVector);
	SHI::PointCloudPtr cloudLabeled(new SHI::PointCloud);
	ProcessLabelCluster(*clusterLabeled, *cloudLabeled, labels, cloudCluster, clusterIndices, cloudBody, cloudGround, cloudExceptionRoi);

	// 클러스터 데이터 후처리
	attitude = GetCraneAttitude();
	if (attitude)
	{
		ProcessPostProcessing(*dataCluster, cloudLabeled, clusterLabeled, labels, attitude);
		dataCluster->timeStamp = pData->timeStamp;
		dataCluster->elapsedTimeClustering = (unsigned int)elapsedTotal.GetElapseTime()*0.001;

		// 결과 송신
#ifdef _DEBUG
		//printf("OnXYZPoint : lat=%.2f, lon=%.2f, hgt=%.1f, azm= %.1f \n"
		//	, dbinfo.hook.gps.latitude
		//	, dbinfo.hook.gps.longitude
		//	, dbinfo.hook.gps.height
		//	, dbinfo.hook.gps.azimuth);
#endif	// _DEBUG

		GetStubDBInfo()->WriteData(&dbinfo);
		GetStubCraneAttitude()->WriteData((SHI::Data::StCraneAttitude*)attitude.get());
		GetStubCluster()->WriteData(dataCluster.get());
		GetStubDebugCluster()->WriteData(dataCluster.get());
		printf(" > Write cluster : %d clusters, %d points, total elapsed  %dms \n",
			dataCluster->GetClusterInfoSize(), dataCluster->GetXYZSize(), dataCluster->elapsedTimeClustering);
	}
}

void CClusterProcessor::OnDBInfo(SHI::Data::StDBInfo* pData)
{
	using namespace std::chrono;

	static steady_clock::time_point lastLog = steady_clock::now();
	static uint64_t intervalCount = 0;
	static double hdopSum = 0.0;
	static float minHdop = std::numeric_limits<float>::max();
	static float maxHdop = std::numeric_limits<float>::lowest();
	static int minQuality = std::numeric_limits<int>::max();
	static int maxQuality = std::numeric_limits<int>::lowest();
	static int minSatellites = std::numeric_limits<int>::max();
	static int maxSatellites = std::numeric_limits<int>::lowest();

	++intervalCount;
	hdopSum += pData->hook.gps.hdop;
	minHdop = (std::min)(minHdop, pData->hook.gps.hdop);
	maxHdop = (std::max)(maxHdop, pData->hook.gps.hdop);
	minQuality = (std::min)(minQuality, static_cast<int>(pData->hook.gps.quality));
	maxQuality = (std::max)(maxQuality, static_cast<int>(pData->hook.gps.quality));
	minSatellites = (std::min)(minSatellites, static_cast<int>(pData->hook.gps.numOfSatellites));
	maxSatellites = (std::max)(maxSatellites, static_cast<int>(pData->hook.gps.numOfSatellites));

	const auto now = steady_clock::now();
	const double secondsElapsed = duration_cast<duration<double>>(now - lastLog).count();

	if (secondsElapsed >= 5.0)
	{
		const double fps = secondsElapsed > 0.0 ? intervalCount / secondsElapsed : 0.0;
		const double avgHdop = intervalCount > 0 ? hdopSum / intervalCount : 0.0;

		const float lastLatitude = pData->hook.gps.latitude;
		const float lastLongitude = pData->hook.gps.longitude;
		const float lastHeight = pData->hook.gps.height;
		const float lastAzimuth = pData->hook.gps.azimuth;
		const int lastQuality = pData->hook.gps.quality;
		const int lastNumSats = pData->hook.gps.numOfSatellites;

		printf("[DBInfo] window=%.1fs fps=%.2f hdop(avg/min/max)=%.2f/%.2f/%.2f quality(min/max/last)=%d/%d/%d sats(min/max/last)=%d/%d/%d last(lat=%.6f lon=%.6f h=%.2f az=%.2f)\n",
			secondsElapsed,
			fps,
			avgHdop,
			minHdop == std::numeric_limits<float>::max() ? 0.0f : minHdop,
			maxHdop == std::numeric_limits<float>::lowest() ? 0.0f : maxHdop,
			minQuality == std::numeric_limits<int>::max() ? 0 : minQuality,
			maxQuality == std::numeric_limits<int>::lowest() ? 0 : maxQuality,
			lastQuality,
			minSatellites == std::numeric_limits<int>::max() ? 0 : minSatellites,
			maxSatellites == std::numeric_limits<int>::lowest() ? 0 : maxSatellites,
			lastNumSats,
			lastLatitude,
			lastLongitude,
			lastHeight,
			lastAzimuth);

		lastLog = now;
		intervalCount = 0;
		hdopSum = 0.0;
		minHdop = std::numeric_limits<float>::max();
		maxHdop = std::numeric_limits<float>::lowest();
		minQuality = std::numeric_limits<int>::max();
		maxQuality = std::numeric_limits<int>::lowest();
		minSatellites = std::numeric_limits<int>::max();
		maxSatellites = std::numeric_limits<int>::lowest();
		fflush(stdout);
	}
}

void CClusterProcessor::OnPlcInfo(SHI::Data::StPlcInfo* pData)
{

}

void CClusterProcessor::OnTimerDbInfoLog()
{
    // Periodic DBInfo snapshot logging to CSV (Excel-friendly)
    SHI::Data::StDBInfo dbinfo = {};
    if (!GetProxyDBInfo()->ReadBuffer(&dbinfo)) return;

    // Ensure header once per file
    bool needHeader = false;
    if (!m_dbInfoCsvHeaderWritten)
    {
        struct _stat st{};
        if (_stat(m_dbInfoCsvFile.c_str(), &st) != 0 || st.st_size == 0)
            needHeader = true;
    }

    FILE* fp = nullptr;
    if (fopen_s(&fp, m_dbInfoCsvFile.c_str(), "a") != 0 || fp == nullptr) return;

    if (needHeader)
    {
        fprintf(fp,
            "wall_ts,"
            "gps_time,gps_lat,gps_lon,gps_hdop,gps_height,gps_azimuth,gps_quality,gps_numSats,"
            "tc_hookHeight,tc_posTrolly,tc_azimuth,tc_statData,tc_statNetwork,"
            "llc_hookHeight1,llc_hookHeight2,llc_posDriving,llc_angleAzimuth,llc_statData,llc_statNetwork,"
            "gc_hookHeight1,gc_hookHeight2,gc_hookHeight3,gc_posTrolly1,gc_posTrolly2,"
            "gc_weight1,gc_weight2,gc_weight3,gc_weightTotal,gc_statData,gc_statNetwork,"
            "top_latitude,top_logitude\n");
        m_dbInfoCsvHeaderWritten = true;
    }

    Routine::CTime now;
    fprintf(fp,
        "%04d-%02d-%02d %02d:%02d:%02d,"
        "%.6f,%.8f,%.8f,%.3f,%.3f,%.3f,%d,%d,"
        "%.3f,%.3f,%.3f,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,%d,%d,"
        "%.8f,%.8f\n",
        now.Year(), now.Month(), now.Day(), now.Hour(), now.Min(), now.Second(),
        dbinfo.hook.gps.time, dbinfo.hook.gps.latitude, dbinfo.hook.gps.longitude, dbinfo.hook.gps.hdop,
        dbinfo.hook.gps.height, dbinfo.hook.gps.azimuth, dbinfo.hook.gps.quality, dbinfo.hook.gps.numOfSatellites,
        dbinfo.hook.tc.hookHeight, dbinfo.hook.tc.posTrolly, dbinfo.hook.tc.azimuth, dbinfo.hook.tc.statData, dbinfo.hook.tc.statNewtwork,
        dbinfo.hook.llc.hookHeight1, dbinfo.hook.llc.hookHeight2, dbinfo.hook.llc.posDriving, dbinfo.hook.llc.angleAzimuth, dbinfo.hook.llc.statData, dbinfo.hook.llc.statNewtwork,
        dbinfo.hook.gc.hookHeight1, dbinfo.hook.gc.hookHeight2, dbinfo.hook.gc.hookHeight3, dbinfo.hook.gc.posTrolly1, dbinfo.hook.gc.posTrolly2,
        dbinfo.hook.gc.weight1, dbinfo.hook.gc.weight2, dbinfo.hook.gc.weight3, dbinfo.hook.gc.weightTotal, dbinfo.hook.gc.statData, dbinfo.hook.gc.statNewtwork,
        dbinfo.latitude, dbinfo.logitude);
    fclose(fp);
}
