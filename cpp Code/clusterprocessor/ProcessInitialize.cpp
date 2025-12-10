#include "../ClusterProcessor.h"

#include <Utility/BuildInfo.h>
#include <Utility/Types.h>
#include <Utility/LoadCraneModel.h>
#include <Config/InitalizeCrane.h>
#include <Config/ClusteringParameter.h>
#include <Config/CraneInfo.h>
#include <Config/InitalizeCrane.h>
#include <Routine/Include/Base/RoutineUtility.h>
#include <Object/CLog/CLogDynamic.h>

int g_idProxyXYZPoint = 2;
int g_idStubCluster = 2;
int g_idStubCraneAttitude = 2;
int g_idProxyDBInfo = 2;
int g_idStubDBInfo = 3;
int g_idDebug = 10;
int g_maxAccumSize = 3;
int g_idProxyPlcInfo = 3;

bool CClusterProcessor::ProcessInitialize()
{
	bool ret = false;

	// 버전 정보 콘솔에 출력
	unsigned int date = SHI::BuildInfo::Date();
	int year = date / 10000;
	int month = (date % 10000) / 100;
	int day = date % 100;
	printf("[Cluster Processor]\n");
	printf("Built : %d-%d-%d %s \n", year, month, day, __TIME__);

	// 파라미터 설정 및 초기화
	SHI::CraneAttitude attitude;
	SHI::ClusterParam clusterParam;
	std::vector<SHI::PointCloudPtr> vRefPartPoints;
	int craneId = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
	std::string szPier = Routine::GetConfigString("CraneType", "Pier", "Y", "CraneType.json");
	int pierId = SHI::ConvPierInt(szPier);

	bool bDebug = Routine::GetConfigInt("Debug", "bDebugMode", 0) == 0 ? false : true;
	SetDebugMode(bDebug);

	// 크레인 초기화
	ret = SHI::InitializeCrane(pierId, craneId, this, attitude, clusterParam, this, bDebug);
	
	if (ret == true)
	{
		// 포인트 모델 로드
		bool bLoad = SHI::LoadPointModel(vRefPartPoints, attitude, clusterParam.fnameModel);
		printf("LoadPointModel : %s \n", bLoad ? "succeed" : "fail");

		// 파라미터 업데이트
		SetCraneAttitude(attitude);
		SetClusterParam(clusterParam);
		SetRefPartPoints(vRefPartPoints);

		// 인터페이스 생성
		bool bCreatedProxyXYZPoint = GetProxyXYZPoint()->Create(g_idProxyXYZPoint);
		printf("create ProxyXYZPoint(%d) : %s \n", g_idProxyXYZPoint, bCreatedProxyXYZPoint ? "succeed" : "fail");

		bool bCreatedStubCluster = GetStubCluster()->Create(g_idStubCluster);
		printf("create StubCluster(%d) : %s \n", g_idStubCluster, bCreatedStubCluster ? "succeed" : "fail");

		bool bCreatedstubAttitude = GetStubCraneAttitude()->Create(g_idStubCraneAttitude);
		printf("create StubCraneAttitude(%d) : %s \n", g_idStubCraneAttitude, bCreatedstubAttitude ? "succeed" : "fail");

		bool bcreatedproxydbinfo = GetProxyDBInfo()->Create(g_idProxyDBInfo);
		printf("create proxydbinfo(%d) : %s \n", g_idProxyDBInfo, bcreatedproxydbinfo ? "succeed" : "fail");

		bool bcreatedstubdbinfo = GetStubDBInfo()->Create(g_idStubDBInfo);
		printf("create stubdbinfo(%d) : %s \n", g_idStubDBInfo, bcreatedstubdbinfo ? "succeed" : "fail");
		
		bool bCreatedStubDebug = GetStubDebugCluster()->Create(g_idDebug);
		printf("create stubDebug(%d) : %s \n", g_idDebug, bCreatedStubDebug ? "succeed" : "fail");

    bool bCreatedProxyPlcInfo = GetProxyPlcInfo()->Create(g_idProxyPlcInfo);
    printf("create ProxyPlcInfo(%d) : %s \n", g_idProxyPlcInfo, bCreatedProxyPlcInfo ? "succeed" : "fail");

    // DBInfo CSV Logger (optional, independent cadence)
    int logEnable = Routine::GetConfigInt("DBInfoLog", "Enable", 0, "ClusterProcessor.json");
    double periodSec = Routine::GetConfigDouble("DBInfoLog", "PeriodSec", 5.0, "ClusterProcessor.json");
    std::string logDir = Routine::GetConfigString("DBInfoLog", "Dir", "./log", "ClusterProcessor.json");
    std::string logFile = Routine::GetConfigString("DBInfoLog", "CsvFile", "dbinfo.csv", "ClusterProcessor.json");
    if (!logDir.empty() && (logDir.back() == '/' || logDir.back() == '\\'))
        m_dbInfoCsvFile = logDir + logFile;
    else
        m_dbInfoCsvFile = logDir + "/" + logFile;
    if (logEnable)
    {
#ifdef _WIN32
        _mkdir(logDir.c_str());
#else
        mkdir(logDir.c_str(), 0755);
#endif
        int periodMs = static_cast<int>(periodSec * 1000.0);
        if (periodMs < 100) periodMs = 100; // clamp
        m_timerDbInfoLog.StartTimer(periodMs, &CClusterProcessor::OnTimerDbInfoLog, this);
        printf("DBInfo CSV logging: %s, period=%.1fs\n", m_dbInfoCsvFile.c_str(), periodSec);
    }
		

		// DBInfo Logger (optional, independent cadence)
		logEnable = Routine::GetConfigInt("DBInfoLog", "Enable", 0, "ClusterProcessor.json");
		periodSec = Routine::GetConfigDouble("DBInfoLog", "PeriodSec", 5.0, "ClusterProcessor.json");
		logDir = Routine::GetConfigString("DBInfoLog", "Dir", "./log/dbinfo", "ClusterProcessor.json");
		std::string logExt = Routine::GetConfigString("DBInfoLog", "Ext", "dbinfo", "ClusterProcessor.json");
		if (logEnable)
		{
			SHI::Object::CLogDynamic::Instance().Initialize(logDir, logExt, 1000);
			int periodMs = static_cast<int>(periodSec * 1000.0);
			if (periodMs < 100) periodMs = 100; // clamp
			m_timerDbInfoLog.StartTimer(periodMs, &CClusterProcessor::OnTimerDbInfoLog, this);
			printf("DBInfo logging enabled: dir=%s, ext=%s, period=%.1fs\n", logDir.c_str(), logExt.c_str(), periodSec);
		}

        ret = bLoad && bCreatedProxyXYZPoint && bCreatedStubCluster && bCreatedstubAttitude && bCreatedStubDebug && bCreatedProxyPlcInfo;
	}

	return ret;
}
