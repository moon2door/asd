#include "../DistanceProcessor.h"

#include <Utility/BuildInfo.h>
#include <Utility/Types.h>
#include <Config/InitialAttitude.h>
#include <Config/DistanceParameter.h>
#include <Utility/LoadCraneModel.h>

#include <Routine/include/Base/RoutineUtility.h>

int32_t idProxyCluster = 1;
int32_t idStubDistance = 1;
int32_t idProxyCraneAttitude = 2;
int32_t idProxyMiniInfo = 1;
int32_t idStubDebug = 10;

bool CDistanceProcessor::ProcessInitialize()
{
	bool ret = true;

	// ���� ���� �ֿܼ� ���
	uint32_t date = SHI::BuildInfo::Date();
	int32_t year = date / 10000;
	int32_t month = (date % 10000) / 100;
	int32_t day = date % 100;
	printf("[Distance Processor]\n");
	printf("Built : %d-%d-%d %s \n", year, month, day, __TIME__);
	
	// �Ķ���� ���� �� �ʱ�ȭ
	m_attitude = SHI::CraneAttitudePtr(new SHI::CraneAttitude);
	m_distanceParam = SHI::DistanceParamPtr(new SHI::DistanceParam);
	m_vRefPartModels = SHI::DistanceModelVectorPtr(new SHI::DistanceModelVector);
	m_bDebug = Routine::GetConfigInt("Debug", "bDebugMode", 0) == 0 ? false : true;
	int32_t craneId = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
	std::string szPier = Routine::GetConfigString("CraneType", "Pier","Y", "CraneType.json");
	int32_t pierId = SHI::ConvPierInt(szPier);

	//m_attitude = std::make_shared<SHI::CraneAttitude>();
	//m_distanceParam = std::make_shared<SHI::DistanceParam>();
	//m_vRefPartModels = std::make_shared<SHI::DistanceModelVector>();

	bool bGetInitialAttitude = SHI::GetInitialAttitude(pierId, craneId, *m_attitude);
	bool bGetDistanceParam =  SHI::GetDistanceParam(pierId, craneId, *m_distanceParam);
	
	if ( bGetInitialAttitude == true && bGetDistanceParam == true )
	{
		// Ʈ��ŷ �ʱ�ȭ
		m_tracker = new CDistanceTracker;
		m_tracker->Initalize(5.0, 5, 5, 255);

		m_trackerHook = new CDistanceTracker;
		m_trackerHook->Initalize(5.0, 5, 5, 5);

		// �Ÿ� ��� �� �ε�
		bool bLoad = SHI::LoadDistanceModel(*m_vRefPartModels, m_distanceParam);
		printf("LoadPolygonModel : %s \n", bLoad ? "succeed" : "fail");

		// �������̽� ����
		bool bCreatedProxyCluster = GetProxyCluster()->Create(idProxyCluster);
		printf("Create ProxyCluster(%d) : %s \n", idProxyCluster, bCreatedProxyCluster ? "succeed" : "fail");

		bool bCreatedStubDistance = GetStubDistance()->Create(idStubDistance);
		printf("Create StubDistance(%d) : %s \n", idStubDistance, bCreatedStubDistance ? "succeed" : "fail");

		bool bCreatedStubDebug = GetStubDebugDistance()->Create(idStubDebug);
		printf("Create StubDebug(%d) : %s \n", idStubDebug, bCreatedStubDebug ? "succeed" : "fail");

		bool bCreatedProxyMiniInfo = GetProxyMonitoringMiniInfo()->Create(idProxyMiniInfo);
		printf("Create ProxyMiniInfo(%d) : %s \n", idProxyMiniInfo, bCreatedProxyMiniInfo ? "succeed" : "fail");

		ret = bCreatedProxyCluster && bCreatedStubDistance && bCreatedStubDebug;
	}
	return ret;
}
