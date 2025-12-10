#include "stdafx.h"
#include "ColiisionProcessor.h"
#include <Utility/BuildInfo.h>
#include <Config/CraneInfo.h>
#include "FileUtility.h"
#include "colorTable.h"
#include <Utility/Types.h>
#include <Utility/mathematics.h>

#include <Routine/Include/Base/CElapseTimer.h>
#include <Routine/Include/Base/RoutineUtility.h>
#include <Routine/include/Base/CTime.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>

bool IsOnVectorDirection(Eigen::Vector3f& vecStd, Eigen::Vector3f& vecInput, float_t m_angleThreshold);

unsigned char Estimate(float_t distance, float_t v[3]);

int32_t g_idProxyCollisionZoneLength = 2;
int32_t g_idStubCollisionInformation = 2;
int32_t g_idProxyDistance = 2;
float_t g_angleThreshold = 85;

CColiisionProcessor::CColiisionProcessor()
{
	float_t trans[16] = { 0.95f, 0.05f, 0.0f, 0.0f, 0.05f, 0.8f, 0.05f, 0.0f, 0.0f, 0.05f, 0.8f, 0.05f, 0.0f, 0.0f, 0.05f, 0.95f };

	float_t emits[] = {
		0.8f, 0.0f, 0.0f, 0.1f,
		0.2f, 0.8f, 0.0f, 0.1f,
		0.0f, 0.2f, 0.8f, 0.1f,
		0.0f, 0.0f, 0.2f, 0.7f };

	float_t inits[] = { 0.25f, 0.25f, 0.25f, 0.25f };

	m_stateFilter.Create(4, trans, emits, inits, 30);
}


CColiisionProcessor::~CColiisionProcessor()
{
	m_thread.StopThread();
}


bool CColiisionProcessor::Create()
{
	//R::Mathematics::InitSinCosTable();

	// 버전 정보
	uint32_t date = SHI::BuildInfo::GetBuildDate();
	int32_t year = date / 10000;
	int32_t month = (date % 10000) / 100;
	int32_t day = date % 100;
	printf("[Collision Processor]\n");
	printf("Built : %d-%d-%d %s \n", year, month, day, __TIME__);

	// 크레인 번호 읽어오기
	std::string szPier = Routine::GetConfigString("CraneType", "Pier", "Y","CraneType.json");
	m_pier = SHI::ConvPierInt(szPier);
	m_craneIndex = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
	printf("Crane pier : %s, Type : %s \n", SHI::ConvPierDisplayStr(m_pier).c_str(), SHI::ConvCraneStr(m_pier, m_craneIndex).c_str());


	m_llcBodyAngleThreshold = 5;
	if (m_pier == SHI::PIER7)
	{
		if (m_craneIndex == 1 || m_craneIndex == 2)
		{
			m_llcBodyAngleThreshold = Routine::GetConfigDouble("LLC_Option", "AngleThreshold", 7);
		}
	}
	else if (m_pier == SHI::PIER7)
	{
		m_llcBodyAngleThreshold = Routine::GetConfigDouble("LLC_Option", "AngleThreshold", 7);
	}
	else
	{
	}

	// 초기화
	m_eDone.Create();
	bool bDebug = Routine::GetConfigInt("Debug", "ShowDebugViewer", 0) == 0 ? false : true;
	if (bDebug) m_thread.StartThread(&CColiisionProcessor::Run, this);
	m_bDebug = Routine::GetConfigInt("Debug", "bDebugLog", 0) == 0 ? false : true;

	// 인터페이스 생성
	bool bRet = false;

	bool bCreateProxyCollisionZoneLength = GetProxyCollisionZoneLength()->Create(g_idProxyCollisionZoneLength);
	printf("Create ProxyCollisionZoneLength(%d) : %s \n", g_idProxyCollisionZoneLength, bCreateProxyCollisionZoneLength ? "succeed" : "failed");

	bool bCreateStubCollisionInformation = GetStubCollisionInformation()->Create(g_idStubCollisionInformation);
	printf("Create ProxyCollisionZoneLength(%d) : %s \n", g_idStubCollisionInformation, bCreateStubCollisionInformation ? "succeed" : "failed");

	bool bCreateProxyDistance = GetProxyDistance()->Create(g_idProxyDistance);
	printf("Create ProxyDistance(%d) : %s \n", g_idProxyDistance, bCreateProxyDistance ? "succeed" : "failed");


	bRet = bCreateProxyDistance & bCreateProxyCollisionZoneLength & bCreateStubCollisionInformation;

	return bRet;
}

void CColiisionProcessor::OnDistance(SHI::Data::StDistance* pData)
{
	printf("OnDistance : %d points, %d clusters, %d distances \n", pData->GetXYZSize(), pData->GetClusterInfoSize(), pData->GetDistanceInfoSize());
	Routine::CElapseTimer elapsed;
	std::shared_ptr<std::vector<bool>> vOnExceptionInfo(new std::vector<bool>);

	// 입력 데이터 초기화
	std::shared_ptr<SHI::Data::StDistance> distanceInput(new SHI::Data::StDistance);
	memset(distanceInput.get(), 0, sizeof(sizeof(SHI::Data::StDistance)));
	memcpy(distanceInput.get(), pData, pData->GetSize());

	// 출력 데이터 초기화
	std::shared_ptr<SHI::Data::StCollisionInformation> collisionOutput(new SHI::Data::StCollisionInformation);
	memset(collisionOutput.get(), 0, sizeof(SHI::Data::StCollisionInformation));
	collisionOutput->InitBuffer();

	uint32_t sizeDistance = distanceInput->GetDistanceInfoSize();
	if (collisionOutput->AllocCollisionInfo(sizeDistance) &&
		collisionOutput->AllocReasonInfo(sizeDistance) &&
		distanceInput->GetDistanceLabelSize() == distanceInput->GetDistanceInfoSize())
	{
		unsigned char collisionTotal = 0;
		vOnExceptionInfo->resize(sizeDistance);

		// 입력된 거리 데이터를 순회하며 충돌 위험도 평가 수행
		for (size_t i = 0; i < sizeDistance; i++)
		{
			unsigned char label = distanceInput->GetDistanceLabel()[i];
			SHI::Data::StDistanceInfo info = distanceInput->GetDistanceInfo()[i];
			float_t distance = info.Distance;
			Eigen::Vector3f vecObject(info.PosCluster.X, info.PosCluster.Y, info.PosCluster.Z);
			Eigen::Vector3f vecCrane(info.PosCrane.X, info.PosCrane.Y, info.PosCrane.Z);
			Eigen::Vector3f vecCraneToObject = vecObject - vecCrane; // 기준 벡터(크레인 포인트 ~ 물체 포인트)
			unsigned char level = 0;
			bool bException = true;
			unsigned char collisionReason = 0;

			if (label == SHI::DISTANCE_NORMAL) // 후크 거리가 아닌 경우에만 충돌 위험도 계산
			{
				// 1. 크레인 종류와 파트 부위에 따른 기준 방향 검사
				if (m_pier == SHI::PIER7)
				{
					if (m_craneIndex == 0)
					{
						// 골리앗 크레인
						// 크레인 이동에 따른 이동 방향의 충돌 고려한다
						// 이동 방향(X축)이 아닌 충돌은 예외 영역으로 판단한다.
						bException = !IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitX()), 90 - g_angleThreshold);
					}
					else if (m_craneIndex == 1 || m_craneIndex == 2) // LLC
					{
						// LLC 크레인				
						if (info.CraneIndex == SHI::Pier7::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							// Jib의 경우 크레인 고각 이동(고각에 수직방향의 충돌)과 회전 방향(X축방향 충돌)의 충돌을 함께 고려한다
							// 그리고 크레인 이동에 따른 이동 방향도 같이 고려한다.
							// 따라서, 모든 방향의 거리가 모두 관심 있는 거리가 된다.
							bException = false;
						}
						else if (info.CraneIndex == SHI::Pier7::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								bException = false;
							}
							else
							{
								// 조건 : 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
					else
					{
						// 타워 크레인
						// 조건 : 회전방향에 수직인 경우 예외 처리
						bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);
					}
				}
				if (m_pier == SHI::PIERJ)
				{
					if (info.CraneIndex == SHI::PierJ::LLC_JIB) // LLC Jib (100=jib, 101:tower)
					{
						// Jib의 경우 크레인 고각 이동(고각에 수직방향의 충돌)과 회전 방향(X축방향 충돌)의 충돌을 함께 고려한다
						// 그리고 크레인 이동에 따른 이동 방향도 같이 고려한다.
						// 따라서, 모든 방향의 거리가 모두 관심 있는 거리가 된다.
						bException = false;
					}
					else if (info.CraneIndex == SHI::PierJ::LLC_TOWER) // LLC Tower
					{
						// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
						double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
						Eigen::Matrix3f rot;
						rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
						Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																								// 조건 : 이동방향과 수직인 경우 예외 처리
						bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
					}
					else // LLC BODY
					{
						// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
						double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
						Eigen::Matrix3f rot;
						rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
						Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
						Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
						Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
						if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
						{
							bException = false;
						}
						else
						{
							// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
					}
				}
				if (m_pier == SHI::PIERK)
				{
					if (m_craneIndex == SHI::PierK::JIB1 ||
						m_craneIndex == SHI::PierK::JIB2 ||
						m_craneIndex == SHI::PierK::JIB3 ||
						m_craneIndex == SHI::PierK::LLC18 ||
						m_craneIndex == SHI::PierK::LLC19)
					{
						if (info.CraneIndex == SHI::PierK::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							// Jib의 경우 크레인 고각 이동(고각에 수직방향의 충돌)과 회전 방향(X축방향 충돌)의 충돌을 함께 고려한다
							// 그리고 크레인 이동에 따른 이동 방향도 같이 고려한다.
							// 따라서, 모든 방향의 거리가 모두 관심 있는 거리가 된다.
							bException = false;
						}
						else if (info.CraneIndex == SHI::PierK::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								bException = false;
							}
							else
							{
								// 조건 : 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
					else // TTC
					{
						if (info.CraneIndex == SHI::PierK::TTC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);
						}
						else if (info.CraneIndex == SHI::PierK::TTC_TOWER)
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier7::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else
						{
						}
					}
				}
				else if (m_pier == SHI::PIERHAN)
				{
					if (m_craneIndex == SHI::PierHan::GC1 || m_craneIndex == SHI::PierHan::GC2)
					{
						// 골리앗 크레인

						if (info.CraneIndex == SHI::PierHan::GC_LOWER_TROLLY_HOOK ||
							info.CraneIndex == SHI::PierHan::GC_UPPER_TROLLY_HOOK)
						{
							bException = false;
						}
						else
						{
							// 크레인 이동에 따른 이동 방향의 충돌 고려한다
							// 이동 방향(X축)이 아닌 충돌은 예외 영역으로 판단한다.
							bException = !IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitX()), 90 - g_angleThreshold);
						}
					}
					else if (m_craneIndex == SHI::PierHan::TTC4)
					{
						if (info.CraneIndex == SHI::PierHan::TTC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 79.0f;
							float_t distanceCounterJib = 29.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
						else if (info.CraneIndex == SHI::PierHan::TTC_TOWER)
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::PierHan::TTC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else if (info.CraneIndex == SHI::PierHan::TTC_TROLLY || info.CraneIndex == SHI::PierHan::TTC_TROLLY2)
						{
							bException = false;
						}
						else
						{
						}
					}
					// TC
					else if (m_craneIndex == SHI::PierHan::TC1)
					{
						// 타워 크레인
						// 조건 : 회전방향에 수직인 경우 예외 처리
						//bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

						if (info.CraneIndex == SHI::PierHan::TC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 71.0f;
							float_t distanceCounterJib = 24.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
					}
					else if (m_craneIndex == SHI::PierHan::TC2)
					{
						if (info.CraneIndex == SHI::PierHan::TC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 61.0f;
							float_t distanceCounterJib = 23.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
					}
					else if (m_craneIndex == SHI::PierHan::TC3)
					{
						if (info.CraneIndex == SHI::PierHan::TC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 71.0f;
							float_t distanceCounterJib = 24.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
					}
					else if (m_craneIndex == SHI::PierHan::TC5)
					{
						if (info.CraneIndex == SHI::PierHan::TC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 71.0f;
							float_t distanceCounterJib = 24.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
					}
					else if (m_craneIndex == SHI::PierHan::TC6)
					{
						if (info.CraneIndex == SHI::PierHan::TC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 71.0f;
							float_t distanceCounterJib = 23.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
					}
					else
					{
						// 회전방향에 수직인 경우 예외 처리
						bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);
					}
				}
				else if (m_pier == SHI::PIER6)
				{
					if (m_craneIndex == SHI::Pier6::LLC7 ||
						m_craneIndex == SHI::Pier6::LLC23)
					{
						if (info.CraneIndex == SHI::Pier6::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							// Jib의 경우 크레인 고각 이동(고각에 수직방향의 충돌)과 회전 방향(X축방향 충돌)의 충돌을 함께 고려한다
							// 그리고 크레인 이동에 따른 이동 방향도 같이 고려한다.
							// 따라서, 모든 방향의 거리가 모두 관심 있는 거리가 된다.
							bException = false;
						}
						else if (info.CraneIndex == SHI::Pier6::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier6::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::Pier6::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								bException = false;
							}
							else
							{
								// 조건 : 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
				}
				else if (m_pier == SHI::G2DOCK)
				{
					if (m_craneIndex == SHI::G2Dock::LLC12 ||
						m_craneIndex == SHI::G2Dock::LLC13)
					{
						if (info.CraneIndex == SHI::G2Dock::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							float_t distanceJib = 52.0f;
							bException = true;

							bool bHighAngle = false;
							if (vecCraneToObject.y() > 0 &&
								vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecX = Eigen::Vector3f::UnitX();
								bHighAngle = IsOnVectorDirection(vecCraneToObject, vecX, 90 - g_angleThreshold);
							}

							bool bAzimuth = false;
							if (vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecZ = Eigen::Vector3f::UnitZ();
								bAzimuth = IsOnVectorDirection(vecCraneToObject, vecZ, 90 - g_angleThreshold);
							}

							if (bHighAngle || bAzimuth)
							{
								bException = false;
							}
						}
						else if (info.CraneIndex == SHI::G2Dock::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::G2Dock::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우 A, 아닌 경우 B 적용
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::G2Dock::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								// 조건A: 몸체보다 아래에 있으면 예외 처리
								//bException = vecCraneToObject.z() < -0.5f;
								bException = vecObject.z() < -1.0f;
							}
							else
							{
								// 조건B: 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
				}
				else if (m_pier == SHI::G3DOCK)
				{
					if (m_craneIndex == SHI::G3Dock::LLC19 ||
						m_craneIndex == SHI::G3Dock::LLC20)
					{
						if (info.CraneIndex == SHI::G3Dock::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							float_t distanceJib = 52.0f;
							bException = true;

							bool bHighAngle = false;
							if (vecCraneToObject.y() > 0 &&
								vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecX = Eigen::Vector3f::UnitX();
								bHighAngle = IsOnVectorDirection(vecCraneToObject, vecX, 90 - g_angleThreshold);
							}

							bool bAzimuth = false;
							if (vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecZ = Eigen::Vector3f::UnitZ();
								bAzimuth = IsOnVectorDirection(vecCraneToObject, vecZ, 90 - g_angleThreshold);
							}

							if (bHighAngle || bAzimuth)
							{
								bException = false;
							}
						}
						else if (info.CraneIndex == SHI::G3Dock::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::G3Dock::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우 A, 아닌 경우 B 적용
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::G3Dock::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								// 조건A: 몸체보다 아래에 있으면 예외 처리
								//bException = vecCraneToObject.z() < -0.5f;
								bException = vecObject.z() < -2.0f; // 바디 아래로 1m 정도 여유
							}
							else
							{
								// 조건B: 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
				}
				else if (m_pier == SHI::G4DOCK)
				{
					if (m_craneIndex == SHI::G4Dock::LLC25 ||
						m_craneIndex == SHI::G4Dock::LLC26)
					{
						if (info.CraneIndex == SHI::G4Dock::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							float_t distanceJib = 52.0f;
							bException = true;

							bool bHighAngle = false;
							if (vecCraneToObject.y() > 0 &&
								vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecX = Eigen::Vector3f::UnitX();
								bHighAngle = IsOnVectorDirection(vecCraneToObject, vecX, 90 - g_angleThreshold);
							}

							bool bAzimuth = false;
							if (vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecZ = Eigen::Vector3f::UnitZ();
								bAzimuth = IsOnVectorDirection(vecCraneToObject, vecZ, 90 - g_angleThreshold);
							}

							if (bHighAngle || bAzimuth)
							{
								bException = false;
							}
						}
						else if (info.CraneIndex == SHI::G4Dock::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::G4Dock::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우 A, 아닌 경우 B 적용
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::G4Dock::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								// 조건A: 몸체보다 아래에 있으면 예외 처리
								//bException = vecCraneToObject.z() < -0.5f;
								bException = vecObject.z() < -2.52f;
							}
							else
							{
								// 조건B: 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
				}
				//pjh
				else if (m_pier == SHI::PIERZ)
				{
					
						if (info.CraneIndex == SHI::PierZ::LLC_JIB) // LLC Jib (100=jib, 101:tower)
						{
							float_t distanceJib = 52.0f;
							bException = true;

							bool bHighAngle = false;
							if (vecCraneToObject.y() > 0 &&
								vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecX = Eigen::Vector3f::UnitX();
								bHighAngle = IsOnVectorDirection(vecCraneToObject, vecX, 90 - g_angleThreshold);
							}

							bool bAzimuth = false;
							if (vecCraneToObject.norm() < distanceJib)
							{
								Eigen::Vector3f vecZ = Eigen::Vector3f::UnitZ();
								bAzimuth = IsOnVectorDirection(vecCraneToObject, vecZ, 90 - g_angleThreshold);
							}

							if (bHighAngle || bAzimuth)
							{
								bException = false;
							}
						}
						else if (info.CraneIndex == SHI::PierZ::LLC_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::PierZ::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우 A, 아닌 경우 B 적용
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::PierZ::LLC_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								// 조건A: 몸체보다 아래에 있으면 예외 처리
								//bException = vecCraneToObject.z() < -0.5f;
								bException = vecObject.z() < -2.52f;
							}
							else
							{
								// 조건B: 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					
				}
				//
				//Y
				else if (m_pier == SHI::PIERY) 
				{
					if (m_craneIndex == SHI::PierY::TC1)
					{
						/**				방법1				**/ 
						// 타워 크레인
						// 조건 : 회전방향에 수직인 경우 예외 처리
						bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);


						/* 방법2 */
						/*
						if (info.CraneIndex == SHI::PierY::TC_JIB)
						{
							// 회전방향에 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, Eigen::Vector3f(Eigen::Vector3f::UnitZ()), g_angleThreshold);

							// 지브 반경에 대한 예외 처리
							float_t distanceJib = 71.0f;
							float_t distanceCounterJib = 24.0f;
							if (bException == false)
							{
								float_t distance = sqrt(vecObject.x() * vecObject.x() + vecObject.y() * vecObject.y() + vecObject.z() * vecObject.z());
								if (vecObject.y() > 0 && distance > distanceJib)
								{
									bException = true;
								}
								if (vecObject.y() < 0 && distance > distanceCounterJib)
								{
									bException = true;
								}
							}
						}
						*/

						// 현재 더 심플해보이는 방법1 사용중.
					}

					else if (m_craneIndex == SHI::PierY::JIB1 || m_craneIndex == SHI::PierY::JIB2 ||
				m_craneIndex == SHI::PierY::JIB3 || m_craneIndex == SHI::PierY::JIB4 ||
				m_craneIndex == SHI::PierY::JIB5)
					{
						if (info.CraneIndex == SHI::PierY::JIB_JIB) // LLC Jib (100=jib, 101:tower)
						{
							// Jib의 경우 크레인 고각 이동(고각에 수직방향의 충돌)과 회전 방향(X축방향 충돌)의 충돌을 함께 고려한다
							// 그리고 크레인 이동에 따른 이동 방향도 같이 고려한다.
							// 따라서, 모든 방향의 거리가 모두 관심 있는 거리가 된다.
							bException = false;
						}
						else if (info.CraneIndex == SHI::PierY::JIB_TOWER) // LLC Tower
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::PierY::JIB_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
																									// 조건 : 이동방향과 수직인 경우 예외 처리
							bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
						}
						else // LLC BODY
						{
							// 조건 : 방위각이 진행 방향과 나란하지 않은 경우에는 무조건 일반 거리로 적용한다.
							double_t angle = SHI::Math::DegreeToRadian(distanceInput->attitude.pose[SHI::PierY::JIB_TOWER]);
							Eigen::Matrix3f rot;
							rot = Eigen::AngleAxisf(-angle, Eigen::Vector3f::UnitZ());
							Eigen::Vector3f vecFront = Eigen::Vector3f::UnitY();
							Eigen::Vector3f vecMovingDirection = rot * Eigen::Vector3f::UnitY(); // 이동 방향 벡터
							Eigen::Vector3f vecMovingDirectionArc = rot * Eigen::Vector3f::UnitX(); // 이동 방향에 수직인 벡터
							if (!IsOnVectorDirection(vecFront, vecMovingDirection, m_llcBodyAngleThreshold))
							{
								bException = false;
							}
							else
							{
								// 조건 : 이동방향과 수직인 경우 예외 처리
								bException = IsOnVectorDirection(vecCraneToObject, vecMovingDirectionArc, g_angleThreshold);
							}
						}
					}
				}
				// 
				// 2. 거리에 다른 위험도 평가
				if (bException)
				{
					CModelCollisionZoneLength::StZone zone = GetCollisionZoneException(m_pier, m_craneIndex, info.CraneIndex);
					level = Estimate(distance, zone.dists);
					collisionReason = GetCollisionIndexException(m_pier, m_craneIndex, info.CraneIndex);
					if (level > 0) printf(" #%d : %d(d=%.2f, exception roi) (patr %d, distsException - %.2f/%.2f/%.2f)\n", i, level, distance,
						info.CraneIndex,
						zone.dists[0],
						zone.dists[1],
						zone.dists[2]);
				}
				else
				{
					CModelCollisionZoneLength::StZone zone = GetCollisionZone(m_pier, m_craneIndex, info.CraneIndex);
					level = Estimate(distance, zone.dists);
					collisionReason = GetCollisionIndex(m_pier, m_craneIndex, info.CraneIndex);
					if (level > 0) printf(" #%d : %d(d=%.2f, normal roi) (patr %d, dists - %.2f/%.2f/%.2f)\n", i, level, distance,
						info.CraneIndex,
						zone.dists[0],
						zone.dists[1],
						zone.dists[2]);
				}
			}

			// 3. 최종 위험도 갱신
			collisionOutput->GetCollisionInfo()[i] = level;
			collisionOutput->GetReasonInfo()[i] = collisionReason;
			collisionTotal = std::max(collisionTotal, level);

			vOnExceptionInfo->push_back(bException);
		}

		// 위험도 수준 추정필터 처리
		if (m_stateFilter.IsCreated())
		{
			collisionTotal = m_stateFilter.Push(collisionTotal);
			Eigen::MatrixXi seq, states;
			m_stateFilter.GetStates(states, seq);

			std::cout << " state filter " << seq << " => " << states << std::endl;
		}

		// 최종 위험도 필터 처리
		// 최근 n회 결과중 가장 높은 위험 수준으로 처리함
		if (m_collisionHistory.size() >= m_maxCollision)
		{
			m_collisionHistory.pop_front();
		}

		m_collisionHistory.emplace_back(collisionTotal);

		const auto itMaxElement = std::max_element(m_collisionHistory.begin(), m_collisionHistory.end());
		if (itMaxElement != m_collisionHistory.end())
		{
			collisionTotal = *itMaxElement;
		}

		// 필터 처리된 최종 위험도를 기준으로 각 거리의 위험도 재평가
		for (uint32_t i = 0; i < collisionOutput->GetCollisionInfoSize(); i++)
		{
			collisionOutput->GetCollisionInfo()[i] = std::min(collisionOutput->GetCollisionInfo()[i], collisionTotal);
		}

		// 출력 데이터 쓰기
		collisionOutput->version = SHI::BuildInfo::GetBuildDate();
		collisionOutput->collisionTotal = collisionTotal;
		collisionOutput->elapsedTimeClustering = distanceInput->elapsedTimeClustering;
		collisionOutput->elapsedTimeDistance = distanceInput->elapsedTimeDistance;
		collisionOutput->elapsedTimeCollision = elapsed.GetElapseTime() * 0.001;
		collisionOutput->timeStamp = pData->timeStamp;

		// 결과 화면 출력
		char result[256] = "";
		if (collisionTotal == 0) sprintf(result, "OK");
		else if (collisionTotal == 1) sprintf(result, "Caution");
		else if (collisionTotal == 2) sprintf(result, "Warning");
		else sprintf(result, "Stop");
		printf(" Result : %s, elapsed %d ms\n", result, collisionOutput->elapsedTimeCollision);

		// 결과 송신
		GetStubCollisionInformation()->WriteData(collisionOutput.get());

		// 멤버에 결과 전달
		m_distanceInput = distanceInput;
		m_collisionOutput = collisionOutput;
		m_vOnDirectionInfo = vOnExceptionInfo;
		m_eDone.SetEvent();

		// 디버깅용 저장
		if (m_bDebug)
		{
			// 결과가 '정상' 이 아닌 경우 입력 거리데이터를 파일에 저장함
			if (collisionTotal != 0)
			{
				char fname[256] = "";
				Routine::CTime t;
				t.UpdateCurrentTime();
				sprintf(fname, "./%s(%d).StDistance", t.GetTimeToString(), collisionTotal);

				FILE* fp = fopen(fname, "wb");
				if (fp)
				{
					fwrite(distanceInput.get(), sizeof(SHI::Data::StDistance), 1, fp);
					fclose(fp);
				}

			}
		}
	}
	else
	{
		printf(" Error : StCollisionInformation Allocation failed. \n");
	}
}

void CColiisionProcessor::OnCollisionZoneLength(SHI::Data::StCollisionZoneLength* pData)
{
	UpdateCollisionZoneLength(pData);

}

void CColiisionProcessor::Run()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("test viewer"));
	std::shared_ptr<SHI::Data::StDistance> distanceInput;
	std::shared_ptr<SHI::Data::StCollisionInformation> collisionOutput;
	std::shared_ptr<std::vector<bool>> vOnDirectionInfo;

	// 포인트 클라우드
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer->addPointCloud(cloudPoints, "cloudInput");
	viewer->addPointCloud(cloudCluster, "cloudCluster");

	while (m_thread.IsRunThread())
	{
		if (m_eDone.WaitForEvent(10))
		{
			// 입출력 데이터 가져오기
			distanceInput = m_distanceInput;
			collisionOutput = m_collisionOutput;
			vOnDirectionInfo = m_vOnDirectionInfo;

			viewer->removeAllShapes();

			// 입력 포인트 그리기
			cloudPoints->resize(distanceInput->GetXYZSize());
			memcpy(cloudPoints->points.data(), distanceInput->GetXYZ(), cloudPoints->points.size() * sizeof(pcl::PointXYZ));
			viewer->updatePointCloud(cloudPoints, "cloudInput");

			// 클러스터 정보 그리기
			cloudCluster->clear();
			uint32_t clusterInfoSize = distanceInput->GetClusterInfoSize();
			for (uint32_t i = 0; i < clusterInfoSize; i++)
			{
				uint32_t index = distanceInput->GetClusterInfo()[i].Index;
				uint32_t size = distanceInput->GetClusterInfo()[i].Size;

				unsigned char b = ColorTable::GetColorTableB(i) * 255;
				unsigned char g = ColorTable::GetColorTableG(i) * 255;
				unsigned char r = ColorTable::GetColorTableR(i) * 255;

				for (uint32_t i = 0; i < size; i++)
				{
					uint32_t idxPoint = distanceInput->GetClusterIndices()[index + i];
					pcl::PointXYZRGB p(r, g, b);
					p.x = distanceInput->GetXYZ()[idxPoint].X;
					p.y = distanceInput->GetXYZ()[idxPoint].Y;
					p.z = distanceInput->GetXYZ()[idxPoint].Z;
					cloudCluster->push_back(p);
				}
			}
			viewer->updatePointCloud(cloudCluster, "cloudCluster");

			// 거리 데이터 및 충돌 정보 그리기
			for (int32_t i = 0; i < distanceInput->GetDistanceInfoSize(); i++)
			{
				pcl::PointXYZ posObject(
					distanceInput->GetDistanceInfo()[i].PosCluster.X,
					distanceInput->GetDistanceInfo()[i].PosCluster.Y,
					distanceInput->GetDistanceInfo()[i].PosCluster.Z);

				pcl::PointXYZ posCrane(
					distanceInput->GetDistanceInfo()[i].PosCrane.X,
					distanceInput->GetDistanceInfo()[i].PosCrane.Y,
					distanceInput->GetDistanceInfo()[i].PosCrane.Z);

				char name[64] = "";
				sprintf(name, "line%d", i);
				if (m_collisionOutput->GetCollisionInfo()[i] == 0)
				{
					// 정상(초록)
					viewer->addLine(posObject, posCrane, 0, 1, 0, name);
				}
				else if (m_collisionOutput->GetCollisionInfo()[i] == 1)
				{
					// 주의(노랑)
					viewer->addLine(posObject, posCrane, 1, 1, 0, name);
				}
				else if (m_collisionOutput->GetCollisionInfo()[i] == 2)
				{
					// 경고(주황)
					viewer->addLine(posObject, posCrane, 1, 0.5, 0, name);
				}
				else
				{
					// 정지(빨강)
					viewer->addLine(posObject, posCrane, 1, 0, 0, name);
				}
			}
		}
		viewer->spinOnce();
	}
}

bool IsOnVectorDirection(Eigen::Vector3f& vecStd, Eigen::Vector3f& vecInput, float_t m_angleThreshold)
{
	bool ret = false;
	float_t angle = vecStd.dot(vecInput) / sqrt(vecStd.squaredNorm() * vecInput.squaredNorm());
	angle = std::max(angle, -1.0f);
	angle = std::min(angle, 1.0f);
	angle = SHI::Math::RadianToDegree(acos(angle));
	angle = std::min(angle, 180 - angle);

	if (angle < m_angleThreshold) ret = true;
	return ret;
}

unsigned char Estimate(float_t distance, float_t v[3])
{
	unsigned char ret = 0;
	if (distance < v[0])
	{
		ret = 3;
	}
	else if (distance < v[1])
	{
		ret = 2;
	}
	else if (distance < v[2])
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}
	return ret;
}
