//#include "stdafx.h"
//#include "resource.h"
//
//#include <Utility/mathematics.h>
//#include <Utility/Types.h>
//#include <Config/CraneInfo.h>
//#include <Routine/include/Base/CTime.h>
//
//#include "CollisionProcessorManagerDlg.h"
//
//inline char* GetCraneName(int32_t craneNum)
//{
//	char* pName;
//	switch (craneNum)
//	{
//	case 0:
//		pName = "CS8000003";
//		break;
//	case 1:
//		pName = "CG0300006";
//		break;
//	case 2:
//		pName = "CG0300007";
//		break;
//	case 3:
//		pName = "CO0160012";
//		break;
//	case 4:
//		pName = "CO0120010";
//		break;
//	case 5:
//		pName = "CO0160008";
//		break;
//	case 6:
//		pName = "CO0120011";
//		break;
//	case 7:
//		pName = "CO0160011";
//		break;
//	default:
//		pName = "Unknown";
//		break;
//	}
//
//	return pName;
//}
//
//void CCollisionProcessorManagerDlg::OnTimerConnectMongoDB()
//{
//	std::string dbIP = Routine::GetConfigString("DB", "IP", "60.100.91.175");
//	int32_t dbPort = Routine::GetConfigInt("DB", "Port", 27017);
//	if (m_dbConnector_mongo->ConnectDB(dbIP, dbPort))
//	{
//		m_dbConnector_mongo->RequestCraneInfo();
//
//		SetDlgItemText(IDC_TXT_DB_CONNECTION, "Connected");
//
//		StartTimerUpdateHeading();
//		StartTimerUpdateCooperation();
//		m_dbConnector_mongo->ClearCraneDataAll();
//	}
//}
//
//void CCollisionProcessorManagerDlg::OnTimerUpdateCraneInfoFromMongoDB()
//{
//	if (m_objSensorManager->GetSocketTCP()->GetClientInfo().size())
//	{
//		/*m_dbConnector_mongo->RequestCraneInfo(m_dbConnector_mongo->GetCraneNumber(GetCraneName(GetRotorStatus().Crane)));
//
//		SHI::Object::StCraneInfoDB tmp = m_dbConnector_mongo->GetHookInfo();
//		SHI::Data::StDBInfo data;
//		memcpy(&data, &tmp, sizeof(data));
//		data.latitude = m_dbConnector_mongo->GetLat();
//		data.logitude = m_dbConnector_mongo->GetLon();
//		GetStubDBInfo()->WriteData(&data);*/
//	}
//}
//
//void CCollisionProcessorManagerDlg::OnTimerUpdateCooperationFromMongoDB()
//{
//	if (m_objSensorManager->GetSocketTCP()->GetClientInfo().size())
//	{
//		//m_dbConnector_mongo->RequestCooperation(m_dbConnector_mongo->GetCraneNumber(GetCraneName(GetRotorStatus().Crane)));
//	}
//}
//
//void CCollisionProcessorManagerDlg::OnUpdatedDistanceToMongoDB()
//{
//	uint32_t nClusterSize = GetDistance()->GetClusterInfoSize();
//	uint32_t nClusterLabel = GetDistance()->GetLabelSize();
//	SHI::Data::StClusterInfo* pClusterInfo = GetDistance()->GetClusterInfo();
//	SHI::Data::StDistanceInfo* pDistanceInfo = GetDistance()->GetDistanceInfo();
//	unsigned char* pClusterLabel = GetDistance()->GetLabel();
//	SHI::Data::StXYZ* pXYZ = GetDistance()->GetXYZ();
//	uint32_t* pIndices = GetDistance()->GetClusterIndices();
//	unsigned char* pCollisionInfo = m_dataCollisionInformation->GetCollisionInfo();
//	float_t						azimuth = -GetDistance()->attitude.pose[SHI::Pier7::LLC_TOWER];
//
//	struct St2Point
//	{
//		float_t minX;
//		float_t minY;
//		float_t minZ;
//		float_t maxX;
//		float_t maxY;
//		float_t maxZ;
//
//		float_t Distance;
//		unsigned char CollisionInfo;
//	};
//
//	struct StResult
//	{
//		float_t point[24];
//	};
//
//	// 병합 추가 필요.
//	int32_t offX = 0;
//	int32_t offY = 0;
//	int32_t offZ = 0;
//
//	switch (GetRotorStatus().Crane)
//	{
//	case 0:
//		offX = 0;
//		offY = 69.5;
//		offZ = 70 + 2;
//		break;
//	case 1:
//		offX = 0;
//		offY = 0;
//		offZ = 37 + 0;
//		break;
//	case 2:
//		offX = 0;
//		offY = 0;
//		offZ = 37 + 0;
//		break;
//	/*case 3:
//		offX = 0;
//		offY = 0;
//		offZ = 52.3215f - m_dbConnector_mongo->GetHeight();
//		azimuth = -m_dbConnector_mongo->GetHeading();
//		break;
//	case 4:
//		offX = 0;
//		offY = 0;
//		offZ = 42.46f - m_dbConnector_mongo->GetHeight();
//		azimuth = -m_dbConnector_mongo->GetHeading();
//		break;
//	case 5:
//		offX = 0;
//		offY = 0;
//		offZ = 51.3215f - m_dbConnector_mongo->GetHeight();
//		azimuth = -m_dbConnector_mongo->GetHeading();
//		break;
//	case 6:
//		offX = 0;
//		offY = 0;
//		offZ = 54.f - m_dbConnector_mongo->GetHeight();
//		azimuth = -m_dbConnector_mongo->GetHeading();
//		break;
//	case 7:
//		offX = 0;
//		offY = 0;
//		offZ = 48.5f - m_dbConnector_mongo->GetHeight();
//		azimuth = -m_dbConnector_mongo->GetHeading();
//		break;*/
//	default:
//		break;
//	}
//
//	if (GetRotorStatus().Crane == 0)
//	{
//		std::vector<St2Point> v2Points;
//		Routine::CTime time;
//		char strTime[32] = "";
//		sprintf(strTime, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
//			time.Year(), time.Month(), time.Day(),
//			time.Hour(), time.Min(), time.Second(), time.MilliSecond());
//
//		for (uint32_t nCluster = 0; nCluster < nClusterSize; nCluster++)
//		{
//			if (nClusterLabel != nClusterSize) break;
//			if (pClusterLabel[nCluster] != SHI::LABEL_OBJECT) break;
//
//			St2Point pt{ FLT_MAX, FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX };
//
//			// 클러스터 탐색 : 최대, 최소점
//			int32_t clusterIndex = pClusterInfo[nCluster].Index;
//			for (uint32_t nPoint = 0; nPoint < pClusterInfo[nCluster].Size; nPoint++)
//			{
//				if (pXYZ[pIndices[clusterIndex + nPoint]].X < pt.minX) pt.minX = pXYZ[pIndices[clusterIndex + nPoint]].X;
//				if (pXYZ[pIndices[clusterIndex + nPoint]].X > pt.maxX) pt.maxX = pXYZ[pIndices[clusterIndex + nPoint]].X;
//
//				if (-pXYZ[pIndices[clusterIndex + nPoint]].Y < pt.minY) pt.minY = -pXYZ[pIndices[clusterIndex + nPoint]].Y;
//				if (-pXYZ[pIndices[clusterIndex + nPoint]].Y > pt.maxY) pt.maxY = -pXYZ[pIndices[clusterIndex + nPoint]].Y;
//
//				if (pXYZ[pIndices[clusterIndex + nPoint]].Z < pt.minZ) pt.minZ = pXYZ[pIndices[clusterIndex + nPoint]].Z;
//				if (pXYZ[pIndices[clusterIndex + nPoint]].Z > pt.maxZ) pt.maxZ = pXYZ[pIndices[clusterIndex + nPoint]].Z;
//			}
//
//
//			// DB 추가 정보 입력
//			pt.Distance = pDistanceInfo[nCluster].Distance;
//			pt.CollisionInfo = pCollisionInfo[nCluster];
//
//			v2Points.push_back(pt);
//		}
//
//		for (uint32_t i = 0; i < v2Points.size() && i < 5; i++)
//		{
//			StResult res;
//			StResult resCvt;
//
//			// 8점 생성
//			res.point[0] = v2Points[i].maxX; res.point[1] = v2Points[i].maxY; res.point[2] = v2Points[i].maxZ;
//			res.point[3] = v2Points[i].minX; res.point[4] = v2Points[i].maxY; res.point[5] = v2Points[i].maxZ;
//			res.point[6] = v2Points[i].minX; res.point[7] = v2Points[i].minY; res.point[8] = v2Points[i].maxZ;
//			res.point[9] = v2Points[i].maxX; res.point[10] = v2Points[i].minY; res.point[11] = v2Points[i].maxZ;
//
//			res.point[12] = v2Points[i].maxX; res.point[13] = v2Points[i].maxY; res.point[14] = v2Points[i].minZ;
//			res.point[15] = v2Points[i].minX; res.point[16] = v2Points[i].maxY; res.point[17] = v2Points[i].minZ;
//			res.point[18] = v2Points[i].minX; res.point[19] = v2Points[i].minY; res.point[20] = v2Points[i].minZ;
//			res.point[21] = v2Points[i].maxX; res.point[22] = v2Points[i].minY; res.point[23] = v2Points[i].minZ;
//
//			// 축 변환
//			resCvt.point[2] = res.point[0] + offX;
//			resCvt.point[0] = (res.point[1] + offY);
//			resCvt.point[1] = res.point[2] + offZ;
//
//			resCvt.point[5] = res.point[3] + offX;
//			resCvt.point[3] = res.point[4] + offY;
//			resCvt.point[4] = res.point[5] + offZ;
//
//			resCvt.point[8] = res.point[6] + offX;
//			resCvt.point[6] = res.point[7] + offY;
//			resCvt.point[7] = res.point[8] + offZ;
//
//			resCvt.point[11] = res.point[9] + offX;
//			resCvt.point[9] = res.point[10] + offY;
//			resCvt.point[10] = res.point[11] + offZ;
//
//			resCvt.point[14] = res.point[12] + offX;
//			resCvt.point[12] = res.point[13] + offY;
//			resCvt.point[13] = res.point[14] + offZ;
//
//			resCvt.point[17] = res.point[15] + offX;
//			resCvt.point[15] = res.point[16] + offY;
//			resCvt.point[16] = res.point[17] + offZ;
//
//			resCvt.point[20] = res.point[18] + offX;
//			resCvt.point[18] = res.point[19] + offY;
//			resCvt.point[19] = res.point[20] + offZ;
//
//			resCvt.point[23] = res.point[21] + offX;
//			resCvt.point[21] = res.point[22] + offY;
//			resCvt.point[22] = res.point[23] + offZ;
//
//
//			// 0:정상, 1:주의, 2:경보, else :정지
//			switch (v2Points[i].CollisionInfo)
//			{
//			case 0:	// 정상
//				break;
//			case 1:	// 주의
//				break;
//			case 2:	// 경보
//				break;
//			default:	// 정지
//				break;
//			}
//
//			//printf("%d ( %s ) \n", i, m_dbConnector_mongo->PushData(GetCraneName(GetRotorStatus().NumCrane), i, strTime, resCvt.point, v2Points[i].Distance, v2Points[i].CollisionInfo) ? "True" : "False");
//		}
//	}
//	else
//	{
//		std::vector<St2Point> v2Points;
//		Routine::CTime time;
//		char strTime[32] = "";
//		sprintf(strTime, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
//			time.Year(), time.Month(), time.Day(),
//			time.Hour(), time.Min(), time.Second(), time.MilliSecond());
//
//		for (uint32_t nCluster = 0; nCluster < nClusterSize; nCluster++)
//		{
//			if (nClusterLabel != nClusterSize) break;
//			if (pClusterLabel[nCluster] != SHI::LABEL_OBJECT) break;
//
//			St2Point pt{ FLT_MAX, FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX };
//
//			// 클러스터 탐색 : 최대, 최소점
//			float_t rx = 0, ry = 0;
//			int32_t clusterIndex = pClusterInfo[nCluster].Index;
//			for (uint32_t nPoint = 0; nPoint < pClusterInfo[nCluster].Size; nPoint++)
//			{
//				// 회전
//				rx = (float_t)SHI::Math::GetRotateX(-azimuth, pXYZ[pIndices[clusterIndex + nPoint]].X, pXYZ[pIndices[clusterIndex + nPoint]].Y);
//				ry = (float_t)SHI::Math::GetRotateY(-azimuth, pXYZ[pIndices[clusterIndex + nPoint]].X, pXYZ[pIndices[clusterIndex + nPoint]].Y);
//
//				// 최대, 최소 비교
//				if (rx < pt.minX) pt.minX = rx;
//				if (ry < pt.minY) pt.minY = ry;
//				if (pXYZ[pIndices[clusterIndex + nPoint]].Z < pt.minZ) pt.minZ = pXYZ[pIndices[clusterIndex + nPoint]].Z;
//
//				if (rx > pt.maxX) pt.maxX = rx;
//				if (ry > pt.maxY) pt.maxY = ry;
//				if (pXYZ[pIndices[clusterIndex + nPoint]].Z > pt.maxZ) pt.maxZ = pXYZ[pIndices[clusterIndex + nPoint]].Z;
//			}
//
//			// DB 추가 정보 입력
//			pt.Distance = pDistanceInfo[nCluster].Distance;
//			pt.CollisionInfo = pCollisionInfo[nCluster];
//
//			v2Points.push_back(pt);
//		}
//
//		for (uint32_t i = 0; i < v2Points.size() && i < 5; i++)
//		{
//			StResult res;
//			StResult resCvt;
//
//			// 8점 생성
//			res.point[0] = v2Points[i].maxX; res.point[1] = v2Points[i].maxY; res.point[2] = v2Points[i].maxZ;
//			res.point[3] = v2Points[i].minX; res.point[4] = v2Points[i].maxY; res.point[5] = v2Points[i].maxZ;
//			res.point[6] = v2Points[i].minX; res.point[7] = v2Points[i].minY; res.point[8] = v2Points[i].maxZ;
//			res.point[9] = v2Points[i].maxX; res.point[10] = v2Points[i].minY; res.point[11] = v2Points[i].maxZ;
//
//			res.point[12] = v2Points[i].maxX; res.point[13] = v2Points[i].maxY; res.point[14] = v2Points[i].minZ;
//			res.point[15] = v2Points[i].minX; res.point[16] = v2Points[i].maxY; res.point[17] = v2Points[i].minZ;
//			res.point[18] = v2Points[i].minX; res.point[19] = v2Points[i].minY; res.point[20] = v2Points[i].minZ;
//			res.point[21] = v2Points[i].maxX; res.point[22] = v2Points[i].minY; res.point[23] = v2Points[i].minZ;
//
//			memset(&resCvt, 0, sizeof(resCvt));
//
//			//////////////////////////////////////////////////////////////////////////
//			// 역회전 적용
//			float_t _pt1x = SHI::Math::GetRotateX(azimuth, res.point[0], res.point[1]);
//			float_t _pt1y = SHI::Math::GetRotateY(azimuth, res.point[0], res.point[1]);
//
//			float_t _pt2x = SHI::Math::GetRotateX(azimuth, res.point[3], res.point[4]);
//			float_t _pt2y = SHI::Math::GetRotateY(azimuth, res.point[3], res.point[4]);
//
//			float_t _pt3x = SHI::Math::GetRotateX(azimuth, res.point[6], res.point[7]);
//			float_t _pt3y = SHI::Math::GetRotateY(azimuth, res.point[6], res.point[7]);
//
//			float_t _pt4x = SHI::Math::GetRotateX(azimuth, res.point[9], res.point[10]);
//			float_t _pt4y = SHI::Math::GetRotateY(azimuth, res.point[9], res.point[10]);
//
//			res.point[0] = _pt1x; res.point[1] = _pt1y;
//			res.point[3] = _pt2x; res.point[4] = _pt2y;
//			res.point[6] = _pt3x; res.point[7] = _pt3y;
//			res.point[9] = _pt4x; res.point[10] = _pt4y;
//
//			res.point[12] = _pt1x; res.point[13] = _pt1y;
//			res.point[15] = _pt2x; res.point[16] = _pt2y;
//			res.point[18] = _pt3x; res.point[19] = _pt3y;
//			res.point[21] = _pt4x; res.point[22] = _pt4y;
//			//////////////////////////////////////////////////////////////////////////
//
//
//			// 축 변환
//			resCvt.point[0] = -(res.point[12] + offX);
//			resCvt.point[2] = -(res.point[13] + offY);
//			resCvt.point[1] = res.point[14] + offZ;
//
//			resCvt.point[3] = -(res.point[15] + offX);
//			resCvt.point[5] = -(res.point[16] + offY);
//			resCvt.point[4] = res.point[17] + offZ;
//
//			resCvt.point[6] = -(res.point[18] + offX);
//			resCvt.point[8] = -(res.point[19] + offY);
//			resCvt.point[7] = res.point[20] + offZ;
//
//			resCvt.point[9] = -(res.point[21] + offX);
//			resCvt.point[11] = -(res.point[22] + offY);
//			resCvt.point[10] = res.point[23] + offZ;
//
//			resCvt.point[12] = -(res.point[0] + offX);
//			resCvt.point[14] = -(res.point[1] + offY);
//			resCvt.point[13] = res.point[2] + offZ;
//
//			resCvt.point[15] = -(res.point[3] + offX);
//			resCvt.point[17] = -(res.point[4] + offY);
//			resCvt.point[16] = res.point[5] + offZ;
//
//			resCvt.point[18] = -(res.point[6] + offX);
//			resCvt.point[20] = -(res.point[7] + offY);
//			resCvt.point[19] = res.point[8] + offZ;
//
//			resCvt.point[21] = -(res.point[9] + offX);
//			resCvt.point[23] = -(res.point[10] + offY);
//			resCvt.point[22] = res.point[11] + offZ;
//
//
//			// 0:정상, 1:주의, 2:경보, else :정지
//			switch (v2Points[i].CollisionInfo)
//			{
//			case 0:	// 정상
//				break;
//			case 1:	// 주의
//				break;
//			case 2:	// 경보
//				break;
//			default:	// 정지
//				break;
//			}
//
//			//printf("%d ( %s ) \n", i, m_dbConnector_mongo->PushData(GetCraneName(GetRotorStatus().NumCrane), i, strTime, resCvt.point, v2Points[i].Distance, v2Points[i].CollisionInfo) ? "True" : "False");
//		}
//	}
//}