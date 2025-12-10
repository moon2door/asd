#include "ServiceRotorLog.h"
#include <Data/StCluster.h>
#include <Data/StLogDBInfo.h>
#include <Utility/Types.h>
#include <queue>
#include "Cvt/class_log_processing.h"

typedef std::pair<SHI::PointCloudPtr, unsigned int> PointCloudSynced;

bool ReadDbLogFile(std::vector<SHI::Data::StLogDBInfo>& vLogInfo, std::string fname);

bool ReadLogFile(std::queue<PointCloudSynced>& qPointCloud, std::string fname);

CServiceRotorLog::CServiceRotorLog()
{
	// create events
	m_waitStatus = SYNC_KEY;
	m_eCluster.Create();
	m_eDistance.Create();
	m_eCollision.Create();
	m_eKey.Create();

	// SHI Interface
	// interface to cluster processor
	GetStubXYZPoint()->Create(2);
	GetStubDBInfo()->Create(2);
	GetProxyCluster()->Create(2);
	// interface to collision manager
	GetInterfaceClientSensorProcessorManager()->Create("127.0.0.1", 8098, sizeof(SHI::Data::StCluster));
	GetInterfaceClientMonitoring()->Create("127.0.0.1", 9098, sizeof(SHI::Data::StCluster));
}


CServiceRotorLog::~CServiceRotorLog()
{
}

void CServiceRotorLog::FileOpen(const char* fname)
{
	static int countXYZ = 0;
	std::string filePath = fname;
	std::string dbfilePath = filePath.substr(0, filePath.size() - 3) + "dblog";

	// Read file
	std::vector<SHI::Data::StLogDBInfo> vDbLog;
	std::queue<PointCloudSynced> qPointCloud;
	bool bReadDbLog = ReadDbLogFile(vDbLog, dbfilePath);
	bool bReadLog = ReadLogFile(qPointCloud, filePath);

	// event for fitst send
	SetClusterEvent();
	SetDistanceEvent();
	SetCollisionEvent();
	SetKeyEvent();

	if (bReadLog)
	{
		unsigned int totalSize = qPointCloud.size();
		unsigned int curFrame = 0;

		while (qPointCloud.size())
		{
			// get point cloud
			PointCloudSynced cloudSync = qPointCloud.front();
			SHI::PointCloudPtr cloud = cloudSync.first;
			int timestamp = cloudSync.second;
			qPointCloud.pop();

			if (cloud)
			{
				// convert
				unsigned int size = (std::min)((unsigned int)cloud->size(), (unsigned int)500000);
				SHI::Data::StXYZPoints* dataXYZ = new SHI::Data::StXYZPoints;
				memset(dataXYZ, 0, sizeof(SHI::Data::StXYZPoints));
				dataXYZ->InitBuffer();
				dataXYZ->AllocXYZ(size);
				memcpy(dataXYZ->GetXYZ(), cloud->points.data(), size * sizeof(SHI::Data::StXYZ));

				// find nearest db
				SHI::Data::StDBInfo* dbinfo = new SHI::Data::StDBInfo;
				memset(dbinfo, 0, sizeof(SHI::Data::StDBInfo));
				if (vDbLog.size() > 0)
				{
					// find mating index
					unsigned int idxCur = 0;
					float tVelodyne = timestamp * 0.000001;
					for (idxCur = 0; idxCur < vDbLog.size(); idxCur++)
					{
						if (tVelodyne <= vDbLog.data()[idxCur].syncTime.rotor_1.timeVLP)
						{
							break;
						}
					}

					// get mating db info
					*dbinfo = vDbLog.at(idxCur).hookDB;
				}

				// send
				WaitForClusterSync();
				WaitForDistanceSync();
				WaitForCollisionSync();
				WaitForKeySync();
				GetStubDBInfo()->WriteData(dbinfo);
				GetStubXYZPoint()->WriteData(dataXYZ);
				printf("send xyz %d/%d \n", ++curFrame, totalSize);

				delete dbinfo;
				delete dataXYZ;
			}

		}
	}
}

void CServiceRotorLog::OnCluster(SHI::Data::StCluster* pData)
{
	SetClusterEvent();
	GetInterfaceClientSensorProcessorManager()->SendCluster(pData);
}

void CServiceRotorLog::OnDistance(SHI::Data::StDistanceSocket* pDistance)
{
	SetCollisionEvent();
}

void CServiceRotorLog::SetWaitStatus(int stat)
{
	m_waitStatus = stat;
}

bool ReadDbLogFile(std::vector<SHI::Data::StLogDBInfo>& vLogInfo, std::string fname)
{
	bool ret = false;
	FILE* fp = fopen(fname.c_str(), "rb");
	if (fp)
	{
		vLogInfo.clear();
		SHI::Data::StLogDBInfo *data = new SHI::Data::StLogDBInfo(SHI::Data::StLogSyncTime(), SHI::Data::StDBInfo());
		memset(data, 0, sizeof(SHI::Data::StLogDBInfo));

		size_t nRead = INT_MAX;
		while (nRead > 0)
		{
			nRead = fread(data, 1, sizeof(SHI::Data::StLogDBInfo), fp);
			if (nRead > 0)
			{
				vLogInfo.push_back(*data);
			}
		}


		fclose(fp);
		delete data;
		ret = true;
	}
	return ret;
}

typedef std::pair<SHI::PointCloudPtr, unsigned int> PointCloudSynced;

bool ReadLogFile(std::queue<PointCloudSynced>& qPointCloud, std::string fname)
{
	bool ret = false;
	qPointCloud.empty();
	FILE* fp = fopen(fname.c_str(), "rb");
	if (fp)
	{
		class_log_processing cvt1(1);
		class_log_processing cvt2(2);
		class_log_processing cvt3(3);
		class_log_processing cvt4(4);
		class_log_processing cvt5(5);

		struct_log_data log_data;				// Input
		struct_sub_point_cloud point_cloud;		// Output
		point_cloud.rotor_id = new unsigned char[100000];
		point_cloud.xyz = new SHI::Data::StXYZ[100000];

		SHI::PointCloudPtr cloud(new SHI::PointCloud);

		unsigned int timestamp = 0;
		while (fread(&log_data, 1, sizeof(struct_log_data), fp))
		{
			// parse data
			unsigned int nPoint = 0;
			switch (log_data.rotor_id)
			{
			case 1:
				timestamp = log_data.velodyne_packet.Timestamp;
				nPoint = cvt1.parsing_velodyne_log(log_data, &point_cloud);
				break;
			case 2:
				nPoint = cvt2.parsing_velodyne_log(log_data, &point_cloud);
				break;
			case 3:
				nPoint = cvt3.parsing_velodyne_log(log_data, &point_cloud);
				break;
			case 4:
				nPoint = cvt4.parsing_velodyne_log(log_data, &point_cloud);
				break;
			case 5:
				nPoint = cvt5.parsing_velodyne_log(log_data, &point_cloud);
				break;
			default:
				break;
			};

			if (log_data.rotor_id != 1000)
			{
				// push parsed data to current cloud
				std::vector<SHI::Point3D> curPoints;
				curPoints.resize(nPoint);
				memcpy(curPoints.data(), point_cloud.xyz, sizeof(SHI::Point3D)*nPoint);
				cloud->points.insert(cloud->points.end(), curPoints.begin(), curPoints.end());
			}
			else
			{
				// end of current cloud
				qPointCloud.push(PointCloudSynced(cloud, timestamp));

				// new cloud
				cloud = SHI::PointCloudPtr(new SHI::PointCloud);
			}
		}

		// remained data on tail of file
		//if (cloud)
		//{
		//	// end of current cloud
		//	qPointCloud.push(PointCloudSynced(cloud, timestamp));
		//}

		fclose(fp);
		delete point_cloud.rotor_id;
		delete point_cloud.xyz;
		ret = true;
	}
	return ret;
}
