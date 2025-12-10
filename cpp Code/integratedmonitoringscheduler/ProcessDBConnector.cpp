
#include <MongoDB/CMongoDB.h>
#include <Data/StCraneInfo.h>
#include <Config/CraneInfo.h>
#include <windows.h>
#include <string>
#include "ProcessDBConnector.h"

namespace IntegratedRouter
{

	std::string databaseNameLogin()
	{
		return std::string("LoginInfo");
	}

	std::string databaseName(int32_t id)
	{
		int32_t pier = SHI::GetID2Pier(id);
		int32_t crane = SHI::GetID2Crane(id);
		char buf[256] = "";
		sprintf(buf, "%s_%s", SHI::ConvPierStr(pier).c_str(), SHI::ConvCraneStr(pier, crane).c_str());
		return std::string(buf);
	}

	std::string collectionCraneInfoName()
	{
		return std::string("CraneInfo");
	}

	std::string collectionCollisionName(Routine::CTime& time)
	{
		char buf[256] = "";
		sprintf(buf, "D%04d%02d%02d_Collision", time.Year(), time.Month(), time.Day());
		return std::string(buf);
	}

	std::string collectionStatusName(Routine::CTime& time)
	{
		char buf[256] = "";
		sprintf(buf, "D%04d%02d%02d_Status", time.Year(), time.Month(), time.Day());
		return std::string(buf);
	}

	std::string collectionOperationHistory(Routine::CTime& time)
	{
		char buf[256] = "";
		sprintf(buf, "D%04d_OperationHistory", time.Year());
		return std::string(buf);
	}

	std::string collectionCollisionHistory(Routine::CTime& time)
	{
		char buf[256] = "";
		sprintf(buf, "D%04d_CollisionHistory", time.Year());
		return std::string(buf);
	}

	std::string collectionCraneAttitude()
	{
		return std::string("CraneAttitude");
	}

	std::string collectionCooperation()
	{
		return std::string("Cooperation");
	}

	std::string collectionCooperationDone(Routine::CTime& time)
	{
		char buf[256] = "";
		sprintf(buf, "D%04d%02d%02d_Cooperation", time.Year(), time.Month(), time.Day());
		return std::string(buf);
	}

	std::string collectionLogin()
	{
		return std::string("LoginInfo");
	}

	CProcessDBConnector::CProcessDBConnector()
	{
		m_db = new CMongoDB();
		m_eProcessData.Create();
		m_threadProcessDB.StartThread(&CProcessDBConnector::ProcessDB, this);
	}

	CProcessDBConnector::~CProcessDBConnector()
	{
	}

	bool CProcessDBConnector::ConnectDB(std::string ip, int32_t port)
	{
		char szPort[32] = "";
		sprintf(szPort, "%d", port);
		return reinterpret_cast<CMongoDB*>(m_db)->ConnectDB(ip, szPort) == 0;
	}

	bool CProcessDBConnector::IsConnectedDB()
	{
		return reinterpret_cast<CMongoDB*>(m_db)->IsConnected();
	}

	bool CProcessDBConnector::InsertDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.UpdateCurrentTime();

			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionCollisionName(time);
			data.payload.resize(pDistance->GetSize());
			memcpy(data.payload.data(), pDistance, pDistance->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();

			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StDistanceSocketCompressed &pDistance, uint32_t idx)
	{
		bool ret = false;
		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadDocumentsBinary(databaseName(id), collectionCollisionName(time), binData, idx);
			if (readBinary)
			{
				memcpy(&pDistance, binData.data(), binData.size());
				ret = true;
			}
		}
		m_csDB.UnLock();
		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t date)
	{
		uint32_t ret = 0;
		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			std::vector<unsigned char> binData;
			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionCollisionName(time));
		}
		m_csDB.UnLock();
		return ret;
	}

	void CProcessDBConnector::RemoveDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t day) const
	{
		bool ret = false;
		m_csDB.Lock();
		if (static_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = day;
			static_cast<CMongoDB*>(m_db)->DeleteCollection(databaseName(id), collectionCollisionName(time));
			printf("delete [%04d-%02d-%02d] %s, %s collections \n", year, month, day, databaseName(id).c_str(), collectionCollisionName(time).c_str());
		}
		m_csDB.UnLock();
	}

	bool CProcessDBConnector::InsertCraneStatus(int32_t id, SHI::Data::StCraneStatus * pStatus)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.UpdateCurrentTime();

			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionStatusName(time);
			data.payload.resize(pStatus->GetSize());
			memcpy(data.payload.data(), pStatus, pStatus->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadCraneStatus(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCraneStatus & pStatus, uint32_t idx)
	{
		bool ret = false;
		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadDocumentsBinary(databaseName(id), collectionStatusName(time), binData, idx);
			if (readBinary)
			{
				if (binData.size() == pStatus.GetSize())
				{
					memcpy(&pStatus, binData.data(), binData.size());
					ret = true;
				}
			}
		}
		m_csDB.UnLock();

		return ret;
	}

	bool CProcessDBConnector::InsertCraneInfo(int32_t id)
	{
		bool ret = false;
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			int32_t pier = SHI::GetID2Pier(id);
			int32_t crane = SHI::GetID2Crane(id);
			SHI::Data::StCraneInfo info;
			memset(&info, 0, sizeof(info));
			info.YardNum = pier;
			info.CraneNum = crane;
			sprintf(info.YardName, "%s", SHI::ConvPierStr(pier).c_str());
			sprintf(info.CraneName, "%s", SHI::ConvCraneStr(pier, crane).c_str());

			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = "CraneInfo";
			data.payload.resize(info.GetSize());
			memcpy(data.payload.data(), &info, info.GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadCraneInfo(int32_t id, SHI::Data::StCraneInfo& info)
	{
		bool ret = false;
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			std::vector<unsigned char> binData;
			m_csDB.Lock();
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadLastDocumentBinary(databaseName(id), collectionCraneInfoName(), binData);
			m_csDB.UnLock();

			if (readBinary)
			{
				if (binData.size() == info.GetSize())
				{
					memcpy(&info, binData.data(), binData.size());
					ret = true;
				}
			}
		}
		return ret;
	}

	void CProcessDBConnector::ProcessDB()
	{
		std::queue<StMessageDB> qInsertData;
		CMongoDB* db = reinterpret_cast<CMongoDB*>(m_db);

		while (m_threadProcessDB.IsRunThread())
		{
			if (m_eProcessData.WaitForEvent(100))
			{
				qInsertData.empty();

				m_csQueue.Lock();
				qInsertData.swap(m_qMessageData);
				m_csQueue.UnLock();

				//printf("m_qMessageData %d\n", qInsertData.size());
				m_csDB.Lock();
				//printf("m_csDB.Lock\n");
				while (!qInsertData.empty())
				{
					if (qInsertData.size() < 100)
					{
						if (db)
						{
							StMessageDB data = qInsertData.front();
							switch (data.type)
							{
							case DB_INSERT:
								db->InsertDocumentBinary(data.dbName, data.collectionName, data.payload);
								break;
							case DB_UPDATE:
								db->UpdateDocumentBinary(data.dbName, data.collectionName, data.payload);
								break;
							default:
								break;
							}
						}
					}
					qInsertData.pop();
				}
				m_csDB.UnLock();
				//printf("m_csDB.UnLock\n");

			}
		}
	}

	bool CProcessDBConnector::InsertOperationHistory(int32_t id, int32_t year, SHI::Data::StOperationHistory* history)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionOperationHistory(time);
			data.payload.resize(history->GetSize());
			memcpy(data.payload.data(), history, history->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::UpdateOperationHistory(int32_t id, int32_t year, SHI::Data::StOperationHistory* history)
	{

		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			StMessageDB data;
			data.type = DB_UPDATE;
			data.dbName = databaseName(id);
			data.collectionName = collectionOperationHistory(time);
			data.payload.resize(history->GetSize());
			memcpy(data.payload.data(), history, history->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadOperationHistory(int32_t id, int32_t year, SHI::Data::StOperationHistory &history)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadLastDocumentBinary(databaseName(id), collectionOperationHistory(time), binData);
			if (readBinary)
			{
				if (binData.size() == history.GetSize())
				{
					memcpy(&history, binData.data(), binData.size());
					ret = true;
				}
			}
		}
		m_csDB.UnLock();

		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumOperationHistory(int32_t id, int32_t year)
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionOperationHistory(time));
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::InsertCollisionHistory(int32_t id, int32_t year, SHI::Data::StCollisionHistory* history)
	{
		bool ret = false;
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionCollisionHistory(time);
			data.payload.resize(history->GetSize());
			memcpy(data.payload.data(), history, history->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::UpdateCollisionHistory(int32_t id, int32_t year, SHI::Data::StCollisionHistory* history)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			StMessageDB data;
			data.type = DB_UPDATE;
			data.dbName = databaseName(id);
			data.collectionName = collectionCollisionHistory(time);
			data.payload.resize(history->GetSize());
			memcpy(data.payload.data(), history, history->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadCollisionHistory(int32_t id, int32_t year, SHI::Data::StCollisionHistory &history)
	{
		bool ret = false;
		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadLastDocumentBinary(databaseName(id), collectionCollisionHistory(time), binData);

			if (readBinary)
			{
				if (binData.size() == history.GetSize())
				{
					memcpy(&history, binData.data(), binData.size());
					ret = true;
				}
			}
		}
		m_csDB.UnLock();
		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumCollisionHistory(int32_t id, int32_t year)
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;

			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionCollisionHistory(time));
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::InsertCraneAttitude(int32_t id, SHI::Data::StCraneAttitude* attitude)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionCraneAttitude();
			data.payload.resize(attitude->GetSize());
			memcpy(data.payload.data(), attitude, attitude->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::UpdateCraneAttitude(int32_t id, SHI::Data::StCraneAttitude* attitude)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			StMessageDB data;
			data.type = DB_UPDATE;
			data.dbName = databaseName(id);
			data.collectionName = collectionCraneAttitude();
			data.payload.resize(attitude->GetSize());
			memcpy(data.payload.data(), attitude, attitude->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadCraneAttitude(int32_t id, SHI::Data::StCraneAttitude &attitude)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadLastDocumentBinary(databaseName(id), collectionCraneAttitude(), binData);
			if (readBinary)
			{
				if (binData.size() == attitude.GetSize())
				{
					memcpy(&attitude, binData.data(), binData.size());
					ret = true;
				}
			}
		}
		m_csDB.UnLock();

		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumCraneAttitude(int32_t id)
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionCraneAttitude());
		}
		m_csDB.UnLock();
		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumCraneStatus(int32_t id, int32_t year, int32_t month, int32_t date)
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionStatusName(time));
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::InsertCooperation(int32_t id, SHI::Data::StCooperationMode* cooperation)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionCooperation();
			data.payload.resize(cooperation->GetSize());
			memcpy(data.payload.data(), cooperation, cooperation->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::UpdateCooperation(int32_t id, SHI::Data::StCooperationMode* cooperation)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			StMessageDB data;
			data.type = DB_UPDATE;
			data.dbName = databaseName(id);
			data.collectionName = collectionCooperation();
			data.payload.resize(cooperation->GetSize());
			memcpy(data.payload.data(), cooperation, cooperation->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadCooperation(int32_t id, SHI::Data::StCooperationMode &cooperation, uint32_t idx)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadDocumentsBinary(databaseName(id), collectionCooperation(), binData, idx);
			if (readBinary)
			{
				if (binData.size() == cooperation.GetSize())
				{
					memcpy(&cooperation, binData.data(), binData.size());
					ret = true;
				}
			}
		}
		m_csDB.UnLock();

		return ret;
	}

	bool CProcessDBConnector::RemoveCooperation(int32_t id, uint32_t idx)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			ret = reinterpret_cast<CMongoDB*>(m_db)->DeleteDocument(databaseName(id), collectionCooperation(), idx);
		}
		m_csDB.UnLock();

		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumCooperation(int32_t id)
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionCooperation());
		}
		m_csDB.UnLock();
		return ret;
	}


	bool CProcessDBConnector::InsertCooperationDone(int32_t id, SHI::Data::StCooperationMode* cooperation)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.UpdateCurrentTime();
			time.m_time.tm_year = cooperation->startTime.year;
			time.m_time.tm_mon = cooperation->startTime.month;
			time.m_time.tm_mday = cooperation->startTime.date;
			time.m_time.tm_hour = cooperation->startTime.hour;
			time.m_time.tm_min = cooperation->startTime.min;

			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseName(id);
			data.collectionName = collectionCooperationDone(time);
			data.payload.resize(cooperation->GetSize());
			memcpy(data.payload.data(), cooperation, cooperation->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::UpdateCooperationDone(int32_t id, SHI::Data::StCooperationMode* cooperation)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.UpdateCurrentTime();
			time.m_time.tm_year = cooperation->startTime.year;
			time.m_time.tm_mon = cooperation->startTime.month;
			time.m_time.tm_mday = cooperation->startTime.date;
			time.m_time.tm_hour = cooperation->startTime.hour;
			time.m_time.tm_min = cooperation->startTime.min;

			StMessageDB data;
			data.type = DB_UPDATE;
			data.dbName = databaseName(id);
			data.collectionName = collectionCooperationDone(time);
			data.payload.resize(cooperation->GetSize());
			memcpy(data.payload.data(), cooperation, cooperation->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}
	
	bool CProcessDBConnector::ReadCooperationDone(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCooperationMode &cooperation, uint32_t idx)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadDocumentsBinary(databaseName(id), collectionCooperationDone(time), binData, idx);
			if (readBinary)
			{
				memcpy(&cooperation, binData.data(), binData.size());
				ret = true;
			}
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::RemoveCooperationDone(int32_t id, int32_t year, int32_t month, int32_t date, uint32_t idx)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			ret = reinterpret_cast<CMongoDB*>(m_db)->DeleteDocument(databaseName(id), collectionCooperationDone(time), idx);
		}
		m_csDB.UnLock();

		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumCooperationDone(int32_t id, int32_t year, int32_t month, int32_t date)
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			time.m_time.tm_year = year;
			time.m_time.tm_mon = month;
			time.m_time.tm_mday = date;

			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseName(id), collectionCooperationDone(time));
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::InsertLoginInfo(SHI::Data::StLoginInfo* info)
	{
		bool ret = false;

		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			StMessageDB data;
			data.type = DB_INSERT;
			data.dbName = databaseNameLogin();
			data.collectionName = collectionLogin();
			data.payload.resize(info->GetSize());
			memcpy(data.payload.data(), info, info->GetSize());

			m_csQueue.Lock();
			m_qMessageData.push(data);
			m_csQueue.UnLock();

			m_eProcessData.SetEvent();
			ret = true;
		}
		return ret;
	}

	bool CProcessDBConnector::ReadLoginInfo(SHI::Data::StLoginInfo &info, uint32_t idx)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			std::vector<unsigned char> binData;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadDocumentsBinary(databaseNameLogin(), collectionLogin(), binData, idx);
			if (readBinary)
			{
				memcpy(&info, binData.data(), binData.size());
				ret = true;
			}
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::ReadLoginInfo(std::vector<SHI::Data::StLoginInfo> &retInfo)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			std::vector<std::vector<unsigned char>> data;
			bool readBinary = reinterpret_cast<CMongoDB*>(m_db)->ReadDocumentsBinary(databaseNameLogin(), collectionLogin(), data);
			if (readBinary)
			{
				for (uint32_t i = 0; i < data.size(); i++)
				{
					SHI::Data::StLoginInfo info;
					memcpy(&info, data[i].data(), data[i].size());
					retInfo.push_back(info);
				}
				ret = true;
			}
		}
		m_csDB.UnLock();
		return ret;
	}

	bool CProcessDBConnector::RemoveLoginInfo(uint32_t idx)
	{
		bool ret = false;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			Routine::CTime time;
			std::vector<std::vector<unsigned char>> data;
			std::vector<unsigned char> binData;
			ret = reinterpret_cast<CMongoDB*>(m_db)->DeleteDocument(databaseNameLogin(), collectionLogin(), idx);
		}
		m_csDB.UnLock();

		return ret;
	}

	uint32_t CProcessDBConnector::ReadNumLoginInfo()
	{
		uint32_t ret = 0;

		m_csDB.Lock();
		if (reinterpret_cast<CMongoDB*>(m_db)->IsConnected())
		{
			ret = reinterpret_cast<CMongoDB*>(m_db)->ReadNumDocuments(databaseNameLogin(), collectionLogin());
		}
		m_csDB.UnLock();
		return ret;
	}

}