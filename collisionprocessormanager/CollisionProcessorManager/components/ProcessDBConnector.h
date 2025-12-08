#pragma once
#include <Data/StCraneInfo.h>
#include <Data/StOperationHistory.h>
#include <Data/StCollisionHistory.h>
#include <Data/StCraneAttitude.h>
#include <Data/StDistanceSocketCompressed.h>
#include <Data/StCraneStatus.h>
#include <Data/StCooperationMode.h>
#include <Data/StLoginInfo.h>

#include <Routine/Include/Base/CTime.h>
#include <Routine/Include/Base/CTimer.h>//pjh
#include <Routine/Include/Base/CCriticalSection.h>
#include <Routine/Include/Base/CThread.h>
#include <Routine/Include/Base/CEvent.h>
#include <queue>
#include <string>

#include <MongoDB/CMongoDB.h>

struct StCraneInfo
{
	int		nCID;
	char	strName[20];
};

struct StCoop
{
	StCoop(int _idx, int _cid) : idx(_idx), cid(_cid) {}

	int idx;
	int cid;
};

struct StCraneInfoDB
{
	union
	{
		float vlu[20];
		struct
		{
			float hookHeight1;
			float hookHeight2;
			float hookHeight3;
			float posTrolly1;
			float posTrolly2;
			float reserved1[2];
			float weight1;
			float weight2;
			float weight3;
			float weightTotal;
			int statData;
			int statNewtwork;
			float reserved2[7];
		}gc;
		struct
		{
			float hookHeight1;
			float hookHeight2;
			float posDriving;
			float angleAzimuth;
			int statData;
			int statNewtwork;
			float reserved[14];
		}llc;
		struct
		{
			float hookHeight;
			float azimuth;
			float posTrolly;
			float maxWeight;
			float weight;
			float weightSub;
			float weightTotal;
			int statData;
			int statNewtwork;
			float reserved[11];
		}tc;
	}hook;
};

	std::string databaseNameLogin();

	std::string databaseName(int32_t id);

	std::string collectionCraneInfoName();

	std::string collectionCollisionName(Routine::CTime& time);

	std::string collectionStatusName(Routine::CTime& time);

	std::string collectionOperationHistory(Routine::CTime& time);

	std::string collectionCollisionHistory(Routine::CTime& time);

	std::string collectionCraneAttitude();

	std::string collectionCooperation();

	std::string collectionCooperationDone(Routine::CTime& time);

	std::string collectionCooperationCancled(Routine::CTime& time);

	std::string collectionLogin();

	class CProcessDBConnector
	{
	public:
		CProcessDBConnector();

		~CProcessDBConnector();

		bool ConnectDB(std::string ip, int32_t port);

		bool IsConnectedDB();

		//pjh
		bool RequestCraneInfo();

		bool ClearCraneDataAll();
		//

		// Distance data
		bool InsertDistanceCompressed(int32_t id, SHI::Data::StDistanceSocketCompressed* pDistance);

		bool ReadDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StDistanceSocketCompressed& pDistance, uint32_t idx);

		bool ReadDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StDistanceSocketCompressed& pDistance, SHI::Data::StDateTime& timestamp, uint32_t idx);

		uint32_t ReadNumDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t date);

		bool FindDistanceCompressed(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StDistanceSocketCompressed& pDistance, SHI::Data::StDateTime findTime);

		// Status data
		bool InsertCraneStatus(int32_t id, SHI::Data::StCraneStatus* pStatus);

		bool ReadCraneStatus(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCraneStatus& pStatus, uint32_t idx);

		bool ReadCraneStatus(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCraneStatus& pStatus, SHI::Data::StDateTime& timestamp, uint32_t idx);

		uint32_t ReadNumCraneStatus(int32_t id, int32_t year, int32_t month, int32_t date);

		bool FindStatus(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCraneStatus& pStatus, SHI::Data::StDateTime findTime);

		// CraneInfo
		bool InsertCraneInfo(int32_t id);

		bool ReadCraneInfo(int32_t id, SHI::Data::StCraneInfo& info);

		// Operation history
		bool InsertOperationHistory(int32_t id, int32_t year, SHI::Data::StOperationHistory* history);

		bool UpdateOperationHistory(int32_t id, int32_t year, SHI::Data::StOperationHistory* history);

		bool ReadOperationHistory(int32_t id, int32_t year, SHI::Data::StOperationHistory& history);

		uint32_t ReadNumOperationHistory(int32_t id, int32_t year);

		// Collision history
		bool InsertCollisionHistory(int32_t id, int32_t year, SHI::Data::StCollisionHistory* history);

		bool UpdateCollisionHistory(int32_t id, int32_t year, SHI::Data::StCollisionHistory* history);

		bool ReadCollisionHistory(int32_t id, int32_t year, SHI::Data::StCollisionHistory& history);

		uint32_t ReadNumCollisionHistory(int32_t id, int32_t year);

		// Crane attitude
		bool InsertCraneAttitude(int32_t id, SHI::Data::StCraneAttitude* attitude);

		bool UpdateCraneAttitude(int32_t id, SHI::Data::StCraneAttitude* attitude);

		bool ReadCraneAttitude(int32_t id, SHI::Data::StCraneAttitude& attitude);

		uint32_t ReadNumCraneAttitude(int32_t id);

		// Cooperation
		bool InsertCooperation(int32_t id, SHI::Data::StCooperationMode* cooperation);

		bool UpdateCooperation(int32_t id, SHI::Data::StCooperationMode* cooperation);

		bool ReadCooperation(int32_t id, SHI::Data::StCooperationMode& cooperation, uint32_t idx);

		bool RemoveCooperation(int32_t id, uint32_t idx);

		uint32_t ReadNumCooperation(int32_t id);

		// CooperationDone
		bool InsertCooperationDone(int32_t id, SHI::Data::StCooperationMode* cooperation);

		bool UpdateCooperationDone(int32_t id, SHI::Data::StCooperationMode* cooperation);

		bool ReadCooperationDone(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCooperationMode& cooperation, uint32_t idx);

		bool RemoveCooperationDone(int32_t id, int32_t year, int32_t month, int32_t date, uint32_t idx);

		uint32_t ReadNumCooperationDone(int32_t id, int32_t year, int32_t month, int32_t date);

		// CooperationDoneCancled
		bool InsertCooperationCancled(int32_t id, SHI::Data::StCooperationMode* cooperation);

		bool UpdateCooperationCancled(int32_t id, SHI::Data::StCooperationMode* cooperation);

		bool ReadCooperationCancled(int32_t id, int32_t year, int32_t month, int32_t date, SHI::Data::StCooperationMode& cooperation, uint32_t idx);

		bool RemoveCooperationCancled(int32_t id, int32_t year, int32_t month, int32_t date, uint32_t idx);

		uint32_t ReadNumCooperationCancled(int32_t id, int32_t year, int32_t month, int32_t date);

		// Login Info
		bool InsertLoginInfo(SHI::Data::StLoginInfo* info);

		bool ReadLoginInfo(SHI::Data::StLoginInfo& info, uint32_t idx);

		bool ReadLoginInfo(std::vector<SHI::Data::StLoginInfo>& retInfo);

		bool RemoveLoginInfo(uint32_t idx);

		uint32_t ReadNumLoginInfo();

	private:
		CMongoDB* m_db;
		Routine::CCriticalSection m_csQueue;
		Routine::CCriticalSection m_csDB;
		//pjh
		float		m_craneHeading;
		float		m_craneTrolley;
		float		m_craneHeight;

		float		m_gpsLat;
		float		m_gpsLon;

		StCraneInfoDB m_craneInfo;

		std::vector<StCraneInfo>		m_vCraneInfo;
		SHI::Data::StCooperationInfo	m_cooperationInfo;
		Routine::CCriticalSection		m_csCooperation;
		//

		enum
		{
			DB_INSERT,
			DB_UPDATE
		};

		struct StMessageDB
		{
			unsigned char type;
			std::string dbName;
			std::string collectionName;
			std::vector<unsigned char> payload;
		};

		std::queue<StMessageDB> m_qMessageData;

		//pjh
		Routine::CTimer						m_timerDBMSConnector;
		Routine::CTimer						m_timerUpdateHeading;
		Routine::CTimer						m_timerUpdateCooperation;
		//
		Routine::CEvent m_eProcessData;
		Routine::CThread m_threadProcessDB;

		void ProcessDB();

		//pjh
		public:
			virtual void OnTimerConnectMongoDB() {}
			virtual void OnTimerUpdateCraneInfoFromMongoDB() {}
			virtual void OnTimerUpdateCooperationFromMongoDB() {}
			virtual void OnUpdatedDistanceToMongoDB() {}

			enum class ETimer
			{
				E_DBConnector,
				E_UpdateHeading,
				E_UpdateCooperation
			};

			//useThread는 DBConnector일 때만 사용
			bool StartTimer(ETimer timer, uint32_t intervalTime);

			void StopTimer(ETimer timer);
		//
	};