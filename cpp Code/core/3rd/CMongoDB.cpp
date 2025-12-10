#include "CMongoDB.h"

CMongoDB::CMongoDB()
{
	static mongocxx::instance inst{};

	m_bConnected = false;
}


CMongoDB::~CMongoDB()
{
}

int CMongoDB::ConnectDB(std::string ip, std::string port)
{
	if (m_bConnected == false)
	{
		// 立加
		std::string uriText = "mongodb://" + ip + ":" + port + "/?compressors=disabled&gssapiServiceName=mongodb";
		mongocxx::uri uri{ uriText.c_str() };
		m_client = mongocxx::client(uri);

		// 立加 抛胶飘
		auto db = m_client["ConnectionTest"];
		
		try
		{
			using namespace bsoncxx::builder::basic;
			auto buildInfo = db.run_command(make_document(kvp("buildInfo", 1)));

			if (buildInfo.view()["ok"].get_double() != double{ 1 })
			{
				throw std::logic_error("BuildInfo Error \n");
			}
			else
			{
				if (m_client)
				{
					std::cout << "MongoDB Connection Success " << ip << ":"<<port << std::endl;

					m_bConnected = true;


					return EXIT_SUCCESS;
				}
			}
		}
		catch (const std::logic_error& e)
		{
			std::cout << " MongoDB Connection Error --> " << e.what() << std::endl;

			m_bConnected = false;
			return EXIT_FAILURE;
		}
		catch (mongocxx::operation_exception& e)
		{
			std::cout << " MongoDB Connection Error --> " << e.what() << std::endl;

			m_bConnected = false;
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}

bool CMongoDB::DisconnectDB()
{
	m_bConnected = false;
	return m_bConnected;
}

bool CMongoDB::IsConnected()
{
	return m_bConnected;
}

void CMongoDB::DeleteDatabase(std::string dbName)
{
	try
	{
		auto db = m_client[dbName.c_str()];
		db.drop();


		//std::cout << dbName << " Delete Success " << std::endl;
	}
	catch (mongocxx::operation_exception & e)
	{
		std::cout << "MongoDB Database Delete Error --> " << e.what() << std::endl;
	}
}

void CMongoDB::InsertDocumentBinary(std::string dbName, std::string colName, const std::vector<uint8_t> &data)
{
	printf("InsertDocumentBinary %s %s\n", dbName.c_str(), colName.c_str());
	bsoncxx::builder::stream::document doc{};
	bsoncxx::types::b_binary binData { bsoncxx::binary_sub_type::k_binary, (uint32_t)data.size(), data.data() };
	doc << "Data" << binData;

	try
	{
		auto res = m_client[dbName.c_str()][colName.c_str()].insert_one(doc.view());
	}
	catch (mongocxx::bulk_write_exception & e)
	{
		std::cout << ":MongoDB Insert Error --> " << e.what() << std::endl;
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Insert Error --> " << e.what() << std::endl;
	}
}

void CMongoDB::DeleteCollection(std::string dbName, std::string colName)
{
	try
	{
		auto doc = m_client[dbName.c_str()][colName.c_str()];
		doc.drop();
	}
	catch(mongocxx::operation_exception e)  // NOLINT(misc-throw-by-value-catch-by-reference)
	{
		std::cout << "Mongo DB DeleteCollection Error --> " << e.what() << " " << e.code() << std::endl;
	}

	
	
}

bool CMongoDB::ReadDocumentsBinary(std::string dbName, std::string colName, std::vector<std::vector<unsigned char>> &outData)
{
	bool ret = false;
	m_csRead.Lock();
	try
	{
		mongocxx::collection collection = m_client[dbName.c_str()][colName.c_str()];
		mongocxx::options::find opt;
		
		opt.limit(0);
		opt.batch_size(100000);

		auto docList = collection.find({}, opt);

		for (auto&& doc : docList)
		{
			bsoncxx::document::view	view = doc;

			bsoncxx::document::element element = view["Data"];
			bsoncxx::types::b_binary binData = element.get_binary();

			std::vector<unsigned char> tmp;
			tmp.resize(binData.size);
			memcpy(tmp.data(), binData.bytes, binData.size);
			outData.push_back(tmp);
			ret = true;
		}
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Show Document Error --> " << e.what() << std::endl;
	}
	m_csRead.UnLock();
	return ret;
}

bool CMongoDB::ReadDocumentsBinary(std::string dbName, std::string colName, std::vector<unsigned char> &binary, unsigned int idx)
{
	bool ret = false;
	unsigned int count = 0;
	m_csRead.Lock();
	try
	{
		bsoncxx::builder::stream::document doc{};
		mongocxx::collection collection = m_client[dbName.c_str()][colName.c_str()];
		mongocxx::options::find opt;
		opt.skip(idx);
		opt.limit(1);

		auto docList = collection.find({}, opt);
		
		for (auto&& doc : docList)
		{
			bsoncxx::document::view	view = doc;
			bsoncxx::document::element element = view["Data"];
			bsoncxx::types::b_binary binData = element.get_binary();
			
			std::vector<unsigned char> tmp;
			tmp.resize(binData.size);
			if (tmp.size() >= binData.size)
			{
				memcpy(tmp.data(), binData.bytes, binData.size);
				binary = tmp;
				ret = true;
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Show Document Error --> " << e.what() << std::endl;
	}
	m_csRead.UnLock();
	return ret;
}

bool CMongoDB::ReadDocumentsBinaryTime(std::string dbName, std::string colName, std::vector<unsigned char> &binary, SHI::Data::StDateTime &time, unsigned int idx)
{
	bool ret = false;
	m_csRead.Lock();
	unsigned int count = 0;
	try
	{
		bsoncxx::builder::stream::document doc{};
		mongocxx::collection collection = m_client[dbName.c_str()][colName.c_str()];
		mongocxx::options::find opt;
		opt.skip(idx);
		opt.limit(1);

		auto docList = collection.find({}, opt);

		for (auto&& doc : docList)
		{
			bsoncxx::document::view	view = doc;


			bsoncxx::document::element element1 = view["Data"];
			bsoncxx::document::element element2 = view["_id"];
			bsoncxx::types::b_binary binData = element1.get_binary();
			bsoncxx::types::b_oid oid = element2.get_oid();

			std::vector<unsigned char> tmp;
			tmp.resize(binData.size);
			if (tmp.size() >= binData.size)
			{
				memcpy(tmp.data(), binData.bytes, binData.size);
				binary = tmp;

				std::time_t t = oid.value.get_time_t();
				std::tm localTime = *std::localtime(&t);
				time.year = localTime.tm_year + 1900;
				time.month = localTime.tm_mon + 1;
				time.date = localTime.tm_mday;
				time.hour = localTime.tm_hour;
				time.min = localTime.tm_min;
				time.sec = localTime.tm_sec;

				ret = true;
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Show Document Error --> " << e.what() << std::endl;
	}
	m_csRead.UnLock();
	return ret;
}

bool CMongoDB::ReadLastDocumentBinary(std::string dbName, std::string colName, std::vector<unsigned char> &binary)
{
	bool ret = false;
	
	try
	{		m_csRead.Lock();
		mongocxx::collection collection = m_client[dbName.c_str()][colName.c_str()];
		auto opt = mongocxx::options::find{};
		opt.limit(1);
		opt.batch_size(100000);
		opt.sort(bsoncxx::builder::stream::document{} << "_id" << -1 << bsoncxx::builder::stream::finalize);
		m_csRead.UnLock();

		mongocxx::cursor docList = collection.find({}, opt);
		
		for (auto&& doc : docList)
		{
			bsoncxx::document::view	view = doc;
			
			bsoncxx::document::element element = view["Data"];
			
			bsoncxx::types::b_binary binData = element.get_binary();
			
			std::vector<unsigned char> tmp;
			tmp.resize(binData.size);
			if (tmp.size() >= binData.size)
			{
				memcpy(tmp.data(), binData.bytes, binData.size);
				binary = tmp;
				ret = true;
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Show Document Error --> " << e.what() << std::endl;
	}
	return ret;
}

unsigned int CMongoDB::ReadNumDocuments(std::string dbName, std::string colName)
{
	unsigned int count = 0;
	m_csRead.Lock();
	try
	{
		bsoncxx::builder::stream::document doc{};
		mongocxx::collection collection = m_client[dbName.c_str()][colName.c_str()];
		mongocxx::options::find opt;
		opt.limit(0);
		opt.batch_size(100000);
		count = collection.count_documents({});
	}
	catch (mongocxx::query_exception& e)
	{
		std::cout << "MongoDB ReadNumDocuments Error --> " << e.what() << " Code = " << e.code() << " DB_Name = " << dbName << "COL_Name = " << colName << std::endl;
	}
	m_csRead.UnLock();
	return count;
}

bool CMongoDB::FindDocumentsBinaryTime(std::string dbName, std::string colName, std::vector<unsigned char> &binary, SHI::Data::StDateTime findTime)
{
	bool ret = false;
	unsigned int count = 0;
	try
	{
		bsoncxx::builder::stream::document doc{};
		mongocxx::collection collection = m_client[dbName.c_str()][colName.c_str()];
		mongocxx::options::find opt;
		opt.limit(1);
		opt.sort(bsoncxx::builder::stream::document{} << "_id" << -1 << bsoncxx::builder::stream::finalize);

		// findtime to oid
		bsoncxx::oid findOid2("60c84437480c0000c3000608");
		std::tm tmTime;
		tmTime.tm_year = findTime.year - 1900;
		tmTime.tm_mon = findTime.month - 1;
		tmTime.tm_mday = findTime.date;
		tmTime.tm_hour = findTime.hour;
		tmTime.tm_min = findTime.min;
		tmTime.tm_sec = findTime.sec;
		std::time_t timespan = std::mktime(&tmTime);
		char bytes[12] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
		char* pt = (char*)&timespan;
		bytes[0] = pt[3];
		bytes[1] = pt[2];
		bytes[2] = pt[1];
		bytes[3] = pt[0];
		bsoncxx::oid findOid(bytes,12);
		
		// find oid <= timestamp
		auto docList = collection.find(
			bsoncxx::builder::stream::document{} << "_id" << bsoncxx::builder::stream::open_document <<
			"$lte" << findOid
			<< bsoncxx::builder::stream::close_document << bsoncxx::builder::stream::finalize
			, opt);

		for (auto&& doc : docList)
		{
			bsoncxx::document::view	view = doc;


			bsoncxx::document::element element1 = view["Data"];
			bsoncxx::document::element element2 = view["_id"];
			bsoncxx::types::b_binary binData = element1.get_binary();
			bsoncxx::types::b_oid oid = element2.get_oid();

			std::vector<unsigned char> tmp;
			tmp.resize(binData.size);
			if (tmp.size() >= binData.size)
			{
				memcpy(tmp.data(), binData.bytes, binData.size);
				binary = tmp;
				//printf(" - oid %s", oid.value.to_string());
				//std::time_t t = oid.value.get_time_t();
				//std::tm localTime = *std::localtime(&t);
				//time.year = localTime.tm_year + 1900;
				//time.month = localTime.tm_mon + 1;
				//time.date = localTime.tm_mday;
				//time.hour = localTime.tm_hour;
				//time.min = localTime.tm_min;
				//time.sec = localTime.tm_sec;

				ret = true;
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Show Document Error --> " << e.what() << std::endl;
	}
	return ret;
}

unsigned int CMongoDB::ShowDatabase(std::vector<std::string>& res)
{
	std::cout << "Show Database " << std::endl;

	res.clear();

	try
	{
		auto dbList = m_client.list_databases();

		for (const bsoncxx::document::view & doc : dbList)
		{
			std::cout << doc["name"].get_utf8().value.data() << std::endl;
			
			res.push_back(doc["name"].get_utf8().value.data());
		}
	}
	catch (mongocxx::operation_exception& e)
	{
		std::cout << "MongoDB Get Database List Error --> " << e.what() << std::endl;
	}

	return res.size();
}

unsigned int CMongoDB::ShowCollections(std::string dbName, std::vector<std::string>& res)
{
	std::cout << "Show Collections " << std::endl;

	res.clear();

	try
	{
		auto colList = m_client[dbName.c_str()].list_collections();

		for (const bsoncxx::document::view & doc : colList)
		{
			std::cout << doc["name"].get_utf8().value.data() << std::endl;			
			res.push_back(doc["name"].get_utf8().value.data());
		}
	}
	catch (mongocxx::operation_exception& e)
	{
		std::cout << "MongoDB Get Collection List Error --> " << e.what() << std::endl;
	}

	return res.size();
}

unsigned int CMongoDB::GetDocSize(std::string dbName, std::string collectionName)
{
	mongocxx::collection collection = m_client[dbName.c_str()][collectionName.c_str()];

	unsigned int result = collection.estimated_document_count();

	return result;
}

void CMongoDB::BackupDatabase(std::string ip, std::string port, std::string dbName, std::string stored, std::string directory)
{
	std::string command = "mongodump --out " + directory + " --host " + ip + " --port " + " --db " + dbName + " --collection " + stored;
	system(command.c_str());
}

void CMongoDB::RestoreDatabase(std::string ip, std::string port, std::string dbName, std::string stored, std::string directory)
{
	std::string restore = "mongorestore --host " + ip + " --port " + port + " --db " + dbName + " --collection " + stored + " " + directory + "/" + dbName + "/" + stored + ".bson";
	system(restore.c_str());
}

void CMongoDB::UpdateDocumentBinary(std::string dbName, std::string collectionName, const std::vector<unsigned char> &data)
{
	printf("UpdateDocumentBinary %s %s\n", dbName.c_str(), collectionName.c_str());
	mongocxx::collection collection = m_client[dbName.c_str()][collectionName.c_str()];
	
	bsoncxx::builder::stream::document doc{};
	uint32_t size = static_cast<uint32_t>(data.size());
	bsoncxx::types::b_binary binData{ bsoncxx::binary_sub_type::k_binary, size, data.data() };
	doc << "Data" << binData;
	try
	{
		// Insert new data
		mongocxx::options::replace opt;
		opt.upsert(true);
		auto res = collection.replace_one({}, doc.view(), opt);
	
		//auto res = collection.insert_one(doc.view());
		//
		//// Delete most old data
		//collection.delete_one({});
	}
	catch (mongocxx::bulk_write_exception & e)
	{
	}
	catch (std::exception & e)
	{
	}

}

void CMongoDB::DeleteDocument(std::string dbName, std::string collectionName)
{
	std::vector<bsoncxx::document::value> document;

	mongocxx::collection collection = m_client[dbName.c_str()][collectionName.c_str()];
	collection.delete_one(bsoncxx::builder::stream::document{} << "ID" << 1 << bsoncxx::builder::stream::finalize);
}

bool CMongoDB::DeleteDocument(std::string dbName, std::string collectionName, unsigned int idx)
{
	bool ret = false;
	unsigned int count = 0;
	try
	{
		bsoncxx::builder::stream::document doc{};
		mongocxx::collection collection = m_client[dbName.c_str()][collectionName.c_str()];

		mongocxx::options::find opt;
		opt.limit(0);
		opt.batch_size(100000);

		auto docList = collection.find({}, opt);

		for (auto&& doc : docList)
		{
			if (count++ == idx)
			{
				bsoncxx::document::view	view = doc;
				bsoncxx::document::element element = view["_id"];
				auto oid = element.get_oid();
				std::cout << "oid = " << oid.value.to_string() << std::endl;

				collection.delete_one(bsoncxx::builder::stream::document{} << "_id" << oid << bsoncxx::builder::stream::finalize);
				ret = true;
			}
		}
	}
	catch (std::exception & e)
	{
		std::cout << " MongoDB Show Document Error --> " << e.what() << std::endl;
	}
	return ret;
}
