
#include <iostream>
#include <string>
#include <conio.h>
#include <Routine/include/Base/CTime.h>
#include <Routine/include/IO/CFile.h>
#include <Config/CraneInfo.h>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <Routine/include/Base/RoutineUtility.h>
#include "ProcessDBConnector.h"
#include <iostream>//pjh
#include <fstream>//pjh

class CProcess
{
	IntegratedRouter::CProcessDBConnector m_db;
public:
	CProcess(std::string ip, int32_t port, int32_t reserveDays)
	{
		m_db.ConnectDB(ip, port);
		if (m_db.IsConnectedDB())
		{
			RemoveCollisionData(reserveDays);
		}
	}
	// 특정 기간 동안의 데이터 수가 0보다 크다면 삭제. 
	void RemoveCollisionData(int reserveDays)
	{
		//int pier = 3;
		//int crane = 1;
		//int id = SHI::GetCraneId(pier, crane);
		//SHI::Data::StOperationHistory history;
		//bool ret = m_db.ReadOperationHistory(id, 2021, history);
		//for (unsigned int i=0; i<366; i++)
		//{
		//	boost::gregorian::date date(2021, 1, 1);
		//	boost::gregorian::days dsys(i);
		//	date = date + dsys;
		//	printf("%d, %d, %d, %d,  \t", i, (int)date.year(), (int)date.month(), (int)date.day());
		//	printf("%d, %d, %d, %d\n", (int)history.dayly[i].startHour, (int)history.dayly[i].startMin, (int)history.dayly[i].lastHour, (int)history.dayly[i].lastMin);
		//}

		for (int pier = 0; pier < SHI::GetNumPier(); pier++)
		{
			for (int crane = 0; crane < SHI::GetNumCrane(pier); crane++)
			{
				for (int dDate = 0; dDate < 365; dDate++)
				{
					Routine::CTime time;
					boost::gregorian::date date(time.Year(), time.Month(), time.Day() );
					boost::gregorian::days days(reserveDays + dDate);
					date = date - days;

					const int32_t id = SHI::GetCraneId(pier, crane);
					const int32_t year = date.year();
					const int32_t month = date.month();
					const int32_t day = date.day();
					const uint32_t num = m_db.ReadNumDistanceCompressed(id, year, month, day);

					//pjh 
					//printf("[%4d-%2d-%2d]Pier = %d, Crane = %d, Read Number of Distance Compressed Data Documents = %d \n",year, month, day, pier, crane, num);
					// 로그 메시지 생성
					char logMessage[256];
					sprintf(logMessage, "[%4d-%02d-%02d] Pier = %d, Crane = %d, Read Number of Distance Compressed Data Documents = %d\n",
						year, month, day, pier, crane, num);

					// 파일 출력 스트림 생성
					std::ofstream logFile("SchedulerLog.txt", std::ios::app); // 'app' 모드로 열어 기존 내용에 추가
					if (logFile.is_open()) {
						logFile << logMessage; // 로그 메시지를 파일에 기록
						logFile.close();       // 파일 닫기
					}
					else {
						std::cerr << "Failed to open SchedulerLog.txt for writing!" << std::endl;
					}
					//~pjh

					if (num > 0)
					{
						m_db.RemoveDistanceCompressed(id, year, month, day);
						printf("delete %s %s(%d-%d-%d) -- %d\n", SHI::ConvPierStr(pier).c_str(), SHI::ConvCraneStr(pier, crane).c_str(),year, month, day, num);
					}
				}
			}
		}
	}
};

void main()
{
	int32_t nReserveDay = Routine::GetConfigInt("Reserve Day", "Value", 90);
	CProcess* p = new CProcess("127.0.0.1", 27017, nReserveDay);
	delete p;

	std::exit(0);
}