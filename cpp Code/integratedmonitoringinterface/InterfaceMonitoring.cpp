
#include "InterfaceMonitoring.h"
#include "protocolMonitoring.h"
#include <Routine/include/IO/CSocket.h>
#include <Routine/include/Base/RoutineTypes.h>

CMonitoring::CMonitoring()
{
	m_server.RegisterCallbackConnection(&CMonitoring::OnConnected, this);
	m_server.RegisterCallbackDisConnection(&CMonitoring::OnDisConnected, this);
	m_server.RegisterCallbackAccept(&CMonitoring::OnAccept, this);
	m_server.RegisterCallbackReceiveData(&CMonitoring::OnClientData, this);
}

CMonitoring::~CMonitoring()
{

}

void CMonitoring::SetObserver(CObserverMonitoring* pObserver)
{
	m_pObserver = pObserver;
}

//pjh
bool CMonitoring::Create(std::string ip, uint16_t port)
{
	bool ret = false;
	m_eMessage.Create();
	m_threadProcessMessage.StartThread(&CMonitoring::ProcessMessage, this);
	ret = m_server.CreateTcpServer(port, ip);

	return ret;
}
//

bool CMonitoring::Create(uint16_t port)
{
	bool ret = false;
	m_eMessage.Create();
	m_threadProcessMessage.StartThread(&CMonitoring::ProcessMessage, this);
	ret = m_server.CreateTcpServer(port);
		
	return ret;
}

bool CMonitoring::SendPointData(StPointDisplayData* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendCraneData(StCraneDisplayData* data)
{
	unsigned int size = sizeof(StCraneDisplayData);
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendStatus(StCraneStatusData* data)
{
	unsigned int size = sizeof(StCraneStatusData);
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendDistanceData(StDistanceDisplayData* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendCollisionHistory(StCollisionHistory* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendOperationHistory(StOperationHistory* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendUserInfo(StMonitoringUserInfo* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendSystempStatus(StSystemStatusData* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendCooperationModeList(StCooperationModeList* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

 bool CMonitoring::SendPlcInfo(StPLCInfo* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendReplyLogin(StReplyLogin* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

bool CMonitoring::SendLogPlayInfo(StIndicateLogPlay* data)
{
	unsigned int size = data->Size();
	return (m_server.Send(static_cast<void*>(data), size) == size);
}

void CMonitoring::ProcessMessage()
{
	std::queue<StServerMessage> qMessage;
	while (m_threadProcessMessage.IsRunThread())
	{
		if (m_eMessage.WaitForEvent(100))
		{
			qMessage.swap(m_qMessage);

			while (!qMessage.empty())
			{
				StServerMessage message = qMessage.front();
				qMessage.pop();

				if (message.data.size() < 12) continue;

				unsigned int *messageId = reinterpret_cast<unsigned int*>(message.data.data());

				if (*messageId == 9)
				{
					StCooperationMode* pData = reinterpret_cast<StCooperationMode*>(message.data.data());
					//printf("OnStCooperationMode\n");
					//printf(" - %d\n", pData->messageId);
					//printf(" - %d\n", pData->pierId);
					//printf(" - %d\n", pData->craneId);
					//printf(" - %d\n", pData->cooperationMode);
					//printf(" - %d\n", pData->startYear);
					//printf(" - %d\n", pData->startMonth);
					//printf(" - %d\n", pData->startDate);
					//printf(" - %d\n", pData->startHour);
					//printf(" - %d\n", pData->startMin);
					//printf(" - %d\n", pData->startSec);
					//printf(" - %d\n", pData->endYear);
					//printf(" - %d\n", pData->endMonth);
					//printf(" - %d\n", pData->endDate);
					//printf(" - %d\n", pData->endHour);
					//printf(" - %d\n", pData->endMin);
					//printf(" - %d\n", pData->endSec);
					//printf(" - %d\n", pData->authority);
					//printf(" - %s\n", pData->id);
					//printf(" - %s\n", pData->name);
					//printf(" - %s\n", pData->reserved);

					if (m_pObserver) m_pObserver->OnCooperationMode(pData);
				}
				else if (*messageId == 10)
				{
					StRequestCooperationList* pData = reinterpret_cast<StRequestCooperationList*>(message.data.data());
					//printf("_StRequestCooperationList\n");
					//printf(" - %d\n", pData->messageId);
					//printf(" - %d\n", pData->pierId);
					//printf(" - %d\n", pData->craneId);
					//printf(" - %d\n", pData->listCode);

					if (m_pObserver) m_pObserver->OnRequestCooperationList(pData);
				}
				else if (*messageId == 11)
				{
					StRequestCooperationAccept* pData = reinterpret_cast<StRequestCooperationAccept*>(message.data.data());
					if (m_pObserver) m_pObserver->OnRequestCooperationAccept(pData);
					//printf("_StRequestCooperationAccept\n");
					//printf(" - %d\n", pData->messageId);
					//printf(" - %d\n", pData->pierId);
					//printf(" - %d\n", pData->craneId);
					//printf(" - %d\n", pData->codeIndex);
					//printf(" - %d\n", pData->messageId);
				}
				else if (*messageId == 14)
				{
					StRequstAddUser* pData = reinterpret_cast<StRequstAddUser*>(message.data.data());
					if (m_pObserver) m_pObserver->OnRequesAddUser(pData);
					//printf("OnRequesAddUser \n");
					//printf(" - %d \n", pData->code);
					//printf(" - %s \n", pData->id);
					//printf(" - %s \n", pData->password);
					//printf(" - %s \n", pData->info);
				}
				else if (*messageId == 15)
				{
					StRequstLogin* pData = reinterpret_cast<StRequstLogin*>(message.data.data());
					if (m_pObserver) m_pObserver->OnRequesLogin(pData);
					//printf("OnRequesLogin \n");
					//printf(" - %s \n", pData->id);
					//printf(" - %s \n", pData->password);
				}
				else if (*messageId == 16)
				{
					StRequestCooperationRemove* pData = reinterpret_cast<StRequestCooperationRemove*>(message.data.data());
					if (m_pObserver) m_pObserver->OnRequestCooperationRemove(pData);
				}
				else if (*messageId == 17)
				{
					StRequestLogPlay* pData = reinterpret_cast<StRequestLogPlay*>(message.data.data());
					if (m_pObserver) m_pObserver->OnRequestLogPlay(pData);
				}
				else
				{
					printf("!!!!!!!!!! Undefined server message (%d,%d)\n", message.id, message.data.size());
				}
			}
		}
	}
}

bool CMonitoring::OnAccept(Routine::IO::CSocket& client)
{
	printf("OnAccept\n");
	return true;
}

void CMonitoring::OnConnected(const std::string& ip, uint16_t port)
{
	printf("OnConnected IP : %s, Port: %d \n", ip.c_str(), port);
	if(m_pObserver) m_pObserver->OnConnectedIntegratedMonitoring();
}

void CMonitoring::OnDisConnected(const std::string& ip, uint16_t port)
{
	printf("OnConnected IP : %s, Port: %d \n", ip.c_str(), port);
	if (m_pObserver) m_pObserver->OnDisconnectedIntegratedMonitoring();
}

//void CMonitoring::OnClientData(const std::string& ip, uint16_t port, void* data, int size)
void CMonitoring::OnClientData(Routine::IO::CSocket & client, const std::string& ip, uint16_t port, Routine::void_ptr data, int32_t size)
{
	StServerMessage message;
	message.id = *reinterpret_cast<unsigned int*>(data);
	message.data.resize(size);
	memcpy(message.data.data(), data, size);

	m_qMessage.push(message);
	m_eMessage.SetEvent();
}
