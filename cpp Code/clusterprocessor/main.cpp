// ClusterProcessor.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "ClusterProcessor.h"
#include <Windows.h>

int main()
{
#ifdef _DEBUG
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//Sleep(10000);
#endif
	HANDLE hHandle = NULL;
	hHandle = ::CreateMutex(NULL, TRUE, "ClusterProcessor");
	if (::GetLastError() == ERROR_ALREADY_EXISTS)
	{
		::CloseHandle(hHandle);
		return 0;
	}

	CClusterProcessor service;

	while (true)
	{
		Sleep(1000);
	}

	return 0;
}

