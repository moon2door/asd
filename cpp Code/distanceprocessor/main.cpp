

// DistanceProcessor.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "DistanceProcessor.h"
#include <windows.h>

int main()
{
	//pjh
#ifdef _DEBUG
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//Sleep(10000);
#endif
	HANDLE hHandle = NULL;
	hHandle = ::CreateMutexA(NULL, TRUE, "DistanceProcessor");
	if (::GetLastError() == ERROR_ALREADY_EXISTS)
	{
		::CloseHandle(hHandle);
		return 0;
	}
	//

	CDistanceProcessor service;
	while (true)  Sleep(1000);
	return 0;
}
