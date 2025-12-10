// CollisionProcessor.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"

#include "ColiisionProcessor.h"
#include <windows.h>
#include <conio.h>


int main()
{
	HANDLE hHandle = NULL;
	hHandle = ::CreateMutex(NULL, TRUE, "CollisionProcessor");
	if (::GetLastError() == ERROR_ALREADY_EXISTS)
	{
		::CloseHandle(hHandle);
		return 0;
	}

	CColiisionProcessor collisionInformation;
	collisionInformation.Create();

	while (true)
	{	
		Sleep(100);
	}

    return 0;
}