#include "IntegratedMonitoringInterface/IntegratedMonitoringInterface.h"
#include <windows.h>
#include <stdio.h>


//#pragma comment(linker, "/subsystem:windows")
//int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char*, int nShowCmd)
//{
//	CIntegratedMonitoringInterface inf;
//	inf.Create(15000);
//
//	while (true) Sleep(1000);
//}

#pragma comment(linker, "/subsystem:console")
void main()
{
#ifdef _DEBUG
	Sleep(10000);
#endif
	try
	{
		CIntegratedMonitoringInterface inf;
		inf.Create(15000);
		//inf.Test();
		Sleep(1000);
		while (true) Sleep(1000);
	}
	catch (std::exception e)
	{
		printf("catch exception %s\n", e.what());
	}
}