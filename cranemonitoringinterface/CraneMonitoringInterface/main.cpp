
#include <iostream>
#include <Routine/include/Routines/CProcess.h>
#include "ProcessCollisionManager.h"
#include "ProcessCraneMonitoring.h"



#ifdef TEMP_KSG
//#pragma comment(linker, "/subsystem:windows")
//int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char*, int nShowCmd)
//{
//	SHI::CProcessCollisionManager::Instance();
//	SHI::CProcessCraneMonitoring::Instance();
//	Routine::CProcess process;
//	process.Init();
//	return process.Run();
//}
#endif
#pragma comment(linker, "/subsystem:console")
int main()
{
    SHI::CProcessCollisionManager::Instance();
    SHI::CProcessCraneMonitoring::Instance();

    const std::string buildDateTime = std::string(__DATE__) + std::string(" ") + std::string(__TIME__);
    printf("build Date Time = %s \n", buildDateTime.c_str());

    Routine::CProcess process;
    process.Init();
    return process.Run();
}