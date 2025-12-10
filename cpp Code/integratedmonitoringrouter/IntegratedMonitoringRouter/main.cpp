// IntegratedMonitoringRouter.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "IntegratedMonitoringRouter/IntegratedMonitoringRouter.h"
#include <windows.h>

int main()
{
	IntegratedRouter::CIntegratedMonitoringRouter i;
	i.Create();
	while (true) Sleep(1000);
    return 0;
}