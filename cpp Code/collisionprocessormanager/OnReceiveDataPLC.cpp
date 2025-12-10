#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"

void CCollisionProcessorManagerDlg::OnConnectedPLCInfo(const std::string& ip, int32_t port)
{
	printf("OnConnectedPLCInfo\n");
}

void CCollisionProcessorManagerDlg::OnDisconnectedPLCInfo(const std::string& ip, int32_t port)
{
	printf("OnDisconnectedPLCInfo\n");
}

void CCollisionProcessorManagerDlg::OnPlcInfo(SHI::Data::StPlcInfo* info)
{
	SHI::Data::StPlcInfo plcInfo = *info;

// 	plcInfo.Trolley1Position += DevLib::Utility::Config::GetConfigInt("plcInfo", "Trolley1Position_mm", 0, "./craneOffset.ini");
// 	plcInfo.Trolley2Position += DevLib::Utility::Config::GetConfigInt("plcInfo", "Trolley2Position_mm", 0, "./craneOffset.ini");
// 	plcInfo.GoliathPosition1 += DevLib::Utility::Config::GetConfigInt("plcInfo", "GoliathPosition1_mm", 0, "./craneOffset.ini");
// 	plcInfo.GoliathPosition2 += DevLib::Utility::Config::GetConfigInt("plcInfo", "GoliathPosition2_mm", 0, "./craneOffset.ini");
// 	plcInfo.Hoist1Position += DevLib::Utility::Config::GetConfigInt("plcInfo", "Hoist1Position_mm", 0, "./craneOffset.ini");
// 	plcInfo.Hoist2Position += DevLib::Utility::Config::GetConfigInt("plcInfo", "Hoist2Position_mm", 0, "./craneOffset.ini");
// 	plcInfo.Hoist3Position += DevLib::Utility::Config::GetConfigInt("plcInfo", "Hoist3Position_mm", 0, "./craneOffset.ini");
// 	plcInfo.Hoist1Weight += DevLib::Utility::Config::GetConfigInt("plcInfo", "Hoist1Weight_ton", 0, "./craneOffset.ini") * 10;
// 	plcInfo.Hoist2Weight += DevLib::Utility::Config::GetConfigInt("plcInfo", "Hoist2Weight_ton", 0, "./craneOffset.ini") * 10;
// 	plcInfo.Hoist3Weight += DevLib::Utility::Config::GetConfigInt("plcInfo", "Hoist3Weight_ton", 0, "./craneOffset.ini") * 10;
// 	plcInfo.AuxHoistWeight += DevLib::Utility::Config::GetConfigInt("plcInfo", "AuxHoistWeight_ton", 0, "./craneOffset.ini") * 10;

	UpdatePLCInfo(&plcInfo);

	GetStubPlcInfo()->WriteData(&plcInfo);

	if (m_craneMonitoringInterface)
	{
		m_craneMonitoringInterface->SendPLCInfo(&plcInfo);
	}
}
