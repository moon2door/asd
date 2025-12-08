#pragma once

#include <Utility/Config.h>
#include <Data/StRotorParameter.h>
#include <Config/CraneInfo.h>


class CModelRotorParameter
{
public:
	CModelRotorParameter()
	{
		m_rotorparameter = new SHI::Data::StRotorParameter[32];
		memset(m_rotorparameter, 0, sizeof(SHI::Data::StRotorParameter) * 10);
	}

	inline void UpdateRotorParameter(SHI::Data::StRotorParameter* pRotorParameter)
	{
		m_rotorparameter[32] = *pRotorParameter;
		char StrZone[32] = "";
		Routine::CMatrix44f intrinsic;
		Routine::CMatrix44f extrinsic;
		std::string szPier = Routine::GetConfigString("CraneType", "Pier", "Y", "CraneType.json");
		int32_t pier = SHI::ConvPierInt(szPier);
		int32_t crane = Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
		int numSensor = SHI::GetNumSensor(pier, crane);
		for (uint8_t i = 0; i < numSensor; i++)
		{
			const uint8_t id = i + 1;
			//for(uint8_t j = 0; j<16; j++)
			//{
			intrinsic << pRotorParameter->Sensor[i].intrinsicsMatrix[0], pRotorParameter->Sensor[i].intrinsicsMatrix[1], pRotorParameter->Sensor[i].intrinsicsMatrix[2], pRotorParameter->Sensor[i].intrinsicsMatrix[3], pRotorParameter->Sensor[i].intrinsicsMatrix[4],
				pRotorParameter->Sensor[i].intrinsicsMatrix[5], pRotorParameter->Sensor[i].intrinsicsMatrix[6], pRotorParameter->Sensor[i].intrinsicsMatrix[7], pRotorParameter->Sensor[i].intrinsicsMatrix[8], pRotorParameter->Sensor[i].intrinsicsMatrix[9],
				pRotorParameter->Sensor[i].intrinsicsMatrix[10], pRotorParameter->Sensor[i].intrinsicsMatrix[11], pRotorParameter->Sensor[i].intrinsicsMatrix[12], pRotorParameter->Sensor[i].intrinsicsMatrix[13], pRotorParameter->Sensor[i].intrinsicsMatrix[14],
				pRotorParameter->Sensor[i].intrinsicsMatrix[15];
				extrinsic << pRotorParameter->Sensor[i].extrinsicsMatrix[0], pRotorParameter->Sensor[i].extrinsicsMatrix[1], pRotorParameter->Sensor[i].extrinsicsMatrix[2], pRotorParameter->Sensor[i].extrinsicsMatrix[3], pRotorParameter->Sensor[i].extrinsicsMatrix[4],
					pRotorParameter->Sensor[i].extrinsicsMatrix[5], pRotorParameter->Sensor[i].extrinsicsMatrix[6], pRotorParameter->Sensor[i].extrinsicsMatrix[7], pRotorParameter->Sensor[i].extrinsicsMatrix[8], pRotorParameter->Sensor[i].extrinsicsMatrix[9],
					pRotorParameter->Sensor[i].extrinsicsMatrix[10], pRotorParameter->Sensor[i].extrinsicsMatrix[11], pRotorParameter->Sensor[i].extrinsicsMatrix[12], pRotorParameter->Sensor[i].extrinsicsMatrix[13], pRotorParameter->Sensor[i].extrinsicsMatrix[14],
					pRotorParameter->Sensor[i].extrinsicsMatrix[15];
			//}
			sprintf_s(StrZone, "Rotor%d", id);
			Utility::SetConfigMatrix44f(StrZone, "intrinsicParameters", intrinsic, "./interfacerotor.json");
			Utility::SetConfigMatrix44f(StrZone, "extrinsicParameters", extrinsic, "./interfacerotor.json");

		}
		
		//intrinsic << 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006,
		//	0.0007, 0.0008, 0.0009, 0.0010, 0.0011, 0.0012, 0.0013, 0.0014, 0.0015, 0.0016;
			/*pRotorParameter->Sensor->intrinsicsMatrix[0], pRotorParameter->Sensor->intrinsicsMatrix[1], pRotorParameter->Sensor->intrinsicsMatrix[2],
			pRotorParameter->Sensor->intrinsicsMatrix[3], pRotorParameter->Sensor->intrinsicsMatrix[4], pRotorParameter->Sensor->intrinsicsMatrix[5],
			pRotorParameter->Sensor->intrinsicsMatrix[6], pRotorParameter->Sensor->intrinsicsMatrix[7], pRotorParameter->Sensor->intrinsicsMatrix[8],
			pRotorParameter->Sensor->intrinsicsMatrix[9], pRotorParameter->Sensor->intrinsicsMatrix[10], pRotorParameter->Sensor->intrinsicsMatrix[11],
			pRotorParameter->Sensor->intrinsicsMatrix[12], pRotorParameter->Sensor->intrinsicsMatrix[13], pRotorParameter->Sensor->intrinsicsMatrix[14],
			pRotorParameter->Sensor->intrinsicsMatrix[15];*/
		//Utility::SetConfigMatrix44f("Rotor1", "intrinsicParameters", intrinsic, "./interfacerotor.json");
		//Routine::SetConfigDouble("Rotor1", "altitudeAngles", pRotorParameter->Sensor, "./interfacerotor.json");
		//Routine::SetConfigDouble(StrZone, "Length2", pCollisionZoneLength->Zone2, "./interfacerotor.json");
		//Routine::SetConfigDouble(StrZone, "Length3", pCollisionZoneLength->Zone3, "./interfacerotor.json");
		//Routine::SetConfigDouble(StrZone, "Length4", pCollisionZoneLength->Zone4, "./interfacerotor.json");

	}

private:
	SHI::Data::StRotorParameter* m_rotorparameter;
};