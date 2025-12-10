#include "CraneInfo.h"
#include <algorithm>
#include <cctype>

namespace SHI
{
	int32_t ConvPierInt(const std::string& pier)
	{
		std::string normalized = pier;
		std::transform(normalized.begin(), normalized.end(), normalized.begin(),
			[](unsigned char c) { return static_cast<char>(std::toupper(c)); });

		int32_t id = -1;
		for (uint32_t i = 0; i < g_numPier; i++)
		{
			if (normalized == SHI::g_codePier[i])
			{
				id = static_cast<int32_t>(i);
				break;
			}
		}
		return id;
	}

	std::string ConvPierStr(uint32_t pier) { return SHI::g_codePier[pier]; }

	std::string ConvPierDisplayStr(uint32_t pier)
	{
		if (pier == PIERY)
		{
			return "Y";
		}
		return SHI::g_codePier[pier];
	}

	std::string ConvCraneStr(uint32_t pier, uint32_t crane)
	{
		std::string ret;
		switch (pier)
		{
		case PIER7:
			ret = SHI::Pier7::g_codeCrane[crane];
			break;
		case PIERJ:
			ret = SHI::PierJ::g_codeCrane[crane];
			break;
		case PIERK:
			ret = SHI::PierK::g_codeCrane[crane];
			break;
		case PIERHAN:
			ret = SHI::PierHan::g_codeCrane[crane];
			break;
		case PIER6:
			ret = SHI::Pier6::g_codeCrane[crane];
			break;
		case G2DOCK:
			ret = SHI::G2Dock::g_codeCrane[crane];
			break;
		case G3DOCK:
			ret = SHI::G3Dock::g_codeCrane[crane];
			break;
		case G4DOCK:
			ret = SHI::G4Dock::g_codeCrane[crane];
			break;
		//PIERZ
		case PIERZ:
			ret = SHI::PierZ::g_codeCrane[crane];
			break;
		//
		//Y
		case PIERY:
			ret = SHI::PierY::g_codeCrane[crane];
			break;
		//
		default:
			break;
		}
		return ret;
	}

	std::string ConvPartStr(uint32_t pier, uint32_t crane, uint32_t part)
	{
		std::string ret;
		if (part<5)
		{
			switch (pier)
			{
			case PIER7:
				ret = SHI::Pier7::g_codePart[crane][part];
				break;
			case PIERJ:
				ret = SHI::PierJ::g_codePart[crane][part];
				break;
			case PIERK:
				ret = SHI::PierK::g_codePart[crane][part];
				break;
			case PIERHAN:
				ret = SHI::PierHan::g_codePart[crane][part];
				break;
			case PIER6:
				ret = SHI::Pier6::g_codePart[crane][part];
				break;
			case G2DOCK:
				ret = SHI::G2Dock::g_codePart[crane][part];
				break;
			case G3DOCK:
				ret = SHI::G3Dock::g_codePart[crane][part];
				break;
			case G4DOCK:
				ret = SHI::G4Dock::g_codePart[crane][part];
				break;
				//pjh
			case PIERZ:
				ret = SHI::PierZ::g_codePart[crane][part];
				break;
				//
				//Y
			case PIERY:
				ret = SHI::PierY::g_codePart[crane][part];
				break;
				//
			default:
				break;
			}
		}
		return ret;
	}

	uint32_t GetNumPier()
	{
		return SHI::g_numPier;
	}

	uint32_t GetNumCrane(uint32_t pier)
	{
		uint32_t ret = 0;
		switch (pier)
		{
		case PIER7:
			ret = SHI::Pier7::g_numCrane;
			break;
		case PIERJ:
			ret = SHI::PierJ::g_numCrane;
			break;
		case PIERK:
			ret = SHI::PierK::g_numCrane;
			break;
		case PIERHAN:
			ret = SHI::PierHan::g_numCrane;
			break;
		case PIER6:
			ret = SHI::Pier6::g_numCrane;
			break;
		case G2DOCK:
			ret = SHI::G2Dock::g_numCrane;
			break;
		case G3DOCK:
			ret = SHI::G3Dock::g_numCrane;
			break;
		case G4DOCK:
			ret = SHI::G4Dock::g_numCrane;
			break;
			//pjh
		case PIERZ:
			ret = SHI::PierZ::g_numCrane;
			break;
			//
			//Y
		case PIERY:
			ret = SHI::PierY::g_numCrane;
			break;
			//
		default:
			break;
		}
		return ret;
	}

	uint32_t GetTotalNumCrane()
	{
		uint32_t sum = 0;
		for (uint32_t i = 0; i<GetNumPier(); i++)
		{
			sum += GetNumCrane(i);
		}
		return sum;
	}

	int GetNumSensor(uint32_t pier, uint32_t crane)
	{
		int ret = 0;
		switch (pier)
		{
		case PIER7:
			ret = SHI::Pier7::g_numSensor[crane];
			break;
		case PIERJ:
			ret = SHI::PierJ::g_numSensor[crane];
			break;
		case PIERK:
			ret = SHI::PierK::g_numSensor[crane];
			break;
		case PIERHAN:
			ret = SHI::PierHan::g_numSensor[crane];
			break;
		case PIER6:
			ret = SHI::Pier6::g_numSensor[crane];
			break;
		case G2DOCK:
			ret = SHI::G2Dock::g_numSensor[crane];
			break;
		case G3DOCK:
			ret = SHI::G3Dock::g_numSensor[crane];
			break;
		case G4DOCK:
			ret = SHI::G4Dock::g_numSensor[crane];
			break;
			//pjh
		case PIERZ:
			ret = SHI::PierZ::g_numSensor[crane];
			break;
			//
			//Y
		case PIERY:
			ret = SHI::PierY::g_numSensor[crane];
			break;
			//
		default:
			break;
		}
		return ret;
	}

	SHI::SENSOR GetCraneSensorInfo(uint32_t pier, uint32_t crane, int idx)
	{
		SHI::SENSOR ret = SHI::SENSOR::ROTOR_NONE;
		switch (pier)
		{
		case PIER7:
			ret = SHI::Pier7::g_rotorInfo[crane][idx];
			break;
		case PIERJ:
			ret = SHI::PierJ::g_rotorInfo[crane][idx];
			break;
		case PIERK:
			ret = SHI::PierK::g_rotorInfo[crane][idx];
			break;
		case PIERHAN:
			ret = SHI::PierHan::g_sensorInfo[crane][idx];
			break;
		case PIER6:
			ret = SHI::Pier6::g_sensorInfo[crane][idx];
			break;
		case G2DOCK:
			ret = SHI::G2Dock::g_sensorInfo[crane][idx];
			break;
		case G3DOCK:
			ret = SHI::G3Dock::g_sensorInfo[crane][idx];
			break;
		case G4DOCK:
			ret = SHI::G4Dock::g_sensorInfo[crane][idx];
			break;
			//pjh
		case PIERZ:
			ret = SHI::PierZ::g_sensorInfo[crane][idx];
			break;
			//
			//Y
		case PIERY:
			ret = SHI::PierY::g_sensorInfo[crane][idx];
			break;
			//
		default:
			break;
		}
		return ret;
	}

	std::string GetSensorName(SHI::SENSOR code)
	{
		return g_sensorName[SHI::GetSensorIndex(code)];
	}

	int32_t GetCraneId(int32_t pier, int32_t crane)
	{
		int32_t ret = -1;
		int32_t count = 0;
		for (uint32_t i = 0; i < SHI::GetNumPier(); i++)
		{
			for (uint32_t j = 0; j < SHI::GetNumCrane(i); j++)
			{
				if (static_cast<uint32_t>(pier) == i && static_cast<uint32_t>(crane) == j)
				{
					ret = count;
				}
				count++;
			}
		}
		return ret;
	}

	int32_t GetID2Pier(int32_t id)
	{
		int32_t ret = -1;
		int32_t count = 0;
		for (uint32_t i = 0; i < SHI::GetNumPier(); i++)
		{
			for (uint32_t j = 0; j < SHI::GetNumCrane(i); j++)
			{
				if (count == id)
				{
					ret = static_cast<int32_t>(i);
				}
				count++;
			}
		}
		return ret;
	}

	int32_t GetID2Crane(int32_t id)
	{
		int32_t ret = -1;
		int32_t count = 0;
		for (uint32_t i = 0; i < SHI::GetNumPier(); i++)
		{
			for (uint32_t j = 0; j < SHI::GetNumCrane(i); j++)
			{
				if (count == id)
				{
					ret = static_cast<int32_t>(j);
				}
				count++;
			}
		}
		return ret;
	}

	bool IsValidPier(int32_t pier)
	{
		return (0 <= pier && static_cast<uint32_t>(pier) < GetNumPier());
	}

	bool IsValidCrane(int32_t pier, int32_t crane)
	{
		bool ret = false;
		if (IsValidPier(pier))
		{
			ret = (0 <= crane && static_cast<uint32_t>(crane) < GetNumCrane(pier));
		}
		return ret;
	}
}


