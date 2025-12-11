#pragma once

#include <string>
#include "SensorInfo.h"

namespace SHI
{
	// enum PIER 규칙
	//  ---------------------------------------------------------------------------
	//  1. **순서를 바꾸지 마세요.**
	//     각 요소의 정수 값(0,1,2 …)을 바이너리 프로토콜·DB·switch 문에서
	//     직접 사용하고 있으므로, 위치가 바뀌면 하위 호환이 즉시 깨집니다.
	//
	//  2. **새 부두를 추가할 땐 항상 맨 끝에만 붙입니다.**
	//     중간 삽입·삭제는 금지입니다.
	//
	//  3. **숫자 값을 직접 지정하지 않습니다.**
	//     “0 부터 연속” 규칙을 깨면 `static_cast<PIER>(index)` 패턴이 망가집니다.
	//
	//  ---------------------------------------------------------------------------
	//  Human-readable names ↔ enum labels
	//      "Pier 7"      → PIER7
	//      "J Pier"      → PIERJ
	//      "Pier K"      → PIERK
	//      "신한내"      → PIERHAN
	//      "Pier 6"      → PIER6
	//      "G2 Dock"     → G2DOCK
	//      "G3 Dock"     → G3DOCK
	//      "G4 Dock"     → G4DOCK
	//      "Pier Z"      → PIERZ
	//      "Pier Y"      → PIERY
	// -----------------------------------------------------------------------------

	enum PIER { PIER7, PIERJ, PIERK, PIERHAN, PIER6, G2DOCK, G3DOCK, G4DOCK, PIERZ, PIERY };
	static const std::string g_codePier[] =
	{
		"7",
		"J",
		"K",
		"HAN",
		"6",
		"G2",
		"G3",
		"G4",
		"Z",
		"Y"
	};
	static constexpr uint32_t g_numPier = sizeof(g_codePier) / sizeof(std::string);

	// =====================================================================
	//  Pier 7
	// =====================================================================
	namespace Pier7
	{
		// Crane identifiers
		enum CRANE { GC = 0, LLC16 = 1, LLC17 = 2, TC1 = 3, TC2 = 4, TC4 = 5, TC5 = 6, TC6 = 7 };
		static const std::string g_codeCrane[] =
		{
			"GC5", "LLC16", "LLC17", "TC1", "TC2", "TC4", "TC5", "TC6"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			5, 3, 3, 2, 2, 2, 2, 2
		};

		// LiDAR type per crane  (up to 5 per crane)
		static constexpr SENSOR g_rotorInfo[][5] =
		{
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 }
		};

		// Structural parts
		enum GC_PART { GC_HINGED_LEG, GC_FIXED_LEG, GC_GIRDER, GC_ROOM };
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };
		enum TC_PART { TC_JIB, TC_TOWER, TC_TROLLY };
		static const std::string g_codePart[][5] =
		{
			{ "GC_HINGED_LEG", "GC_FIXED_LEG", "GC_GIRDER", "GC_ROOM" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" }
		};

		// Hook positions
		enum GC_HOOK_PART { GC_HOOK_C = 0, GC_HOOK_XP = 1, GC_HOOK_XN = 2, GC_HOOK_NUM = 3 };
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
		enum TC_HOOK_PART { TC_HOOK = 0 };
	}

	// =====================================================================
	//  Pier J
	// =====================================================================
	namespace PierJ
	{
		// Crane identifiers
		enum CRANE { LLC24 = 0, LLC11 = 1, LLC8 = 2, LLC9 = 3 };
		static const std::string g_codeCrane[] =
		{
			"LLC24", "LLC11", "LLC8", "LLC9"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2, 3, 2
		};

		static constexpr SENSOR g_rotorInfo[][5] =
		{
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 }
		};

		// Structural parts
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };
		static const std::string g_codePart[][5] =
		{
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
	}

	// =====================================================================
	//  Pier K
	// =====================================================================
	namespace PierK
	{
		// Crane identifiers
		enum CRANE { JIB1 = 0, JIB2 = 1, JIB3 = 2, TTC23 = 3, LLC18 = 4, LLC19 = 5 };
		static const std::string g_codeCrane[] =
		{
			"JIB1", "JIB2", "JIB3", "TTC23", "LLC18", "LLC19"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2, 2, 2, 2, 2
		};

		static constexpr SENSOR g_rotorInfo[][5] =
		{
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_VLP16, SENSOR::PALA720_VLP16 },
			{ SENSOR::PALA720_OS1_16_gen1, SENSOR::PALA720_OS1_16_gen1 },
		};

		// Structural parts
		enum JIB_PART { JIB_BODY, JIB_JIB, JIB_TOWER };
		enum TTC_PART { TTC_JIB, TTC_TOWER };
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "TTC_JIB", "TTC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
		enum JIB_HOOK_PART { JIB_HOOK_NEAR = 0, JIB_HOOK_FAR = 1, JIB_HOOK_NUM = 2 };
		enum TTC_HOOK_PART { TTC_HOOK = 0 };
	}

	// =====================================================================
	//  Pier HAN
	// =====================================================================
	namespace PierHan
	{
		// Crane identifiers
		enum CRANE { GC1 = 0, GC2 = 1, TC1 = 2, TC2 = 3, TC3 = 4, TTC4 = 5, TC5 = 6, TC6 = 7 };
		static const std::string g_codeCrane[] =
		{
			"GC1", "GC2", "TC1", "TC2", "TC3", "TTC4", "TC5", "TC6"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2, 1, 1, 1, 2, 1, 1
		};

		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_Panda20A, SENSOR::PALA720_Panda20A },
			{ SENSOR::PALA720_Panda20A, SENSOR::PALA720_Panda20A },
			{ SENSOR::PALA720_OS1_16_gen1 },
			{ SENSOR::PALA720_OS1_16_gen1 },
			{ SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_16_gen1 },
			{ SENSOR::PALA720_OS1_16_gen1 }
		};

		// Structural parts
		enum GC_PART {
			GC_HINGED_LEG, GC_FIXED_LEG, GC_GIRDER, GC_ROOM, GC_LOWER_TROLLY_HOOK, GC_UPPER_TROLLY_HOOK
		};
		enum TTC_PART { TTC_JIB, TTC_TOWER, TTC_TROLLY, TTC_TROLLY2 };
		enum TC_PART { TC_JIB, TC_TOWER, TC_TROLLY };

		static const std::string g_codePart[][5] =
		{
			{ "GC_HINGED_LEG", "GC_FIXED_LEG", "GC_GIRDER", "GC_ROOM", "GC_LOWER_TROLLY_HOOK" },
			{ "GC_HINGED_LEG", "GC_FIXED_LEG", "GC_GIRDER", "GC_ROOM", "GC_LOWER_TROLLY_HOOK" },
			{ "TTC_JIB", "TTC_TOWER", "TTC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" }
		};

		// Hook positions
		enum GC_HOOK_PART { GC_HOOK_C = 0, GC_HOOK_XP = 1, GC_HOOK_XN = 2, GC_HOOK_NUM = 3 };
		enum TTC_HOOK_PART { TTC_HOOK_NEAR = 0, TTC_HOOK_FAR = 1, TTC_HOOK_NUM = 2 };
		enum TC_HOOK_PART { TC_HOOK = 0, TC_HOOK_NUM = 1 };
	}

	// =====================================================================
	//  Pier 6
	// =====================================================================
	namespace Pier6
	{
		// Crane identifiers
		enum CRANE { LLC7 = 0, LLC23 = 1 };
		static const std::string g_codeCrane[] =
		{
			"LLC7", "LLC23"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2
		};

		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS2_32, SENSOR::PALA720_OS2_32 }
		};

		// Structural parts
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
	}

	// =====================================================================
	//  G2 Dock
	// =====================================================================
	namespace G2Dock
	{
		// Crane identifiers
		enum CRANE { LLC12 = 0, LLC13 = 1 };
		static const std::string g_codeCrane[] =
		{
			"LLC12", "LLC13"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2
		};

		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 }
		};

		// Structural parts
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
	}

	// =====================================================================
	//  G3 Dock
	// =====================================================================
	namespace G3Dock
	{
		// Crane identifiers
		enum CRANE { LLC19 = 0, LLC20 = 1 };
		static const std::string g_codeCrane[] =
		{
			"LLC19", "LLC20"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2
		};

		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 }
		};

		// Structural parts
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
	}

	// =====================================================================
	//  G4 Dock
	// =====================================================================
	namespace G4Dock
	{
		// Crane identifiers
		enum CRANE { LLC25 = 0, LLC26 = 1 };
		static const std::string g_codeCrane[] =
		{
			"LLC25", "LLC26"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2, 2
		};

		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 }
		};

		// Structural parts
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" },
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
	}

	// =====================================================================
	//  Pier Z
	// =====================================================================
	namespace PierZ
	{
		// Crane identifiers
		enum CRANE { LLC16 = 0 };
		static const std::string g_codeCrane[] =
		{
			"LLC16"
		};
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane
		static constexpr int32_t g_numSensor[] =
		{
			2
		};

		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 }
		};

		// Structural parts
		enum LLC_PART { LLC_BODY, LLC_JIB, LLC_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "LLC_BODY", "LLC_JIB", "LLC_TOWER" }
		};

		// Hook positions
		enum LLC_HOOK_PART { LLC_HOOK_NEAR = 0, LLC_HOOK_FAR = 1, LLC_HOOK_NUM = 2 };
	}

	// =====================================================================
	//  *** NEW ***  Pier Y
	// =====================================================================
	namespace PierY
	{
		// Crane identifiers
		//   0 : TC1
		//   1 : JIB1
		//   2 : JIB2
		//   3 : JIB3
		//   4 : JIB4
		//   5 : JIB5
		enum CRANE { TC1 = 0, JIB1 = 1, JIB2 = 2, JIB3 = 3, JIB4 = 4, JIB5 = 5 };
		static const std::string g_codeCrane[] = { "TC1", "JIB1", "JIB2", "JIB3", "JIB4", "JIB5" };
		static constexpr uint32_t g_numCrane = sizeof(g_codeCrane) / sizeof(std::string);

		// Number of LiDARs per crane  (PLACEHOLDER – update if actual counts differ)
		static constexpr int32_t g_numSensor[] = { 1, 2, 2, 2, 2, 2 };

		// LiDAR type per crane  (PLACEHOLDER – update with real models)
		static constexpr SENSOR g_sensorInfo[][5] =
		{
			{ SENSOR::PALA720_OS1_32},
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 },
			{ SENSOR::PALA720_OS1_32, SENSOR::PALA720_OS1_32 }
		};

		// Structural parts  (PLACEHOLDER – refine as needed)
		enum TC_PART { TC_JIB, TC_TOWER, TC_TROLLY };
		enum JIB_PART { JIB_BODY, JIB_JIB, JIB_TOWER };

		static const std::string g_codePart[][5] =
		{
			{ "TC_JIB", "TC_TOWER", "TC_TROLLY" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" },
			{ "JIB_BODY", "JIB_JIB", "JIB_TOWER" }
		};

		// Hook positions  (PLACEHOLDER – adjust if multiple hooks exist)
		enum TC_HOOK_PART { TC_HOOK = 0, TC_HOOK_NUM = 1 };
		enum JIB_HOOK_PART { JIB_HOOK = 0, JIB_HOOK_NUM = 1 };
	}

	// ---------------------------------------------------------------------
	//  Utility / query functions (declarations only)
	// ---------------------------------------------------------------------
	int32_t  ConvPierInt(const std::string& pier);                   // pier string  -> enum
	std::string ConvPierStr(uint32_t pier);                          // enum         -> pier string
	std::string ConvPierDisplayStr(uint32_t pier);                   // enum         -> display string for console
	std::string ConvCraneStr(uint32_t pier, uint32_t crane);         // crane enum   -> crane string
	std::string ConvPartStr(uint32_t pier, uint32_t crane, uint32_t part);

	uint32_t GetNumPier();                                           // total number of piers
	uint32_t GetNumCrane(uint32_t pier);                             // cranes at a pier
	uint32_t GetTotalNumCrane();                                     // cranes overall

	int32_t  GetNumSensor(uint32_t pier, uint32_t crane);            // LiDARs on a crane
	SHI::SENSOR GetCraneSensorInfo(uint32_t pier, uint32_t crane, int32_t idx);
	std::string GetSensorName(SHI::SENSOR code);

	int32_t  GetCraneId(int32_t pier, int32_t crane);                // contiguous ID starting at 0
	int32_t  GetID2Pier(int32_t id);                                 // contiguous ID -> pier enum
	int32_t  GetID2Crane(int32_t id);                                // contiguous ID -> crane enum

	bool IsValidPier(int32_t pier);                                  // range check
	bool IsValidCrane(int32_t pier, int32_t crane);
}
