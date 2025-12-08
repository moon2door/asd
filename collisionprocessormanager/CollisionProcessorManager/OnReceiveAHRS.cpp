#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"
#include <Routine/Include/Base/RoutineUtility.h>
//pjh
//TODO 안쓰이는 코드
void CCollisionProcessorManagerDlg::OnDataEncoderGXM7S(uint32_t count, float_t angle)
{
	static uint32_t cnt = 0;

	if (++cnt > 10)
	{
		// 회전각 계산 파라미터
		float_t zeroOffset = 0;
		float_t zeroStep = Routine::GetConfigDouble("Encoder", "zeroCycle", 16.0);
		float_t zeroAngle = Routine::GetConfigDouble("Encoder", "zeroAngle", 317.0);
		float_t ratio = Routine::GetConfigDouble("Encoder", "ratio", -24.562);
		float_t gearRatio = 1.0 / ratio;
		//bool bZeroSet = DevLib::Utility::Config::GetConfigInt("Encoder", "zeroSet", 0, DevLib::Utility::Config::GetMakeConfigName()) == 1? true:false;

		if (m_zeroSet)
		{
			// 원점 업데이트
			printf("######## Reset\n");
			Routine::SetConfigDouble("Encoder", "zeroCycle", count);
			Routine::SetConfigDouble("Encoder", "zeroAngle", angle);
			//DevLib::Utility::Config::SetConfigInt("Encoder", "zeroSet", 0, DevLib::Utility::Config::GetMakeConfigName());
			m_zeroSet = false;
		}
		else
		{
			// 엔코더 0도 불연속 구간 처리
			int32_t step = count;
			while (step > zeroStep + 2047) step -= 4096;
			while (step < zeroStep - 2048) step += 4096;

			// 회전각 계산
			float_t currentAngle = (step - zeroStep) * 360 + (angle - zeroAngle);
			currentAngle = currentAngle * gearRatio + zeroOffset;

			// 각도 값 제한(0~360)
			while (currentAngle > 360) currentAngle -= 360;
			while (currentAngle < 0) currentAngle += 360;

			// 결과 송신
			SHI::Data::StDBInfo d = { 0, };
			d.hook.llc.angleAzimuth = currentAngle;
			GetStubDBInfo()->WriteData(&d);
		}
		cnt = 0;
	}
}