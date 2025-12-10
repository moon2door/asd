#include "../ClusterProcessor.h"

#include <Utility/Types.h>
#include <Utility/RecognitionObject.h>
#include <Config/CraneInfo.h>
#include <Config/UpdateCraneAttitude.h>
#include <Recognition/AbstractEstimator.h>
#include <Routine/Include/graphics/CImage.h>

bool _GetEstimateImage(Routine::Graphics::CImage& dst, const SHI::CraneAttitude &attitude, SHI::RecognitionObject* pObj, int partNum);

bool _ProcessEstimate(SHI::CraneAttitude &dst, SHI::PointCloudPtr cloud, SHI::RecognitionObject* pObj, int partNum);

bool CClusterProcessor::ProcessCraneAttitude(SHI::PointCloudPtr cloud)
{
	bool ret = false;
	SHI::CraneAttitudePtr attitude = GetCraneAttitude();
	std::shared_ptr<SHI::Data::StDBInfo> dbInfo = GetDBInfo();
	std::shared_ptr<SHI::Data::StPlcInfo> plcInfo = GetPlcInfo();

	if (attitude && dbInfo && plcInfo)
	{
		// 크레인 자세 인식
		bool bEstimate = true;
		for (unsigned int part=0; part<5; part++)
		{
			if (attitude->bUseEstimate[part])
			{
				_ProcessEstimate(*attitude, cloud, this, part);

				bEstimate = bEstimate && attitude->bEstimateSucceed[part];

				if (IsDebugMode() && GetEstimatorVisualizer(part))
				{
					Routine::Graphics::CImage img;
					_GetEstimateImage(img, *attitude, this, part);
					if (!img.IsNull()) GetEstimatorVisualizer(part)->ShowImage(img);
				}
			}
		}
		ret = bEstimate;
		
		// 크레인 자세 갱신: dbInfo를 attitude에 업데이트, plcInfo를 attitude->pose[peir_id]에 업데이트
		SHI::UpdateCraneAttitude(attitude, dbInfo, plcInfo);

		// 크레인 자세 정보 갱신: attitude를 m_attitude에 업데이트
		SetCraneAttitude(*attitude);
	}

	return ret;
}

bool _GetEstimateImage(Routine::Graphics::CImage& dst, const SHI::CraneAttitude &attitude, SHI::RecognitionObject* pObj, int partNum)
{
	bool ret = false;
	if (attitude.bUseEstimate[partNum])
	{
		if (pObj->GetRecognition(partNum) == nullptr)
			return ret;
		Routine::Graphics::CImage img;
		if (pObj->GetRecognition(partNum)->GetResultImage(img))
		{
			dst = img;
			ret = true;
		}
	}
	return ret;
}

bool _ProcessEstimate(SHI::CraneAttitude &dst, SHI::PointCloudPtr cloud, SHI::RecognitionObject* pObj, int partNum)
{
	bool ret = false;
	if (dst.bUseEstimate[partNum])
	{
		if (pObj->GetRecognition(partNum) == nullptr)
			return ret;
		ret = pObj->GetRecognition(partNum)->Run(cloud);
		if (ret)
		{
			dst.pose[partNum] = pObj->GetRecognition(partNum)->GetResult();
			dst.bMoving[partNum] = pObj->GetRecognition(partNum)->IsMoving();
		}
		dst.bEstimateSucceed[partNum] = ret;
	}
	return ret;
}