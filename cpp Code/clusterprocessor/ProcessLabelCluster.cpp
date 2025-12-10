#include "../ClusterProcessor.h"
#include <Config/CraneInfo.h>
#include <Config/HookCluster.h>
#include <Utility/Types.h>

bool CClusterProcessor::ProcessLabelCluster(SHI::PCLIndicesVector& dstIndices, SHI::PointCloud& dstCloud, std::vector<unsigned char> &labels, SHI::PointCloudPtr& cloud, const SHI::PCLIndicesVectorPtr& srcIndices, const SHI::PointCloudPtr& cloudBody, const SHI::PointCloudPtr& cloudGround, const SHI::PointCloudPtr& cloudException)
{
	bool ret = false;
	SHI::PCLIndicesVector _dstIndices;
	std::vector<unsigned char> _labels;

	// 입력 파라미터
	SHI::CraneAttitudePtr attitude = GetCraneAttitude();
	std::shared_ptr<SHI::Data::StDBInfo> dbInfo = GetDBInfo();
	std::shared_ptr<SHI::Data::StPlcInfo> plcInfo = GetPlcInfo();
	SHI::Recognition::HookRecogPtr estimator = GetRecognitionHook();
	
	// 후크 클러스터 라벨링
	if (attitude && dbInfo && estimator)
	{
		// 후크 정보 갱신
		SHI::UpdateHookAttitude(attitude, dbInfo, plcInfo);
		SetCraneAttitude(*attitude);

		// 후크 클러스터 처리
		SHI::PCLIndicesVectorPtr indicesObject(new SHI::PCLIndicesVector);
		SHI::PCLIndicesVectorPtr indicesHook(new SHI::PCLIndicesVector);
		estimator->Run(cloud, srcIndices, attitude);
		estimator->GetResult(*indicesObject, *indicesHook);

		// 인양물 정보 갱신
		attitude->salvageInfo = estimator->GetSalvageInfo();
		SetCraneAttitude(*attitude);

		// 출력 정보
		_dstIndices.insert(_dstIndices.end(), indicesObject->begin(), indicesObject->end());
		_dstIndices.insert(_dstIndices.end(), indicesHook->begin(), indicesHook->end());
		for (unsigned int i = 0; i < indicesObject->size(); i++)
		{
			_labels.push_back(SHI::LABEL_OBJECT); // 물체 클러스터
		}
		for (unsigned int i = 0; i < indicesHook->size(); i++)
		{
			_labels.push_back(SHI::LABEL_HOOK); // 후크 클러스터
		}

		ret = true;
	}

	// 몸체 및 지면 클러스터 라벨링
	dstCloud.insert(dstCloud.end(), cloud->begin(), cloud->end());
	dstCloud.insert(dstCloud.end(), cloudBody->begin(), cloudBody->end()); // 몸체 포인트 병합	
	dstCloud.insert(dstCloud.end(), cloudGround->begin(), cloudGround->end()); // 지면 포인트 병합
	dstCloud.insert(dstCloud.end(), cloudException->begin(), cloudException->end()); // 예외 포인트 병합

	SHI::PCLIndices indicesBody;
	SHI::PCLIndices indicesGround;
	SHI::PCLIndices indicesException;
	unsigned int offset1 = cloud->size();
	unsigned int offset2 = cloud->size() + cloudBody->size();
	unsigned int offset3 = cloud->size() + cloudBody->size() + cloudGround->size();
	for (unsigned int i = 0; i < cloudBody->size(); i++)
	{
		indicesBody.indices.push_back(offset1 + i);
	}
	for (unsigned int i = 0; i < cloudGround->size(); i++)
	{		
		indicesGround.indices.push_back(offset2 + i);
	}
	for (unsigned int i = 0; i < cloudException->size(); i++)
	{
		indicesException.indices.push_back(offset3 + i);
	}
	_dstIndices.push_back(indicesBody);
	_dstIndices.push_back(indicesGround);
	_dstIndices.push_back(indicesException);
	_labels.push_back(SHI::LABEL_BODY); // 몸체 클러스터
	_labels.push_back(SHI::LABEL_GROUND); // 지면 클러스터
	_labels.push_back(SHI::LABEL_EXCEPTION); // 예외 클러스터

	dstIndices = _dstIndices;
	labels = _labels;

	return ret;
}