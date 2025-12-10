#include "EstimateHook.h"

CEstimateHook::CEstimateHook(void)
	: m_minHeight(0), m_maxHeight(0)
{
}

CEstimateHook::~CEstimateHook(void)
{

}

void CEstimateHook::SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInput)
{
	m_cloudInput = cloudInput;
}

void CEstimateHook::SetClusterInfo(std::shared_ptr<std::vector<pcl::IndicesPtr>> clusterInfo)
{
	m_clusterIndices = clusterInfo;
}

void CEstimateHook::SetHangROI(std::vector<double_t>& hangRange)
{
	m_vHangROI.clear();
	for (uint32_t i = 0; i < hangRange.size(); i += 6)
	{
		m_vHangROI.push_back(hangRange[i + 0]);
		m_vHangROI.push_back(hangRange[i + 1]);
		m_vHangROI.push_back(hangRange[i + 2]);
		m_vHangROI.push_back(hangRange[i + 3]);
		m_vHangROI.push_back(hangRange[i + 4]);
		m_vHangROI.push_back(hangRange[i + 5]);
	}
}

void CEstimateHook::SetFloatROI(std::vector<double_t>& floatRange)
{
	m_vfloatROI.clear();
	for (uint32_t i = 0; i < floatRange.size(); i += 6)
	{
		m_vfloatROI.push_back(floatRange[i + 0]);
		m_vfloatROI.push_back(floatRange[i + 1]);
		m_vfloatROI.push_back(floatRange[i + 2]);
		m_vfloatROI.push_back(floatRange[i + 3]);
		m_vfloatROI.push_back(floatRange[i + 4]);
		m_vfloatROI.push_back(floatRange[i + 5]);
	}
}

void CEstimateHook::SetMinFloatingHeight(double_t minHeight)
{
	m_minHeight = minHeight;
}

void CEstimateHook::SetMaxFloatingHeight(double_t maxHeight)
{
	m_maxHeight = maxHeight;
}

bool CEstimateHook::Run()
{
	// 후크 판단 관련 내용
	// 후크 혹은 인양물을 판단하여 예외 처리하기 위하여 다음을 수행한다.
	// * 후크/인양물은 관심 영역 내의 부유체로 정의한다.
	// 
	bool ret = false;
	std::shared_ptr<std::vector<int32_t>> resultIndices(new std::vector<int32_t>);

	if (m_cloudInput && m_clusterIndices)
	{
		ret = true;
		for (size_t idxCluster = 0; idxCluster < m_clusterIndices->size(); idxCluster++)
		{
			pcl::IndicesPtr indices = m_clusterIndices->data()[idxCluster];

			bool bInHangROI = false;
			bool bInFloatROI = false;
			bool bHanging = false;
			bool bFloating = true;
			for (size_t idxIndices = 0; idxIndices < indices->size(); idxIndices++)
			{
				uint32_t idx = (uint32_t)indices->data()[idxIndices];
				pcl::PointXYZ point = m_cloudInput->points[idx];

				// ROI 검사(영역 내에 걸치는 클러스터)
				for (uint32_t i = 0; i < m_vHangROI.size(); i += 6)
				{
					if (m_vHangROI[i + 0] <= point.x && point.x <= m_vHangROI[i + 1] &&
						m_vHangROI[i + 2] <= point.y && point.y <= m_vHangROI[i + 3] &&
						m_vHangROI[i + 4] <= point.z && point.z <= m_vHangROI[i + 5])
					{
						bInHangROI = true;
					}
				}

				for (uint32_t i = 0; i < m_vfloatROI.size(); i+=6)
				{
					if (m_vfloatROI[i + 0] <= point.x && point.x <= m_vfloatROI[i + 1] &&
						m_vfloatROI[i + 2] <= point.y && point.y <= m_vfloatROI[i + 3] &&
						m_vfloatROI[i + 4] <= point.z && point.z <= m_vfloatROI[i + 5])
					{
						bInFloatROI = true;
					}
				}

				// 메달린 물체 검사
				// 하나 또는 모든 클러스터 내의 포인트가 기준 높이보다 높음
				if (m_maxHeight <= point.z)
				{
					bHanging = true;
				}

				// 부유체 검사
				// 기준 높이 이상에 부유함
				if (m_minHeight >= point.z)
				{
					bFloating = false;
				}
			}

			if ((bInHangROI && bHanging) || (bInFloatROI && bFloating))
			{
				// 범위 안의 부유체 결과에 추가
				resultIndices->push_back(idxCluster);
			}
		}
		m_resultIndices = resultIndices;
	}
	return ret;
}

void CEstimateHook::GetResult(std::shared_ptr<std::vector<int32_t>> &resultIndices)
{
	resultIndices = m_resultIndices;
}
