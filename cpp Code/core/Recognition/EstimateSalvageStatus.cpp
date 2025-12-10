#include "EstimateSalvageStatus.h"

#include <algorithm>


namespace SHI
{
	namespace Recognition
	{
		CEstimateSalvageStatus::CEstimateSalvageStatus(void)
			: m_medianFilterSize(0), m_totalAccumSize(0), m_threshold(0), m_bOnSalvage(false), m_countMinFilter(0), m_minWeight(FLT_MAX)
		{
		}

		CEstimateSalvageStatus::~CEstimateSalvageStatus(void)
		{

		}

		void CEstimateSalvageStatus::Initialize(float_t threshold, int32_t medianFilterSize, int32_t totalAccumSize)
		{
			m_medianFilterSize = medianFilterSize;
			m_totalAccumSize = totalAccumSize;
			m_threshold = threshold;

			std::queue<float_t> empty;
			m_qWeight.swap(empty);
		}

		void CEstimateSalvageStatus::UpdateWeight(float_t weight)
		{
			// median filter 데이터 입력
			m_qWeight.push(weight);
			while (m_qWeight.size() > m_medianFilterSize)
			{
				m_qWeight.pop();
			}

			// median filter 적용
			std::vector<float_t> vecMed;
			uint32_t size = m_qWeight.size();
			uint32_t centerIndex = size / 2;
			for (uint32_t i = 0; i < m_qWeight.size(); i++)
			{
				float_t f = m_qWeight.front();
				vecMed.push_back(f);

				m_qWeight.pop();
				m_qWeight.push(f);
			}

			std::sort(vecMed.begin(), vecMed.end());
			float_t med = vecMed[centerIndex];

			// min filter 적용
			m_minWeight = std::min(m_minWeight, med);
			m_countMinFilter++;
			if (m_countMinFilter > m_totalAccumSize)
			{
				if (m_minWeight > m_threshold)
				{
					m_bOnSalvage = true;
				}
				else
				{
					m_bOnSalvage = false;
				}
				OnSalvageStatusChange(m_bOnSalvage);

				m_minWeight = FLT_MAX;
				m_countMinFilter = 0;
			}

		}

		bool CEstimateSalvageStatus::GetResult()
		{
			return m_bOnSalvage;
		}

	}
}