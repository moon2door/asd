#pragma once

#include "BinaryHMMFilter.h"
#include <math.h>
#include <stdio.h>

namespace SHI
{
	namespace Recognition
	{
		static const double_t __d2r = 3.14159265358979323846 / 180.0;
		static const double_t __r2d = 1.0 / __d2r;

		CMovingMean::CMovingMean() : m_mean(0), m_meanc(0), m_means(0), m_count(0) { }

		double_t CMovingMean::Update(double_t val)
		{
			m_count++;
			if (m_bAngleMean)
			{
				double_t s = sin(val * __d2r);
				double_t c = cos(val * __d2r);
				m_means = ((m_count - 1) * m_means + s) / m_count;
				m_meanc = ((m_count - 1) * m_meanc + c) / m_count;
				m_mean = atan2(m_means, m_meanc) * __r2d;
				while (m_mean > 360.0) m_mean -= 360.0;
				while (m_mean < 0) m_mean += 360.0;
			}
			else
			{
				m_mean = ((m_count - 1) * m_mean + val) / m_count;
			}
			return m_mean;
		};
		void CMovingMean::Reset()
		{
			m_mean = 0;
			m_meanc = 0;
			m_means = 0;
			m_count = 0;
		}

		void CMovingMean::SetAngleMean(bool bAngleMean)
		{
			m_bAngleMean = bAngleMean;
			Reset();
		}

		void CBinaryHMMFilter::Initialize(double_t initX, double_t transXX, double_t transXnX, double_t emitEX, double_t emitEnX, double_t df, bool bAngleFilter /*= false*/)
		{
			m_hmm.Initialize(initX, transXX, transXnX, emitEX, emitEnX);
			m_df = df;
			m_prev = 0;
			m_bMoving = initX > 0.5;
			m_mean.SetAngleMean(bAngleFilter);
		}

		double_t CBinaryHMMFilter::Update(double_t f)
		{
			bool bMoving = m_df < fabs(m_prev - f);
			// 이동 여부 판단
			m_bMoving = 0.5 < m_hmm.Update(bMoving);
			if (m_bMoving)
			{
				m_mean.Reset();
			}

			// 누적 평균
			m_prev = m_mean.Update(f);

			// 결과 반환
			return m_prev;
		}
	}
}