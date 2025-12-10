#include "BinaryHMM.h"

namespace SHI
{
	namespace Recognition
	{
		void CBinaryHMM::Initialize(double_t initX, double_t transXX, double_t transXnX, double_t emitEX, double_t emitEnX)
		{
			m_x = initX;
			m_transXX = transXX;
			m_transXnX = transXnX;
			m_emitEX = emitEX;
			m_emitEnX = emitEnX;
		}

		double_t CBinaryHMM::Update(bool e)
		{
			// dynamic update
			double_t _x = m_transXX * m_x + m_transXnX * (1 - m_x);
			double_t _nx = (1 - m_transXX) * m_x + (1 - m_transXnX) * (1 - m_x);

			// observation update
			if (e)
			{
				double_t x = m_emitEX * _x;
				double_t nx = m_emitEnX * _nx;
				double_t a1 = 1.0 / (x + nx);
				x *= a1;
				nx *= a1;
				m_x = x;
			}
			else
			{
				double_t x = (1 - m_emitEX) * _x;
				double_t nx = (1 - m_emitEnX) * _nx;
				double_t a1 = 1.0 / (x + nx);
				x *= a1;
				nx *= a1;
				m_x = x;
			}
			return m_x;
		}
	}
}