
#include "HMMStateFilter.h"

namespace SHI
{
	namespace Recognition
	{

		bool CHMMStateFilter::Create(int32_t nStates, float_t* _trans, float_t* _emits, float_t* _inits, int32_t queueSize)
		{
			if (nStates >= 3)
			{
				// 모델 초기화
				Eigen::MatrixXd trans(nStates, nStates);
				Eigen::MatrixXd emit(nStates, nStates);
				Eigen::MatrixXd init(1, nStates);

				// 전달함수 초기화
				trans.setZero();
				for (uint32_t i = 0; i < trans.rows(); i++)
				{
					for (uint32_t j = 0; j < trans.cols(); j++)
					{
						trans(i, j) = _trans[i*trans.cols() + j];
						emit(i, j) = _emits[i*trans.cols() + j];
					}
				}
				for (uint32_t i = 0; i < init.size(); i++)
				{
					init(i) = _inits[i];
				}

				m_hmm.Initialize(trans, emit, init);

				m_queueSize = queueSize;
			}
			return m_hmm.IsInitialized();
		}

		int32_t CHMMStateFilter::Push(int32_t obsservation)
		{
			int32_t ret = -1;
			if (m_hmm.IsInitialized())
			{

				// 입력 데이터 누적
				m_qObservation.push(obsservation);
				if (m_qObservation.size() > m_queueSize)
				{
					m_qObservation.pop();
				}

				// HMM 상태 추정
				int32_t size = m_qObservation.size();
				Eigen::MatrixXi seq(1, size);
				Eigen::MatrixXi states(1, size);
				states.setZero();
				for (uint32_t i = 0; i < size; i++)
				{
					seq.data()[i] = m_qObservation.front();
					m_qObservation.pop();
					m_qObservation.push(seq.data()[i]);
				}
				m_hmm.Viterbi(states, seq);

				// 현재 상태 반환
				m_seq = seq;
				m_states = states;
				ret = states(size - 1);
			}
			return ret;
		}

		void CHMMStateFilter::GetStates(Eigen::MatrixXi& states, Eigen::MatrixXi& seq)
		{
			states = m_states;
			seq = m_seq;
		}

	}
}