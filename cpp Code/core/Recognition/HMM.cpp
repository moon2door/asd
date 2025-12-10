#include "HMM.h"

namespace SHI
{
	namespace Recognition
	{

		CHMM::CHMM()
			: m_bInitialized(false)
		{
		}

		void CHMM::Initialize(const Eigen::MatrixXd &trans, const Eigen::MatrixXd &emission, const Eigen::MatrixXd &init)
		{
			m_bInitialized = true;
			m_trans = trans;
			m_emission = emission;
			m_init = init;
		}
		
		void CHMM::Viterbi(Eigen::MatrixXi &stats, const Eigen::MatrixXi &seq)
		{
			Eigen::MatrixXd trans = m_trans;
			Eigen::MatrixXd emission = m_emission;
			Eigen::MatrixXd init = m_init;

			correctModel(trans, emission, init);

			int32_t nseq = seq.cols();
			int32_t nstates = trans.cols();
			int32_t nobs = emission.cols();
			Eigen::MatrixXd v(nstates, nseq);
			Eigen::MatrixXi path(nstates, nseq);
			Eigen::MatrixXi newpath(nstates, nseq);
			path.setZero();
			newpath.setZero();

			for (int y = 0; y < nstates; y++)
			{
				v(y, 0) = log(init(0, y)) + log(emission(y, seq(0, 0)));
				path(y, 0) = y;
			}
			double_t maxp, p;
			int32_t state;
			for (int t = 1; t < nseq; t++)
			{
				for (int y = 0; y < nstates; y++)
				{
					maxp = -DBL_MAX;
					state = y;
					for (int y0 = 0; y0 < nstates; y0++)
					{
						p = v(y0, t - 1) + log(trans(y0, y)) + log(emission(y, seq(0, t)));
						if (maxp < p)
						{
							maxp = p;
							state = y0;
						}
					}
					v(y, t) = maxp;
					for (int t1 = 0; t1 < t; t1++)
						newpath(y, t1) = path(state, t1);
					newpath(y, t) = y;
				}

				path = newpath;
			}
			maxp = -DBL_MAX;
			for (int y = 0; y < nstates; y++)
			{
				if (maxp < v(y, nseq - 1))
				{
					maxp = v(y, nseq - 1);
					state = y;
				}
			}
			stats = path.row(state);
		}

		void CHMM::Decode(const Eigen::MatrixXi &seq, const Eigen::MatrixXd &_TRANS, const Eigen::MatrixXd &_EMIS, const Eigen::MatrixXd &_INIT, double_t &logpseq, Eigen::MatrixXd &PSTATES /*= Eigen::MatrixXd()*/, Eigen::MatrixXd &FORWARD /*= Eigen::MatrixXd()*/, Eigen::MatrixXd &BACKWARD /*= Eigen::MatrixXd()*/)
		{
			/* A Revealing Introduction to Hidden Markov Models, Mark Stamp */
			// 1. Initialization
			Eigen::MatrixXd TRANS = _TRANS;
			Eigen::MatrixXd EMIS = _EMIS;
			Eigen::MatrixXd INIT = _INIT;
			correctModel(TRANS, EMIS, INIT);
			int32_t T = seq.cols(); // number of element per sequence
			int32_t C = seq.rows(); // number of sequences
			int32_t N = TRANS.rows(); // number of states | also N = TRANS.cols | TRANS = A = {a_{i,j}} - NxN
			int32_t M = EMIS.cols(); // number of observations | EMIS = B = {b_{j}(k)} - NxM				
							   // compute a_{0}

			FORWARD = Eigen::MatrixXd(N, T);
			Eigen::MatrixXd c(1, T); c(0, 0) = 0;
			for (int32_t i = 0; i < N; i++)
			{
				FORWARD(i, 0) = INIT(0, i)*EMIS(i, seq(0, 0));
				c(0, 0) += FORWARD(i, 0);
			}
			// scale the a_{0}(i)
			c(0, 0) = 1 / c(0, 0);
			for (int32_t i = 0; i < N; i++)
				FORWARD(i, 0) *= c(0, 0);
			// 2. The a-pass
			// compute a_{t}(i)
			for (int32_t t = 1; t < T; t++)
			{
				c(0, t) = 0;
				for (int32_t i = 0; i < N; i++)
				{
					FORWARD(i, t) = 0;
					for (int32_t j = 0; j < N; j++)
						FORWARD(i, t) += FORWARD(i, t - 1)*TRANS(j, i);
					FORWARD(i, t) = FORWARD(i, t) * EMIS(i, seq(0, t));
					c(0, t) += FORWARD(i, t);
				}
				// scale a_{t}(i)
				c(0, t) = 1 / c(0, t);
				for (int32_t i = 0; i < N; i++)
					FORWARD(i, t) = c(0, t)*FORWARD(i, t);
			}
			// 3. The B-pass
			BACKWARD = Eigen::MatrixXd(N, T);
			// Let B_{t-1}(i) = 1 scaled by C_{t-1}
			for (int32_t i = 0; i < N; i++)
				BACKWARD(i, T - 1) = c(0, T - 1);
			// B-pass
			for (int32_t t = T - 2; t > -1; t--)
				for (int32_t i = 0; i < N; i++)
				{
					BACKWARD(i, t) = 0;
					for (int32_t j = 0; j < N; j++)
						BACKWARD(i, t) += TRANS(i, j)*EMIS(j, seq(0, t + 1))*(j, t + 1);
					// scale B_{t}(i) with same scale factor as a_{t}(i)
					BACKWARD(i, t) *= c(0, t);
				}
			// 4. 
			// Compute Y_{t}(i,j) : The probability of being in state i at time t and transiting to state j at time t+1
			// Compute Y_{t}(i) 
			double_t denom;
			int32_t index;

			PSTATES = Eigen::MatrixXd(N, T);
			Eigen::MatrixXd YNN(N*N, T);
			for (int32_t t = 0; t < T - 1; t++)
			{
				denom = 0;
				for (int32_t i = 0; i < N; i++)
					for (int32_t j = 0; j < N; j++)
						denom += FORWARD(i, t)*TRANS(i, j)*EMIS(j, seq(0, t + 1))*BACKWARD(j, t + 1);
				index = 0;
				for (int32_t i = 0; i < N; i++)
				{
					PSTATES(i, t) = 0;
					for (int32_t j = 0; j < N; j++)
					{
						YNN(index, t) = (FORWARD(i, t)*TRANS(i, j)*EMIS(j, seq(0, t + 1))*BACKWARD(j, t + 1)) / denom;
						PSTATES(i, t) += YNN(index, t);
						index++;
					}
				}
			}
			// 6. Compute log[P(O|y)]
			logpseq = 0;
			for (int32_t i = 0; i < T; i++)
				logpseq += log(c(0, i));
			logpseq *= -1;
		}

		void CHMM::correctModel(Eigen::MatrixXd &trans, Eigen::MatrixXd &emission, Eigen::MatrixXd &init)
		{
			double_t eps = 1e-30;
			for (int32_t i = 0; i < emission.rows(); i++)
				for (int32_t j = 0; j < emission.cols(); j++)
					if (emission(i, j) == 0)
						emission(i, j) = eps;
			for (int32_t i = 0; i < trans.rows(); i++)
				for (int32_t j = 0; j < trans.cols(); j++)
					if (trans(i, j) == 0)
						trans(i, j) = eps;
			for (int32_t i = 0; i < init.cols(); i++)
				if (init(0, i) == 0)
					init(0, i) = eps;
			double_t sum;
			for (int32_t i = 0; i < trans.rows(); i++)
			{
				sum = 0;
				for (int32_t j = 0; j < trans.cols(); j++)
					sum += trans(i, j);
				for (int32_t j = 0; j < trans.cols(); j++)
					trans(i, j) /= sum;
			}
			for (int32_t i = 0; i < emission.rows(); i++)
			{
				sum = 0;
				for (int32_t j = 0; j < emission.cols(); j++)
					sum += emission(i, j);
				for (int32_t j = 0; j < emission.cols(); j++)
					emission(i, j) /= sum;
			}
			sum = 0;
			for (int32_t j = 0; j < init.cols(); j++)
				sum += init(0, j);
			for (int32_t j = 0; j < init.cols(); j++)
				init(0, j) /= sum;
		}

	}
}