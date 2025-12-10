
#include <Recognition/HMMStateFilter.h>
#include <iostream>
#include <conio.h>
SHI::Recognition::CHMMStateFilter m_stateFilter;

void main()
{
	std::cout << " type input 0:정상, 1:주의, 2:위험, 3:정지 " << std::endl;

	float trans[] = {
		0.95, 0.05, 0.0, 0.0,
		0.05, 0.9, 0.05, 0.0,
		0.0, 0.05, 0.9, 0.05,
		0.0, 0.0, 0.05, 0.95 };

	//float emits[] = {
	//	0.9, 0.09, 0.001, 0.009,
	//	0.045, 0.9, 0.05, 0.005,
	//	0.005, 0.045, 0.9, 0.05,
	//	0.009, 0.001, 0.09, 0.9 };

	float emits[] = {
		0.8, 0.0, 0.0, 0.1,
		0.2, 0.8, 0.0, 0.1,
		0.0, 0.2, 0.8, 0.1,
		0.0, 0.0, 0.2, 0.7 };

	float inits[] = { 0.25, 0.25, 0.25, 0.25 };

	m_stateFilter.Create(4, trans, emits, inits, 10);

	while (m_stateFilter.IsCreated())
	{
		char ch = _getch();
		int input = -1;
		if (ch == '1') input = 1;
		else if (ch == '2') input = 2;
		else if (ch == '3') input = 3;
		else if (ch == '0') input = 0;
		else {}

		if (0 <= input && input <= 3)
		{



			int collisionTotal = m_stateFilter.Push(input);
			std::cout << " input: " << input << ", output: " << collisionTotal << std::endl;

			Eigen::MatrixXi seq, states;
			m_stateFilter.GetStates(states, seq);

			std::cout << " state filter " << seq << " => " << states << std::endl;
		}
	}
}