#include <Utility/mathematics.h>

namespace SHI
{
	namespace Math
	{
		double_t GetRotateX(double_t degree, double_t x, double_t y)
		{
			return ((cos(degree * D2R) * x) + (sin(degree * D2R) * y));
		}

		double_t GetRotateY(double_t degree, double_t x, double_t y)
		{
			return (-(sin(degree * D2R) * x) + (cos(degree * D2R) * y));
		}

		float_t GetRotateX(float_t degree, float_t x, float_t y)
		{
			return ((cos(degree * D2Rf) * x) + (sin(degree * D2Rf) * y));
		}

		float_t GetRotateY(float_t degree, float_t x, float_t y)
		{
			return (-(sin(degree * D2Rf) * x) + (cos(degree * D2Rf) * y));
		}

	}
}