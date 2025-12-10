#include <Utility/Geometric.h>
#include <Utility/mathematics.h>
#include <Utility/ConvertCoordinates/mgrs.h>

namespace SHI { namespace Geometric
{

	bool WGS84ToMGRS(double Latitude,double Longitude, int* iZone,char* LatitudeZone,char* SquareZone1,char* SquareZone2,double* East,double* North)
	{
		double east = 0.0, north = 0.0;
		char MGRS[256] = "";

		long err = Convert_Geodetic_To_MGRS2(Latitude * SHI::Math::D2R, Longitude * SHI::Math::D2R, 5, MGRS, iZone, &east, &north);
		
		*LatitudeZone	= MGRS[0];
		*SquareZone1	= MGRS[1];
		*SquareZone2	= MGRS[2];	
		*East			= east -	(static_cast<int>(east)	/ 100000) * 100000;	// 다섯자리까지만 사용
		*North			= north -	(static_cast<int>(north)	/ 100000) * 100000;
		
		bool bRet = true;
		if( err != MGRS_NO_ERROR) bRet = false;
		
		return bRet;
	}

	bool MGRSToWGS84(int iZone, char LatitudeZone, char SquareZone1, char SquareZone2, double East, double North, double *Latitude, double *Longitude)
	{
		long err = Convert_MGRS_To_Geodetic2(iZone, LatitudeZone, SquareZone1, SquareZone2, East, North, Latitude, Longitude);

		bool bRet = true;
		if( err != MGRS_NO_ERROR) bRet = false;

		return bRet;
	}

} }