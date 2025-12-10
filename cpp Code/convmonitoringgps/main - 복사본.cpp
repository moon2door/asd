#include <stdio.h>
#include <vector>
#include <algorithm>
#include <Utility/Transform.h>
#include <Utility/Geometric/Geometric.h>
#include <Utility/SimpleEstimator.h>
#include <Config/CraneGps.h>
#include <Config/CraneInfo.h>
#include <Config/InitialAttitude.h>
#include <Data/StDistanceSocket.h>
#include <Routine/include/Base/CTime.h>

#include <Routine/include/Graphics/CImage.h>
#include <Routine/include/Visualization/CImageViewer.h>
#include <Utility/mathematics.h>
#include <Utility/UTM.h>
//#include <Utility/ConvertCoordinates/mgrs.h>
#pragma pack(push, 1)
typedef struct _StGpsData
{
	_StGpsData(float east, float north, float azimuth)
	{
		this->east = east;
		this->north = north;
		this->azimuth = azimuth;
		convEast = 0;
		convNorth = 0;
	}
	float east;
	float north;
	float azimuth;

	float convEast;
	float convNorth;
} StGpsData;
#pragma pop()

double conv_ddmm2d(double ddmm)
{
	double degree = floor(ddmm*0.01);
	double min = ddmm - degree * 100;
	return degree + (min / 60.0);
}

class CEstimator
	: public CSimpleEstimator<float, 3>
{
public:
	CEstimator(int pier, int crane, std::vector<StGpsData>& vData)
	{
		m_pier = pier;
		m_crane = crane;
		m_vData = vData;
	}

	virtual double Evaluate(std::vector<float> coefs)
	{
		std::vector<StGpsData> vData = m_vData;

		ConvertPosition(vData, coefs);

		printf("%d / %d (%.1lf%%) \n", GetCurrentIteration(), GetTotalItertion(), 100.0f * GetCurrentIteration() / (float)GetTotalItertion());

		//ShowGps(vData);
		
		float err = -CalcError(vData);

		return static_cast<double>(err);
	}

	void ConvertPosition(std::vector<StGpsData> &vData, std::vector<float> &coefs)
	{
		SHI::GpsAttitude gpsAttitude = SHI::GetGpsParam(m_pier, m_crane);
		SHI::CraneAttitude attitude;
		SHI::GetInitialAttitude(m_pier, m_crane, attitude);

		gpsAttitude.gpsOffsetX = coefs[0];
		gpsAttitude.gpsOffsetY = coefs[1];
		gpsAttitude.gpsOffsetAzimuth = coefs[2];

		for (unsigned int i = 0; i < vData.size(); i++)
		{
			float angle = -(vData[i].azimuth + gpsAttitude.gpsOffsetAzimuth);
			float c = cos(angle * SHI::Math::D2R);
			float s = sin(angle * SHI::Math::D2R);

			int towerIdx = SHI::PierK::JIB_TOWER;
			if (attitude.IsExact(SHI::PIERK, SHI::PierK::TTC23)) towerIdx = SHI::PierK::TTC_TOWER;
			// Offset crane origin to tower
			float gps2towerX = -gpsAttitude.gpsOffsetX + attitude.jointInfo[towerIdx].cx;
			float gps2towerY = -gpsAttitude.gpsOffsetY + attitude.jointInfo[towerIdx].cy;
			float tower2gpsX = -gps2towerX;
			float tower2gpsY = -gps2towerY;
			float globalTower2GpsX = tower2gpsX * c - tower2gpsY * s;
			float globalTower2GpsY = tower2gpsX * s + tower2gpsY * c;
			float globalGps2TowerX = -globalTower2GpsX;
			float globalGps2TowerY = -globalTower2GpsY;

			// Estimate position of tower in yard axis coordinate system
			vData[i].convEast = vData[i].east + globalGps2TowerX - gpsAttitude.pierOffsetX;
			vData[i].convNorth = vData[i].north + globalGps2TowerY - gpsAttitude.pierOffsetY;
			vData[i].east -= gpsAttitude.pierOffsetX;
			vData[i].north -= gpsAttitude.pierOffsetY;
		}

	}

	static inline int GetPixX(int margin, float east, float minEast, float ratio)
	{
		return margin / 2 + int((east - minEast) * ratio + 0.5f);
	}

	static inline int GetPixY(int margin, float north, float minNorth, float ratio)
	{
		return margin / 2 + int((north - minNorth) * ratio + 0.5f);
	}

	void ShowGps(std::vector<StGpsData>& vData)
	{
		float minEast = FLT_MAX;
		float maxEast = -FLT_MAX;
		float minNorth = FLT_MAX;
		float maxNorth = -FLT_MAX;
		for (unsigned int i = 0; i < vData.size(); i++)
		{
			minEast = (std::min)(vData[i].east, minEast);
			maxEast = (std::max)(vData[i].east, maxEast);
			minNorth = (std::min)(vData[i].north, minNorth);
			maxNorth = (std::max)(vData[i].north, maxNorth);

			minEast = (std::min)(vData[i].convEast, minEast);
			maxEast = (std::max)(vData[i].convEast, maxEast);
			minNorth = (std::min)(vData[i].convNorth, minNorth);
			maxNorth = (std::max)(vData[i].convNorth, maxNorth);
		}

		Routine::Graphics::CImage img;
		int width = 640;
		int height = 640;
		int margin = 100;
		float ratioWidth = 0;
		float ratioHeight = 0;
		if (abs(maxEast - minEast) > FLT_EPSILON) ratioWidth = width / (maxEast - minEast);
		if (abs(maxNorth - minNorth) > FLT_EPSILON) ratioHeight = height / (maxNorth - minNorth);
		float ratio = (std::min)(ratioWidth, ratioHeight);

		img.Create(width + margin, height + margin, Routine::Graphics::ImageType::IMG_BGR555);
		img.Clear(255);


		for (unsigned int i = 0; i < vData.size(); i++)
		{
			int px = GetPixX(margin, vData[i].east, minEast, ratio);
			int py = GetPixY(margin, vData[i].north, minNorth, ratio);
			img.DrawFillCircle(px, py, 2, Routine::Graphics::Black);

			int dx = GetPixX(margin, vData[i].convEast, minEast, ratio);
			int dy = GetPixY(margin, vData[i].convNorth, minNorth, ratio);
			img.DrawLine(px, py, dx, dy, Routine::Graphics::CImageColor(0, 255, 0));

			img.DrawFillCircle(dx, dy, 2, Routine::Graphics::Magenta);

			if(i%10 == 0)
			{
				char text[256] = "";
				sprintf_s(text, "%d", i);
				img.DrawString(px+10, py, text, Routine::Graphics::Black, 10);
			}
		}

		m_win.ShowImage(img);
	}

	static float CalcError(std::vector<StGpsData>& vData)
	{
		float sumX = 0;
		float sumY = 0;
		for (int i = 0; i < vData.size(); i++)
		{
			sumX += vData[i].convEast;
			sumY += vData[i].convNorth;
		}
		float averageX = sumX / vData.size();
		float averageY = sumY / vData.size();

		sumX = 0;
		sumY = 0;
		float sumXY = 0;
		for (int i = 0; i < vData.size(); i++)
		{
			sumX += (vData[i].convEast - averageX) * (vData[i].convEast - averageX);
			sumY += (vData[i].convNorth - averageY) * (vData[i].convNorth - averageY);
			sumXY += (vData[i].convEast - averageX) * (vData[i].convNorth - averageY);
		}
		float deviationX = sumX / vData.size();
		float deviationY = sumY / vData.size();
		//return abs(deviationX * deviationY);
		return sqrt(pow(deviationX,2) + pow(deviationY,2));
	}

	void WaitForWindowClose()
	{
		m_win.WaitForKey();
	}

	int m_pier;
	int m_crane;
	std::vector<StGpsData> m_vData;
	Routine::Visualization::CImageViewer m_win;
};

void main()
{
	std::vector<std::string> fnames =
	{
		"D:\\dev\\FM\\nocol\\binmonitoring\\LLC19\\2023-04-20_09-30_G3Dock_LLC19.distance",
		"D:\\dev\\FM\\nocol\\binmonitoring\\LLC20\\2023-04-20_10-50_G3Dock_LLC20.distance",
		"D:\\dev\\FM\\nocol\\binmonitoring\\LLC25\\2023-04-06_14-40_G4Dock_LLC25.distance",
		"D:\\dev\\FM\\nocol\\binmonitoring\\LLC26\\2023-04-05_15-00_G4Dock_LLC26.distance",
	};

	std::vector<StGpsData> gpsData;
	int pier = 0;
	int crane = 0;
	for (unsigned int i = 0; i < fnames.size(); i++)
	{
		FILE* fp = fopen(fnames[i].c_str(), "rb");
		if (fp)
		{
			printf("%s\n", fnames[i].c_str());
			Routine::CTime time;
			//double_t timestamp;
			unsigned char craneType;
			unsigned int size;

			byte temp[77];
			byte header[4];

			int count = 0;
			SHI::Data::StDistanceSocket *data = new SHI::Data::StDistanceSocket();
			int nRead = 0;
			do
			{
				nRead = fread(&temp, 1, sizeof(temp), fp);
				nRead = fread(&size, 1, sizeof(unsigned int), fp);
				nRead = fread(data, 1, size, fp);

				pier = data->attitude.pierId;
				crane = data->attitude.craneId;
				double latitude = conv_ddmm2d(data->attitude.gpsLatitude2);
				double longitude = conv_ddmm2d(data->attitude.gpsLongitude2);
				float azimuth = -data->attitude.azimuth;
				int zone = 0;
				char latitudeZone = 0;
				char squareZone1 = 0;
				char squareZone2 = 0;
				double east = 0;
				double north = 0;
				
				if(latitude < DBL_EPSILON && longitude < DBL_EPSILON) continue;
				//WGS84ToMGRS(latitude, longitude, &zone, &latitudeZone, &squareZone1, &squareZone2, &east, &north);
				DevLib::Geometric::WGS84ToMGRS(latitude, longitude, &zone, &latitudeZone, &squareZone1, &squareZone2, &east, &north);
				
				if (abs(azimuth) > FLT_EPSILON)
				{
					if (120 <= count && count <= 380)
					{
						gpsData.push_back(StGpsData(east, north, azimuth));
						printf("##[%d] %lf deg -- %lf, %lf\n", count, azimuth, east, north);
					}
				}
				count++;
			} while (nRead > 0);

			delete data;
			fclose(fp);
		}
	}
	
	std::vector<float> minRange = { -20.0f, -20.0f, 0.0f };
	std::vector<float> maxRange = { +20.0f, +20.0f, 360.0f };
	std::vector<float> resolution = { 1.0f, 1.0f, 10.0f };
	printf("# Run %d data\n", gpsData.size());
	if (gpsData.size() > 0)
	{
		CEstimator e(pier, crane, gpsData);
		e.Create(minRange, maxRange, resolution);
		e.Run();
		
		std::vector<float> result = e.GetResult();
		
		float targetX = result[0];
		float targetY = result[1];
		float targetAngle = result[2];
		minRange = { targetX - 0.1f, targetY - 0.1f, targetAngle - 1.0f };
		maxRange = { targetX + 0.1f, targetY + 0.1f, targetAngle + 1.0f };
		resolution = { 0.01f, 0.01f, 0.1f };
		e.Create(minRange, maxRange, resolution);
		e.Run();
		result = e.GetResult();
		
		//result[0] = -0.060000, result[1] = -6.949996, result[2] = 39.299995;
		// LLC19 retult = 0.950000, -5.019998, 49.599991, error = 0.000149
		//       retult = 3.000000, -5.000000, 100.000000, error = 0.001215
		// LLC20 retult = -3.060000, -2.990000, 289.800049, error = 0.000068
		// LLC25 retult = 1.900000, -3.000000, 80.399979, error = 0.000020
		//       retult = 1.900000, -2.900000, 79.599991, error = 0.000286
		//       retult = 1.900000, -7.899996, 79.599991, error = 0.000289    
		// LLC26 retult = -0.080000, -7.939997, 39.000000, error = 0.000165
		//       retult = -0.080000, -7.939997, 39.000000, error = 0.000166
		//       retult = -0.060000, -6.949996, 39.299995, error = 0.000075
		
		e.ConvertPosition(gpsData, result);


		FILE* fp;
		if (fopen_s(&fp, "test2.csv", "wb") == 0)
		{
			char buffer[1024] = "num, azimuth, east, north,\r\n";
			fwrite(buffer, 1, sizeof(buffer), fp);
			for (int i = 0; i < gpsData.size(); i++)
			{
				char bufferData[8192] = "";
				sprintf_s(bufferData, "%d, %lf, %lf, %lf, %lf, %lf\r\n", i, gpsData[i].east, gpsData[i].north, gpsData[i].azimuth, gpsData[i].convEast, gpsData[i].convNorth);
				fwrite(bufferData, 1, sizeof(bufferData), fp);  // NOLINT(cert-err33-c)	
			}
			fclose(fp);  // NOLINT(cert-err33-c)
		}
		printf(" > retult = %lf, %lf, %lf, error = %lf \n", result[0], result[1], result[2], e.CalcError(gpsData));

		e.ShowGps(gpsData);
		e.WaitForWindowClose();
	}
}