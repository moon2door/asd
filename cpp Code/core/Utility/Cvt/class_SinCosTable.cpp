#include "class_SinCosTable.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795

class_SinCosTable::class_SinCosTable(void)
{
}


class_SinCosTable::~class_SinCosTable(void)
{
}


void class_SinCosTable::init_table(double angle_resolution)
{
	m_angle_resolution = angle_resolution;
	int table_size = (int)(360./angle_resolution+1);
	m_sin_table = new double[table_size];
	m_cos_table = new double[table_size];

	for(int i=0; i<table_size; i++)
	{
		double radian = (double)i * angle_resolution / 180. * PI;
		m_sin_table[i] = sin(radian);
		m_cos_table[i] = cos(radian);
	}
}


void class_SinCosTable::delete_table()
{
	delete[] m_sin_table;
	delete[] m_cos_table;
}


double class_SinCosTable::get_SinValue(double degree)
{
	int integer = (int)degree;
	double real = degree - (double)integer;
	if(abs(integer) > 360)
	{
		degree = double(integer % 360) + real;
	}
	
	if(degree < 0.0)
	{
		degree = degree + 360.0;
	}
	int index = int(degree / m_angle_resolution + 0.5) % int(360/m_angle_resolution);

	return m_sin_table[index];	
}


double class_SinCosTable::get_CosValue(double degree)
{
	int integer = (int)degree;
	double real = degree - (double)integer;
	if(abs(integer) > 360)
	{
		degree = double(integer % 360) + real;
	}	

	if(degree < 0.0)
	{
		degree = degree + 360.0;
	}
	int index = int(degree / m_angle_resolution + 0.5) % int(360/m_angle_resolution);

	return m_cos_table[index];
}