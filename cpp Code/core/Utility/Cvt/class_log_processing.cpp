#include "class_log_processing.h"

#include "DevLib/Include/Utility/Config.h"
#include "DevLib/Include/Mathematics/Mathematics.h"

using namespace DevLib;
using namespace DevLib::Utility;
using namespace DevLib::Utility::Config;


class_log_processing::class_log_processing(int rotorID)
{	
	m_packet_count = 0;
	m_count_of_3d_data = 0;

	m_rpm = GetConfigInt("Velodyne", "RPM", 1200, GetMakeConfigName());
	printf("Velodyne RPM = %d\n", m_rpm);

	m_sin_cos.init_table(0.01);

	m_pre_velodyneAngle = 1000.;	// 존재하지 않는 값으로 초기화(실재범위:0~359.99)
	m_pre_velodyneTimestamp = 0.;
	mb_is_first_time = true;

	m_yaw_angle = 0.0;
	m_actuator_time = 0.0;
	m_actuator_rpm = 0.0;	

	m_velodyne_rpm = 360;

	// 초기 파라미터
	m_encoder_pulse = GetConfigInt("Rotor Parameter ", "EncoderPulse", 52224, GetMakeConfigName());
	m_harmonic_drive = GetConfigInt("Rotor Parameter ", "HarmonicDrive", 1, GetMakeConfigName());
	m_resolution = GetConfigInt("Rotor Parameter ", "Resolution", 52224, GetMakeConfigName());
	m_ABS_zero = GetConfigInt("Rotor Parameter ", "AbsZero", 0, GetMakeConfigName());

	char	str_obj[128];
	/* xyz변환 파라미터 초기화 */
	sprintf_s(str_obj, "parameter_%d", rotorID);
	m_tf_param.in_x = (float)GetConfigDouble(str_obj, "In_X", 0.0, GetMakeConfigName());
	m_tf_param.in_y = (float)GetConfigDouble(str_obj, "In_Y", 0.0, GetMakeConfigName());
	m_tf_param.in_z = (float)GetConfigDouble(str_obj, "In_Z", 0.0, GetMakeConfigName());
	m_tf_param.in_roll = (float)GetConfigDouble(str_obj, "In_Roll", 0.0, GetMakeConfigName());
	m_tf_param.in_pitch = (float)GetConfigDouble(str_obj, "In_Pitch", 0.0, GetMakeConfigName());
	m_tf_param.in_yaw = (float)GetConfigDouble(str_obj, "In_Yaw", 0.0, GetMakeConfigName());
	m_tf_param.ex_x = (float)GetConfigDouble(str_obj, "Ex_X", 0.0, GetMakeConfigName());
	m_tf_param.ex_y = (float)GetConfigDouble(str_obj, "Ex_Y", 0.0, GetMakeConfigName());
	m_tf_param.ex_z = (float)GetConfigDouble(str_obj, "Ex_Z", 0.0, GetMakeConfigName());
	m_tf_param.ex_roll = (float)GetConfigDouble(str_obj, "Ex_Roll", 0.0, GetMakeConfigName());
	m_tf_param.ex_pitch = (float)GetConfigDouble(str_obj, "Ex_Pitch", 0.0, GetMakeConfigName());
	m_tf_param.ex_yaw = (float)GetConfigDouble(str_obj, "Ex_Yaw", 0.0, GetMakeConfigName());
	m_tf_param.vlp_x = (float)GetConfigDouble(str_obj, "VLP_X", 0.0, GetMakeConfigName());
	m_tf_param.vlp_y = (float)GetConfigDouble(str_obj, "VLP_Y", 0.0, GetMakeConfigName());
	m_tf_param.vlp_z = (float)GetConfigDouble(str_obj, "VLP_Z", 0.0, GetMakeConfigName());
	m_tf_param.vlp_roll = (float)GetConfigDouble(str_obj, "VLP_Roll", 0.0, GetMakeConfigName());
	m_tf_param.vlp_pitch = (float)GetConfigDouble(str_obj, "VLP_Pitch", 0.0, GetMakeConfigName());
	m_tf_param.vlp_yaw = (float)GetConfigDouble(str_obj, "VLP_Yaw", 0.0, GetMakeConfigName());
	m_velodyne_v_angle_offset = (float)GetConfigDouble(str_obj, "VLP_VAngle_Offset", 0.0, GetMakeConfigName());
	m_axis_delta = 0.167;
}


class_log_processing::~class_log_processing()
{

}


void class_log_processing::convert_Endian(unsigned short* p_data, int size)
{
	char reverce_char[2];
	char* p_src_data = (char*)p_data;

	for (int i = 0; i < size; i++)
	{
		reverce_char[0] = p_src_data[i * 2];
		reverce_char[1] = p_src_data[i * 2 + 1];
		p_src_data[i * 2] = reverce_char[1];
		p_src_data[i * 2 + 1] = reverce_char[0];
	}
}

unsigned int class_log_processing::parsing_velodyne_log(struct_log_data log_data, struct_sub_point_cloud* p_point_cloud)
{
	if (!mb_is_first_time)
	{
		m_count_of_3d_data = 0;
		double velo_time;
		// 센서 데이터 파싱
		unsigned char hour = m_pre_data_buf.actuator_packet.hour;
		unsigned char minute = m_pre_data_buf.actuator_packet.minute;
		unsigned short msec = m_pre_data_buf.actuator_packet.msec;
		convert_Endian(&msec, 2);

		// 구동기 시간정보를 sec로 환산(1시간범위 내에서만 사용)
		m_actuator_time = (double)minute*60. + (double)msec*0.001;

		unsigned short yaw_enc = m_pre_data_buf.actuator_packet.yaw_enc;
		convert_Endian(&yaw_enc, 2);
		m_yaw_angle = yaw_enc / ((m_encoder_pulse*m_harmonic_drive) / 360.0);

		unsigned short yaw_rpm = m_pre_data_buf.actuator_packet.yaw_rpm;
		convert_Endian(&yaw_rpm, 2);
		m_actuator_rpm = (double)yaw_rpm / 100.0;

		/* 각도 범위는 1~360도로 제한 한다 */
		if (m_yaw_angle > 360.0)
		{
			int integer = (int)m_yaw_angle;
			double real = m_yaw_angle - (double)integer;
			m_yaw_angle = double(integer % 360) + (double)real;
		}
		else if (m_yaw_angle < 0.0)
		{
			m_yaw_angle += 360.0;
		}
		else if (m_yaw_angle == 0.0)
		{
			m_yaw_angle = 360.0;
		}
	
		double delta_angle = 0.;
		if (m_pre_velodyneAngle == 1000.)
		{
			m_pre_velodyneAngle = log_data.velodyne_packet.DataBlock[0].Azimuth * 0.01;
		}
		else
		{
			delta_angle = log_data.velodyne_packet.DataBlock[0].Azimuth * 0.01 - m_pre_velodyneAngle;
			delta_angle = delta_angle < 0. ? delta_angle + 360. : delta_angle;
			m_pre_velodyneAngle = log_data.velodyne_packet.DataBlock[0].Azimuth * 0.01;
		}
		double delta_time = 0.;
		if (m_pre_velodyneTimestamp == 0.)
		{
			m_pre_velodyneTimestamp = log_data.velodyne_packet.Timestamp;
		}
		else
		{
			delta_time = log_data.velodyne_packet.Timestamp - m_pre_velodyneTimestamp;
			delta_time = delta_time < 0. ? delta_time + (60 * 60 * 1000000) : delta_time;
			m_pre_velodyneTimestamp = log_data.velodyne_packet.Timestamp;
		}
		if (delta_angle != 0. && delta_time != 0.)
		{
			m_velodyne_rpm = (delta_angle * (60000000. / delta_time)) / 360.;
		}

		/* x,y,z 계산 */
		for (int iBlock = 0; iBlock < 12; iBlock++)
		{
			double  stdAngle = m_pre_data_buf.velodyne_packet.DataBlock[0].Azimuth * 0.01; // -- 기준 각도(deg)

			for (int iLayer = 0; iLayer < 32; iLayer++)
			{
				// 시간(usec), 각도(deg) 오프셋 계산
				int seqIndex = iBlock * 2 + iLayer / 16;
				int layerIndex = iLayer % 16;
				double timeOffset = (55.296 * seqIndex) + (2.304 * layerIndex);
				double angleOffset = (m_velodyne_rpm / 60000000.0) * 360.0 * timeOffset;
				double curAngleH = stdAngle + angleOffset;
				curAngleH = curAngleH > 360. ? curAngleH - 360. : curAngleH;

				velo_time = m_pre_data_buf.velodyne_packet.Timestamp;
				double velodyne_time = m_pre_data_buf.velodyne_packet.Timestamp;
				double angleOffset_actuator = (m_actuator_rpm / 60000000.0) * 360.0 * ((velodyne_time - (m_actuator_time*1000000.0)) + timeOffset);
				double distance = m_pre_data_buf.velodyne_packet.DataBlock[iBlock].LayerData[iLayer].Distance*0.002;
			
				float v = LAYER_2_ANGLE[layerIndex] + m_velodyne_v_angle_offset;
				if (m_pre_data_buf.velodyne_packet.DataBlock[iBlock].LayerData[iLayer].Distance*0.002 > 0.5)
				{
					//// 센서 기준 축(X:우측 Y:전방 Z:상방) --> 차량 기준축(X:전방 Y:우측 Z:하방) 으로 전환
					//double sensor_3x = m_pre_data_buf.velodyne_packet.DataBlock[iBlock].LayerData[iLayer].Distance * m_sin_cos.get_CosValue(v) * m_sin_cos.get_CosValue(curAngleH)*0.002;
					//double sensor_3y = m_pre_data_buf.velodyne_packet.DataBlock[iBlock].LayerData[iLayer].Distance * m_sin_cos.get_CosValue(v) * m_sin_cos.get_SinValue(curAngleH)*0.002;
					//double sensor_3z = -m_pre_data_buf.velodyne_packet.DataBlock[iBlock].LayerData[iLayer].Distance * m_sin_cos.get_SinValue(v)*0.002;

					//// Velodyne센서 내부(레이이저 송수신모듈과 케이스간) 롤 에러 // 회전축 x 기준축 y [Velodyne 기준]
					//double vlp_rx = sensor_3x;
					//double vlp_ry = sensor_3y*m_sin_cos.get_CosValue(m_tf_param.vlp_roll) - sensor_3z*m_sin_cos.get_SinValue(m_tf_param.vlp_roll);
					//double vlp_rz = sensor_3y*m_sin_cos.get_SinValue(m_tf_param.vlp_roll) + sensor_3z*m_sin_cos.get_CosValue(m_tf_param.vlp_roll);

					//// Velodyne센서 내부(레이이저 송수신모듈과 케이스간) 피치 에러 // 회전축 y 기준축 z [Velodyne 기준]
					//double vlp_py = vlp_ry;
					//double vlp_pz = vlp_rz*m_sin_cos.get_CosValue(m_tf_param.vlp_pitch) - vlp_rx*m_sin_cos.get_SinValue(m_tf_param.vlp_pitch);
					//double vlp_px = vlp_rz*m_sin_cos.get_SinValue(m_tf_param.vlp_pitch) + vlp_rx*m_sin_cos.get_CosValue(m_tf_param.vlp_pitch);

					////  Velodyne센서 내부(레이이저 송수신모듈과 케이스간) 요 에러 // 회전축 z 기준축 x [Velodyne 기준]
					//double vlp_yz = vlp_pz;
					//double vlp_yx = vlp_px*m_sin_cos.get_CosValue(m_tf_param.vlp_yaw) - vlp_py*m_sin_cos.get_SinValue(m_tf_param.vlp_yaw);
					//double vlp_yy = vlp_px*m_sin_cos.get_SinValue(m_tf_param.vlp_yaw) + vlp_py*m_sin_cos.get_CosValue(m_tf_param.vlp_yaw);

					//// 구동장치에 장착된 센서의 롤 // 회전축 x 기준축 y [구동장치 기준]
					//double sensor_mount_rx = vlp_yx;
					//double sensor_mount_ry = vlp_yy*m_sin_cos.get_CosValue(m_tf_param.in_roll) - vlp_yz*m_sin_cos.get_SinValue(m_tf_param.in_roll);
					//double sensor_mount_rz = vlp_yy*m_sin_cos.get_SinValue(m_tf_param.in_roll) + vlp_yz*m_sin_cos.get_CosValue(m_tf_param.in_roll);

					//// 구동장치에 장착된 센서의 피치 // 회전축 y 기준축 z [구동장치 기준]
					//double sensor_mount_py = sensor_mount_ry;
					//double sensor_mount_pz = sensor_mount_rz*m_sin_cos.get_CosValue(m_tf_param.in_pitch) - sensor_mount_rx*m_sin_cos.get_SinValue(m_tf_param.in_pitch);
					//double sensor_mount_px = sensor_mount_rz*m_sin_cos.get_SinValue(m_tf_param.in_pitch) + sensor_mount_rx*m_sin_cos.get_CosValue(m_tf_param.in_pitch);

					//// 구동장치에 장착된 센서의 요 // 회전축 z 기준축 x [구동장치 기준]
					//double sensor_mount_yz = sensor_mount_pz;
					//double sensor_mount_yx = sensor_mount_px*m_sin_cos.get_CosValue(m_tf_param.in_yaw) - sensor_mount_py*m_sin_cos.get_SinValue(m_tf_param.in_yaw);
					//double sensor_mount_yy = sensor_mount_px*m_sin_cos.get_SinValue(m_tf_param.in_yaw) + sensor_mount_py*m_sin_cos.get_CosValue(m_tf_param.in_yaw);

					//// 구동장치의 회전구동 적용 // 회전축 z 기준축 x [구동장치 기준]
					//double rotation_yz = sensor_mount_yz;
					//double rotation_yx = sensor_mount_yx*m_sin_cos.get_CosValue(m_yaw_angle + angleOffset_actuator) - sensor_mount_yy*m_sin_cos.get_SinValue(m_yaw_angle + angleOffset_actuator);
					//double rotation_yy = sensor_mount_yx*m_sin_cos.get_SinValue(m_yaw_angle + angleOffset_actuator) + sensor_mount_yy*m_sin_cos.get_CosValue(m_yaw_angle + angleOffset_actuator);

					//// 크레인에 장착된 구동장치 피치 // 회전축 y 기준축 x [크레인 기준]
					//double rotor_mount_py = rotation_yy;
					//double rotor_mount_pz = rotation_yz*m_sin_cos.get_CosValue(m_tf_param.ex_pitch) - rotation_yx*m_sin_cos.get_SinValue(m_tf_param.ex_pitch);
					//double rotor_mount_px = rotation_yz*m_sin_cos.get_SinValue(m_tf_param.ex_pitch) + rotation_yx*m_sin_cos.get_CosValue(m_tf_param.ex_pitch);

					//// 크레인에 장착된 구동장치 요 // 회전축 z 기준축 x [크레인 기준]
					//double rotor_mount_yz = rotor_mount_pz;
					//double rotor_mount_yx = rotor_mount_px*m_sin_cos.get_CosValue(m_tf_param.ex_yaw) - rotor_mount_py*m_sin_cos.get_SinValue(m_tf_param.ex_yaw);
					//double rotor_mount_yy = rotor_mount_px*m_sin_cos.get_SinValue(m_tf_param.ex_yaw) + rotor_mount_py*m_sin_cos.get_CosValue(m_tf_param.ex_yaw);

					//// 크레인에 장착된 구동장치 롤 // 회전축 x 기준축 y [크레인 기준]
					//double rotor_mount_rx = rotor_mount_yx;
					//double rotor_mount_ry = rotor_mount_yy*m_sin_cos.get_CosValue(m_tf_param.ex_roll) - rotor_mount_yz*m_sin_cos.get_SinValue(m_tf_param.ex_roll);
					//double rotor_mount_rz = rotor_mount_yy*m_sin_cos.get_SinValue(m_tf_param.ex_roll) + rotor_mount_yz*m_sin_cos.get_CosValue(m_tf_param.ex_roll);

					// 구동장치의 회전구동 적용 // 회전축 z 기준축 x [구동장치 기준]
					curAngleH += m_tf_param.vlp_yaw;
					if (curAngleH >= 360.)
					{
						curAngleH -= 360.;
					}
					else if (curAngleH < 0.)
					{
						curAngleH += 360.;
					}

					// 센서 기준 축(X:우측 Y:전방 Z:상방) --> 구동기 기준축(X:전방 Y:우측 Z:하방) 으로 전환
					double sensor_3x = distance * m_sin_cos.get_CosValue(v) * m_sin_cos.get_CosValue(curAngleH);
					double sensor_3y = distance * m_sin_cos.get_CosValue(v) * m_sin_cos.get_SinValue(curAngleH);
					double sensor_3z = -distance * m_sin_cos.get_SinValue(v);

					// Velodyne센서 내부(레이저 송수신모듈과 케이스간) 롤 에러 // 회전축 x 기준축 y [Velodyne 기준]
					//double vlp_rx = sensor_3x;
					//double vlp_ry = sensor_3y*m_sin_cos.get_CosValue(m_tf_param.vlp_roll) - sensor_3z*m_sin_cos.get_SinValue(m_tf_param.vlp_roll);
					//double vlp_rz = sensor_3y*m_sin_cos.get_SinValue(m_tf_param.vlp_roll) + sensor_3z*m_sin_cos.get_CosValue(m_tf_param.vlp_roll);
					// roll변환은 의미 없으므로 roll값은 다른 용도로 사용
					double vlp_rx = sensor_3x;
					double vlp_ry = sensor_3y;
					double vlp_rz = sensor_3z;

					// Velodyne센서 내부(레이저 송수신모듈과 케이스간) 피치 에러 // 회전축 y 기준축 z [Velodyne 기준]
					double vlp_py = vlp_ry;
					double vlp_pz = vlp_rz*m_sin_cos.get_CosValue(m_tf_param.vlp_pitch) - vlp_rx*m_sin_cos.get_SinValue(m_tf_param.vlp_pitch);
					double vlp_px = vlp_rz*m_sin_cos.get_SinValue(m_tf_param.vlp_pitch) + vlp_rx*m_sin_cos.get_CosValue(m_tf_param.vlp_pitch);

					// Velodyne센서 내부(레이저 송수신모듈과 케이스간) 요 에러 // 회전축 z 기준축 x [Velodyne 기준]
					//double vlp_yz = vlp_pz;
					//double vlp_yx = vlp_px*m_sin_cos.get_CosValue(m_tf_param.vlp_yaw) - vlp_py*m_sin_cos.get_SinValue(m_tf_param.vlp_yaw);
					//double vlp_yy = vlp_px*m_sin_cos.get_SinValue(m_tf_param.vlp_yaw) + vlp_py*m_sin_cos.get_CosValue(m_tf_param.vlp_yaw);
					// 위에서 curAngleH 에 vlp.yaw를 이미 반영 했음
					double vlp_yz = vlp_pz;
					double vlp_yx = vlp_px;
					double vlp_yy = vlp_py;

					// 구동장치에 장착된 센서의 롤 // 회전축 x 기준축 y [구동장치 기준]
					double sensor_mount_rx = vlp_yx;
					double sensor_mount_ry = vlp_yy*m_sin_cos.get_CosValue(m_tf_param.in_roll) - vlp_yz*m_sin_cos.get_SinValue(m_tf_param.in_roll);
					double sensor_mount_rz = vlp_yy*m_sin_cos.get_SinValue(m_tf_param.in_roll) + vlp_yz*m_sin_cos.get_CosValue(m_tf_param.in_roll);

					// 구동장치에 장착된 센서의 피치 // 회전축 y 기준축 z [구동장치 기준]
					double sensor_mount_py = sensor_mount_ry;
					double sensor_mount_pz = sensor_mount_rz*m_sin_cos.get_CosValue(m_tf_param.in_pitch) - sensor_mount_rx*m_sin_cos.get_SinValue(m_tf_param.in_pitch);
					double sensor_mount_px = sensor_mount_rz*m_sin_cos.get_SinValue(m_tf_param.in_pitch) + sensor_mount_rx*m_sin_cos.get_CosValue(m_tf_param.in_pitch);

					// 구동장치에 장착된 센서의 요 // 회전축 z 기준축 x [구동장치 기준]
					double sensor_mount_yz = sensor_mount_pz;
					double sensor_mount_yx = sensor_mount_px*m_sin_cos.get_CosValue(m_tf_param.in_yaw) - sensor_mount_py*m_sin_cos.get_SinValue(m_tf_param.in_yaw);
					double sensor_mount_yy = sensor_mount_px*m_sin_cos.get_SinValue(m_tf_param.in_yaw) + sensor_mount_py*m_sin_cos.get_CosValue(m_tf_param.in_yaw);

					// 구동장치의 회전구동 적용 // 회전축 z 기준축 x [구동장치 기준]
					double actuator_angle = m_yaw_angle + angleOffset_actuator;
					double bias_phase = m_tf_param.vlp_roll;
					double bias_angle = sin((actuator_angle + bias_phase) / 180. * DevLib::Mathematics::PI) * m_tf_param.in_z;
					actuator_angle = actuator_angle + bias_angle;
					if (actuator_angle >= 360.)
					{
						actuator_angle -= 360.;
					}
					else if (actuator_angle < 0.)
					{
						actuator_angle += 360.;
					}
					//fprintf(mp_analysis_file, "%.3lf, %.3lf\n", curAngleH, actuator_angle);
					double rotation_yz = sensor_mount_yz;
					double rotation_yx = sensor_mount_yx*m_sin_cos.get_CosValue(actuator_angle) - sensor_mount_yy*m_sin_cos.get_SinValue(actuator_angle);
					double rotation_yy = sensor_mount_yx*m_sin_cos.get_SinValue(actuator_angle) + sensor_mount_yy*m_sin_cos.get_CosValue(actuator_angle);

					// 센서 회전축 기울기에 의한 roll 보상(자세 추정)
					double sensor_slope_roll = sin((m_tf_param.in_x + actuator_angle) / 180.*DevLib::Mathematics::PI) * m_tf_param.in_y;
					double sensor_slope_rx = rotation_yx;
					double sensor_slope_ry = rotation_yy*m_sin_cos.get_CosValue(sensor_slope_roll) - rotation_yz*m_sin_cos.get_SinValue(sensor_slope_roll);
					double sensor_slope_rz = rotation_yy*m_sin_cos.get_SinValue(sensor_slope_roll) + rotation_yz*m_sin_cos.get_CosValue(sensor_slope_roll);

					// 센서 회전축 기울기에 의한 pitch 보상(자세 추정)
					double sensor_slope_pitch = -cos((m_tf_param.in_x + actuator_angle) / 180.*DevLib::Mathematics::PI) * m_tf_param.in_y;
					double sensor_slope_py = sensor_slope_ry;
					double sensor_slope_pz = sensor_slope_rz*m_sin_cos.get_CosValue(sensor_slope_pitch) - sensor_slope_rx*m_sin_cos.get_SinValue(sensor_slope_pitch);
					double sensor_slope_px = sensor_slope_rz*m_sin_cos.get_SinValue(sensor_slope_pitch) + sensor_slope_rx*m_sin_cos.get_CosValue(sensor_slope_pitch);

					// 센서 회전축 기울기 보상(위치 추정)
					double slope_3x = m_axis_delta * m_sin_cos.get_CosValue(90. - m_tf_param.in_y) * m_sin_cos.get_CosValue(m_tf_param.in_x + actuator_angle);
					double slope_3y = m_axis_delta * m_sin_cos.get_CosValue(90. - m_tf_param.in_y) * m_sin_cos.get_SinValue(m_tf_param.in_x + actuator_angle);
					double slope_3z = -m_axis_delta * m_sin_cos.get_SinValue(90. - m_tf_param.in_y);

					// 센서 회전축 기울기 보상 적용
					sensor_slope_pz += slope_3z;
					sensor_slope_px += slope_3x;
					sensor_slope_py += slope_3y;

					// 크레인에 장착된 구동장치 피치 // 회전축 y 기준축 x [크레인 기준]
					double rotor_mount_py = sensor_slope_py;
					double rotor_mount_pz = sensor_slope_pz*m_sin_cos.get_CosValue(m_tf_param.ex_pitch) - sensor_slope_px*m_sin_cos.get_SinValue(m_tf_param.ex_pitch);
					double rotor_mount_px = sensor_slope_pz*m_sin_cos.get_SinValue(m_tf_param.ex_pitch) + sensor_slope_px*m_sin_cos.get_CosValue(m_tf_param.ex_pitch);

					// 크레인에 장착된 구동장치 요 // 회전축 z 기준축 x [크레인 기준]
					double rotor_mount_yz = rotor_mount_pz;
					double rotor_mount_yx = rotor_mount_px*m_sin_cos.get_CosValue(m_tf_param.ex_yaw) - rotor_mount_py*m_sin_cos.get_SinValue(m_tf_param.ex_yaw);
					double rotor_mount_yy = rotor_mount_px*m_sin_cos.get_SinValue(m_tf_param.ex_yaw) + rotor_mount_py*m_sin_cos.get_CosValue(m_tf_param.ex_yaw);

					// 크레인에 장착된 구동장치 롤 // 회전축 x 기준축 y [크레인 기준]
					double rotor_mount_rx = rotor_mount_yx;
					double rotor_mount_ry = rotor_mount_yy*m_sin_cos.get_CosValue(m_tf_param.ex_roll) - rotor_mount_yz*m_sin_cos.get_SinValue(m_tf_param.ex_roll);
					double rotor_mount_rz = rotor_mount_yy*m_sin_cos.get_SinValue(m_tf_param.ex_roll) + rotor_mount_yz*m_sin_cos.get_CosValue(m_tf_param.ex_roll);

					// 측정점의 좌표를 월드모델의 그리드 위치 및 고도값으로 변환 , x1 = row , y1 = col , z1 = -elevation
					m_count_of_3d_data = m_count_of_3d_data > 99999 ? 99999 : m_count_of_3d_data;
					p_point_cloud->xyz[m_count_of_3d_data].X = rotor_mount_ry + m_tf_param.ex_y;  // 단위 : m 
					p_point_cloud->xyz[m_count_of_3d_data].Y = rotor_mount_rx + m_tf_param.ex_x;  // 단위 : m
					p_point_cloud->xyz[m_count_of_3d_data].Z = -(rotor_mount_rz + m_tf_param.ex_z);  // 단위 : m
					p_point_cloud->xyz[m_count_of_3d_data].W = 1;
					m_count_of_3d_data++;
				}
			}
		}
		memcpy(&m_pre_data_buf, &log_data, sizeof(struct_log_data));
	}
	else
	{
		memcpy(&m_pre_data_buf, &log_data, sizeof(struct_log_data));
		mb_is_first_time = false;
	}

	return m_count_of_3d_data;
}