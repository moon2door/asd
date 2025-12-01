using System;
using Microsoft.Win32.SafeHandles;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public struct SensorSingle
            {
               public short LidarFps;
               public float LidarTempTop;
               public float LidarTempBottom;
               public byte LidarNetworkStatus; // 0 : 비정상, 1 : 정상
               public byte LidarNetworkRangeData;  // 0 : 비정상, 1 : 정상
               public byte LidarReserved;
               public byte LidarErrorCode; // 0 : 정상, 1 : RPM 비정상

               public float ActuratorRpm;
               public float reserved;
               public short ActuratorFPS;
               public byte ActuratorPacket; // 0 : 정상, 1 : 비정상
               public byte ActuratorRotation; // 0 : 정지, 1 : 회전
               public byte ActuratorProxyMeter; // 근접센서 상태, 0 : 정상, 1 : 비정상
               public byte ActuratorZeroSet; // 0 : 정상, 1 : 비정상
               public byte ActuratorEncoder; // 0 : 정상, 1 : 비정상
               public byte ActuratorNetwork; // 0 : 정상, 1 : 비정상
               public byte ActuratorReserved;
            }

            public struct RotorParameterUnit
            {
                public float intrinsicsPrarmeter00;
                public float intrinsicsPrarmeter01;
                public float intrinsicsPrarmeter02;
                public float intrinsicsPrarmeter03;
                public float intrinsicsPrarmeter10;
                public float intrinsicsPrarmeter11;
                public float intrinsicsPrarmeter12;
                public float intrinsicsPrarmeter13;
                public float intrinsicsPrarmeter20;
                public float intrinsicsPrarmeter21;
                public float intrinsicsPrarmeter22;
                public float intrinsicsPrarmeter23;
                public float intrinsicsPrarmeter30;
                public float intrinsicsPrarmeter31;
                public float intrinsicsPrarmeter32;
                public float intrinsicsPrarmeter33;

                public float extrinsicsPrarmeter00;
                public float extrinsicsPrarmeter01;
                public float extrinsicsPrarmeter02;
                public float extrinsicsPrarmeter03;
                public float extrinsicsPrarmeter10;
                public float extrinsicsPrarmeter11;
                public float extrinsicsPrarmeter12;
                public float extrinsicsPrarmeter13;
                public float extrinsicsPrarmeter20;
                public float extrinsicsPrarmeter21;
                public float extrinsicsPrarmeter22;
                public float extrinsicsPrarmeter23;
                public float extrinsicsPrarmeter30;
                public float extrinsicsPrarmeter31;
                public float extrinsicsPrarmeter32;
                public float extrinsicsPrarmeter33;
            }

            public class RotorStatus
            {
                public static int type = 0x02;
                public static int code = 0x02;

                public uint numSensor = 0;
                public uint pier = 0;
                public uint crane = 0;
                public uint versionInterfacecRotor = 0; 
                public uint versionSensorManager = 0; //20

                public SensorSingle[] sensorParam = new SensorSingle[5]; // 31 * 5 = 155
                public RotorParameterUnit[] rotorParam = new RotorParameterUnit[5]; //128 * 5  = 640

                public RotorStatus(byte[] buffer)
                {
                    FromByte(buffer);
                }

                public void FromByte(byte[] buffer)
                {
                    if (buffer.Length >= 827)
                    {
                        numSensor = BitConverter.ToUInt32(buffer, 0);
                        pier = BitConverter.ToUInt32(buffer, 4);
                        crane = BitConverter.ToUInt32(buffer, 8);
                        versionInterfacecRotor = BitConverter.ToUInt32(buffer, 12);
                        versionSensorManager = BitConverter.ToUInt32(buffer, 16);

                        int offset = 0;
                        for (int i = 0; i < 5; i++)
                        {
                            sensorParam[i].LidarFps = 0;
                            sensorParam[i].LidarTempTop = 0;
                            sensorParam[i].LidarTempBottom = 0;
                            sensorParam[i].LidarNetworkStatus = 0; // 0 : 비정상, 1 : 정상
                            sensorParam[i].LidarNetworkRangeData = 0;  // 0 : 비정상, 1 : 정상
                            sensorParam[i].LidarReserved = 0;
                            sensorParam[i].LidarErrorCode = 0; // 0 : 정상, 1 : RPM 비정상
                            sensorParam[i].ActuratorRpm = 0;
                            sensorParam[i].reserved = 0;
                            sensorParam[i].ActuratorFPS = 0;
                            sensorParam[i].ActuratorPacket = 0; // 0 : 정상, 1 : 비정상
                            sensorParam[i].ActuratorRotation = 0; // 0 : 정지, 1 : 회전
                            sensorParam[i].ActuratorProxyMeter = 0; // 근접센서 상태, 0 : 정상, 1 : 비정상
                            sensorParam[i].ActuratorZeroSet = 0; // 0 : 정상, 1 : 비정상
                            sensorParam[i].ActuratorEncoder = 0; // 0 : 정상, 1 : 비정상
                            sensorParam[i].ActuratorNetwork = 0; // 0 : 정상, 1 : 비정상
                            sensorParam[i].ActuratorReserved = 0;


                            //rotorParam[i].intrinsicY = BitConverter.ToSingle(buffer, i * 60 + 4);
                           
                        }

                    }
                    //if (buffer.Length >= 60*5)
                    //{
                    //    for(int i=0; i<5; i++)
                    //    {
                    //        param[i].intrinsicX = BitConverter.ToSingle(buffer, i * 60 + 0);
                    //        param[i].intrinsicY = BitConverter.ToSingle(buffer, i * 60 + 4);
                    //        param[i].intrinsicZ = BitConverter.ToSingle(buffer, i * 60 + 8);
                    //        param[i].intrinsicRX = BitConverter.ToSingle(buffer, i * 60 + 12);
                    //        param[i].intrinsicRY = BitConverter.ToSingle(buffer, i * 60 + 16);
                    //        param[i].intrinsicRZ = BitConverter.ToSingle(buffer, i * 60 + 20);
                    //        param[i].extrinsicX = BitConverter.ToSingle(buffer, i * 60 + 24);
                    //        param[i].extrinsicY = BitConverter.ToSingle(buffer, i * 60 + 28);
                    //        param[i].extrinsicZ = BitConverter.ToSingle(buffer, i * 60 + 32);
                    //        param[i].extrinsicRX = BitConverter.ToSingle(buffer, i * 60 + 36);
                    //        param[i].extrinsicRY = BitConverter.ToSingle(buffer, i * 60 + 40);
                    //        param[i].extrinsicRZ = BitConverter.ToSingle(buffer, i * 60 + 44);
                    //        param[i].attitudeRX = BitConverter.ToSingle(buffer, i * 60 + 48);
                    //        param[i].attitudeRY = BitConverter.ToSingle(buffer, i * 60 + 52);
                    //        param[i].attitudeRZ = BitConverter.ToSingle(buffer, i * 60 + 56);
                    //    }
                    //}
                }
            }
        }
    }
}