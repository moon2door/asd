using System;

namespace CsCore
{
    namespace Data
    {
        namespace CollisionProcessor
        {
            public class StCraneAttitude
            {
                // 크레인 정보
                public int pierId;
                public int craneId;
                public float groundHeight;
                public int numPart;
                public int numHook;

                // 크레인 자세 값
                public float[] pose = new float[5];
                public bool[] bUseEstimate = new bool[5];
                public bool[] bEstimateSucceed = new bool[5];
                public bool[] bMoving = new bool[5];

                // 조인트 정보
                public float[] jointInfo = new float[45];

                // 후크 정보
                public float[] hookPoint = new float[9];
                public float[] hookWeight = new float[3];
                public float[] hookWightThreshold = new float[3];
                public float[] hookRoi = new float[18]; // 6 * 3
                public float[] hookBoundRoi = new float[18]; // 6 * 3

                // GPS 관련 정보
                public float gpsTimestamp;
                public float gpsLatitude;
                public float gpsLongitude;
                public float hdop;
                public float height;
                public float azimuth;
                public int quality;
                public int numOfStatellites;
                public double gpsLatitude2;
                public double gpsLongitude2;

                // 인양물 정보
                public bool[] salvageStatus = new bool[5];
                public int numSalvageRoi;
                public float[] salvageRoi = new float[30]; // 6 * 3

                // 크레인 ROI
                public float[] craneRoi = new float[30]; // 6 * 5

                public StCraneAttitude()
                {
                }
                public StCraneAttitude(byte[] buffer)
                {
                    FromByte(buffer);
                }
                public int FromByte(byte[] buffer)
                {
                    int ret = 0;
                    if (buffer.Length >= 824)
                    {
                        int pos = 0;
                        // 크레인 정보
                        pierId = BitConverter.ToInt32(buffer, pos); pos += 4;
                        craneId = BitConverter.ToInt32(buffer, pos); pos += 4;
                        groundHeight = BitConverter.ToSingle(buffer, pos); pos += 4;
                        numPart = BitConverter.ToInt32(buffer, pos); pos += 4;
                        numHook = BitConverter.ToInt32(buffer, pos); pos += 4;

                        // 크레인 자세 값
                        for (int i = 0; i < 5; i++)
                        {
                            pose[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }
                        for (int i = 0; i < 5; i++)
                        {
                            bUseEstimate[i] = (buffer[pos] == 1);
                            pos++;
                        }
                        for (int i = 0; i < 5; i++)
                        {
                            bEstimateSucceed[i] = (buffer[pos] == 1);
                            pos++;
                        }
                        for (int i = 0; i < 5; i++)
                        {
                            bMoving[i] = (buffer[pos] == 1);
                            pos++;
                        }

                        // 조인트 정보
                        for (int i = 0; i < 45; i++)
                        {
                            jointInfo[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }

                        // 후크 정보
                        for (int i = 0; i < 9; i++)
                        {
                            hookPoint[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }
                        pos += 6 * 4; // reserved
                        for (int i = 0; i < 3; i++)
                        {
                            hookWeight[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }
                        pos += 2 * 4; // reserved
                        for (int i = 0; i < 3; i++)
                        {
                            hookWightThreshold[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }
                        pos += 2 * 4; // reserved
                        for (int i = 0; i < 18; i++)
                        {
                            hookRoi[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }
                        pos += 12 * 4; // reserved
                        for (int i = 0; i < 18; i++)
                        {
                            hookBoundRoi[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }

                        // GPS 관련 정보
                        gpsTimestamp = BitConverter.ToSingle(buffer, pos); pos += 4;
                        gpsLatitude = BitConverter.ToSingle(buffer, pos); pos += 4;
                        gpsLongitude = BitConverter.ToSingle(buffer, pos); pos += 4;
                        hdop = BitConverter.ToSingle(buffer, pos); pos += 4;
                        height = BitConverter.ToSingle(buffer, pos); pos += 4;
                        azimuth = BitConverter.ToSingle(buffer, pos); pos += 4;
                        quality = BitConverter.ToInt32(buffer, pos); pos += 4;
                        numOfStatellites = BitConverter.ToInt32(buffer, pos); pos += 4;
                        gpsLatitude2 = BitConverter.ToDouble(buffer, pos); pos += 8;
                        gpsLongitude2 = BitConverter.ToDouble(buffer, pos); pos += 8;

                        // 인양물 정보
                        for (int i = 0; i < 5; i++)
                        {
                            salvageStatus[i] = (buffer[pos] == 1);
                            pos++;
                        }
                        int numSalvageRoi = BitConverter.ToInt32(buffer, pos); pos += 4;
                        for (int i = 0; i < 30; i++)
                        {
                            salvageRoi[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }

                        // 크레인 ROI
                        for (int i = 0; i < 30; i++)
                        {
                            craneRoi[i] = BitConverter.ToSingle(buffer, pos);
                            pos += 4;
                        }
                        ret = pos;
                    }
                    return ret;
                }
            }
        }
    }
}