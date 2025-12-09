using System;
using System.Runtime.InteropServices;

//pjh
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct StCraneAttitude
{
    // 크레인 정보
    public int pierId;
    public int craneId;
    public float groundHeight;
    public int numPart;
    public int numHook;

    // 크레인 자세 값
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =5)]
    public float[] pose;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =5)]
    public bool[] bUseEstimate;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =5)]
    public bool[] bEstimateSucceed;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =5)]
    public bool[] bMoving;

    // 조인트 정보
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =45)]
    public float[] jointInfo;

    // 후크 정보
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =9)]
    public float[] hookPoint;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =6)]
    float[] reserved1;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst =3)]
    public float[] hookWeight;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =2)]
    float[] reserved2;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst =3)]
    public float[] hookWightThreshold;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =2)]
    float[] reserved3;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst =18)]
    public float[] hookRoi; // 6 * 3
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =12)]
    float[] reserved4 ;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst =18)]
    public float[] hookBoundRoi; // 6 * 3

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
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =5)]
    public bool[] salvageStatus;
    public int numSalvageRoi;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =30)]
    public float[] salvageRoi; // 6 * 3

    // 크레인 ROI
    [MarshalAs(UnmanagedType.ByValArray, SizeConst =30)]
    public float[] craneRoi; // 6 * 5
    
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
//~pjh