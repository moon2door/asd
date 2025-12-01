using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class CraneUtility
{
    public enum PierType
    {
        PIER_7, 
        PIER_J, 
        PIER_K, 
        PIER_HAN,
        PIER_6, 
        PIER_G2,
        PIER_G3,
        PIER_G4,
        PIER_Z,
        PIER_Y
    }
    static string[] pierString= 
    {
        "7",
        "J",
        "K",
        "HAN",
        "6",
        "G2",
        "G3",
        "G4",
        "Z",
        "Y"
    };
    // GPS Offset 저장
    readonly static IReadOnlyList<GPSOffset[]> gpsOffsets = new List<GPSOffset[]>()
    {
 
        // J 0
        new GPSOffset[]
        {
            new GPSOffset(0f, 0f),        // LLC24
            new GPSOffset(3f, -3f),       // LLC11
            new GPSOffset(0.96f, -2.98f), // LLC8
            new GPSOffset(-4.1f, -7.1f),  // LLC9
        },

        // K 1
        new GPSOffset[]
        {
            new GPSOffset(5.899999f, -3.6f), // JIB1
            new GPSOffset(-4.6f, -8.9f),     // JIB2
            new GPSOffset(-1f, -6.9f),       // JIB3
            new GPSOffset(5.899999f, -3.6f), // LLC18
            new GPSOffset(-4.6f, -8.9f),     // LLC19
        },

        // HAN 2
        new GPSOffset[]
        {
            new GPSOffset(-6.2f, -1.4f),   // GC1
            new GPSOffset(6.8f, 1.6f),     // GC2
            new GPSOffset(19.1f, 8.08f),   // TC1
            new GPSOffset(20.07f, 5.01f),  // TC2
            new GPSOffset(-20.1f, -19.1f), // TTC4
            new GPSOffset(9.6f, 18.4f),    // TC5
        },

        // 6 3
        new GPSOffset[]
        {
            new GPSOffset(-0.93f, -4.069999f), // LLC7
            new GPSOffset(-16.1f, -12.1f),     // LLC23
        },

        // G2 4
        new GPSOffset[]
        {
            new GPSOffset(1.92f, -7.009998f), // LLC12
            new GPSOffset(0.9f, -0.1f),       // LLC13
        },

        // G3 5
        new GPSOffset[]
        {
            new GPSOffset(2.92f, -5.039999f), // LLC19
        },

        // G4 6
        new GPSOffset[]
        {
            new GPSOffset(7.812122f,4.816815f,-33,0),        // LLC25
            new GPSOffset(6.101968f,4.678275f), // LLC26
        },

        // Z 7
        new GPSOffset[]
        {
            new GPSOffset(0f, 0f), // LLC16
        },

        // ENI 8
        new GPSOffset[]
        {
            new GPSOffset(-19.98089f, 14.04872f,-15,0),    // TC1
            new GPSOffset(0f, 0f),    // JIB1
            new GPSOffset(-9.1f, -12.1f), // JIB2
            new GPSOffset(0f, 0f),    // JIB3
            new GPSOffset(0f, 0f),    // JIB4
            new GPSOffset(0f, 0f),    // JIB5
        },
    };
    public static string GetPierString(int pier)
    {
        if (pierString.Length > pier)
            return pierString[pier];
        else return "6";

    }
    public static int GetPierString(string pier)
    {
        for(int i=0;i< pierString.Length; i++)
        {
            if (pierString[i] == pier)
                return i;
        }
        return 0;
    }
    public static string GetPierString(PierType pier)
    {
        int iPier = (int)pier;
        return GetPierString(iPier);
    }
    public static PierType GetPierType(string pier)
    {
        for (int i = 0; i < pierString.Length; i++)
        {
            if (pierString[i] == pier)
                return (PierType)i;
        }
        return PierType.PIER_6;
    }

    public static string GetCraneString(int pier, int crane)
    {
        string strCrane = "LLC7";
        if (pier == 4)
        {
            switch (crane)
            {
                case 0:
                    strCrane = "LLC7";
                    break;
                case 1:
                    strCrane = "LLC23";
                    break;
                default:
                    break;
            }
        }
        else if (pier == 5)
        {
            switch (crane)
            {
                case 0:
                    strCrane = "LLC12";
                    break;
                case 1:
                    strCrane = "LLC13";
                    break;
                default:
                    break;
            }
        }
        else if (pier == 6)
        {
            switch (crane)
            {
                case 0:
                    strCrane = "LLC19";
                    break;
                case 1:
                    strCrane = "LLC20";
                    break;
                default:
                    break;
            }
        }
        else if (pier == 7)
        {
            switch (crane)
            {
                case 0:
                    strCrane = "LLC25";
                    break;
                case 1:
                    strCrane = "LLC26";
                    break;
                default:
                    break;
            }
        }
        //pjh
        else if(pier == 8)
        {
            switch(crane)
            {
                case 0:
                strCrane = "LLC16";
                break;
            }
        }
        //~pjh

        //Y
        else if (pier == 9)
        {
            switch (crane)
            {
                case 0:
                    strCrane = "TC1";
                    break;
                case 1:
                    strCrane = "JIB1";
                    break;
                case 2:
                    strCrane = "JIB2";
                    break;
                case 3:
                    strCrane = "JIB3";
                    break;
            }
        }
        //~Y
        else
        {
            strCrane = "Unknown";
        }


        return strCrane;
    }

    //CYS
    // 선상 크레인에 의한 움직이지 못하는 경우의 크레인 추가.
    public static bool isAbleMovePier(string pier)
    {
        if(pier == GetPierString((int)PierType.PIER_Y))
        {
            return false;
        }

        return true;
    }

    //XZ 평면으로 이동
    public static Vector3 CraneGPSOffset(int pier, int craneId)
    {
        int pierId = pier-1;
        if (gpsOffsets.Count > pierId && pierId >= 0)
            if(gpsOffsets[pierId].Length > craneId && craneId >= 0)
                return new Vector3(gpsOffsets[pierId][craneId].gpsX, 0, gpsOffsets[pierId][craneId].gpsY);
        return Vector3.zero;
    }

    //XZ 평면으로 이동
    public static Vector3 CranePierOffset(int pier, int craneId)
    {
        int pierId = pier - 1;
        if (gpsOffsets.Count > pierId && pierId >= 0)
            if (gpsOffsets[pierId].Length > craneId && craneId >= 0)
                return new Vector3(gpsOffsets[pierId][craneId].pierX, 0, gpsOffsets[pierId][craneId].pierY);
        return Vector3.zero;
    }
}
class GPSOffset
{
    public float gpsX = 0;
    public float gpsY = 0;
    public float pierX = 0;
    public float pierY = 0;
    public GPSOffset(float x, float y, float pX=0 , float pY=0)
    {
        this.gpsX = x;
        this.gpsY = y;
        this.pierX = pX;
        this.pierY = pY;
    }
}