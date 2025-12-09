using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PierUtility 
{
    public enum PierType
    {
        Pier_7  ,
        Pier_J  ,
        Pier_K  ,
        Pier_HAN,
        Pier_6  ,
        Pier_G2 ,
        Pier_G3 ,
        Pier_G4 ,
        Pier_0D ,
        Pier_ENI,
        MAX
    }
    public static string[] PierStr = new string[] { "7", "J", "K", "HAN", "6", "G2", "G3", "G4", "0D", "ENI" };
    public static int PierNum(PierType pier)
    {
        return (int)pier;
    }
    public static int PierNum(string pier)
    {
        PierType pierType = Str2Pier(pier);
        return PierNum(pierType);
    }
    public static string PierName(PierType pier)
    {
        return PierStr[(int)pier];
    }
    public static string PierName(int pier)
    {
        PierType pierType = Num2Pier(pier);
        return PierName(pierType);
    }
    public static PierType Str2Pier(string pier)
    {
        for(int i=0;i< PierStr.Length; i++)
        {
            if (PierStr[i] == pier)
                return (PierType)i;
        }
        return PierType.Pier_J;
    }
    public static PierType Num2Pier(int pier)
    {
        if (pier < 0)
            return PierType.Pier_7;
        else if (pier >= (int)PierType.MAX)
            return PierType.Pier_0D;
        return (PierType)(pier);
    }
}
