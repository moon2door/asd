using System.Collections.Generic;

namespace PortSystem.Data
{
    // C++의 enum PIER 대응
    public enum PierType
    {
        P7 = 0, J, K, HAN, P6, G2, G3, G4, Z, ENI
    }

    public static class PortCode
    {
        // 부두 ID -> 이름 변환
        public static string GetPierName(int pierId)
        {
            switch ((PierType)pierId)
            {
                case PierType.ENI: return "ENI";
                case PierType.P7: return "7";
                case PierType.J: return "J";
                case PierType.K: return "K";
                case PierType.HAN: return "HAN";
                default: return "Unknown";
            }
        }

        // 크레인 ID -> 이름 변환 (ENI 부두 예시)
        public static string GetCraneName(int pierId, int craneId)
        {
            if (pierId == (int)PierType.ENI)
            {
                // CraneInfo.h의 PierENI namespace 참고
                string[] names = { "TC1", "JIB1", "JIB2", "JIB3", "JIB4", "JIB5" };
                if (craneId >= 0 && craneId < names.Length) return names[craneId];
            }

            // 다른 부두는 필요할 때 추가
            return $"C_{pierId}_{craneId}";
        }
    }
}