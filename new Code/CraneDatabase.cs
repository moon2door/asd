using System.Collections.Generic;
using UnityEngine;

namespace PortSystem.Data
{
    [CreateAssetMenu(fileName = "CraneDatabase", menuName = "PortSystem/CraneDatabase")]
    public class CraneDatabase : ScriptableObject
    {
        [Header("Prefabs")]
        public GameObject defaultPrefab; // 혹시 모를 대비용
        public GameObject jibCranePrefab;
        public GameObject towerCranePrefab;
        public GameObject gantryCranePrefab;

        // "이 부두의 이 번호는 무슨 타입이다"라는 규칙 정의
        // 복잡하게 코딩하지 말고, 간단하게 함수로 구분
        public GameObject GetPrefab(int pierId, int craneId)
        {
            // 예: ENI 부두(9번) 처리
            if (pierId == (int)PierType.ENI)
            {
                // CraneInfo.h 참고: 0번은 TC1, 1~5번은 JIB1~5
                if (craneId == 0) return towerCranePrefab;
                if (craneId >= 1 && craneId <= 5) return jibCranePrefab;
            }

            // 다른 부두 규칙들도 여기에 추가하면 됩니다.
            // 일단은 기본값 리턴
            return defaultPrefab != null ? defaultPrefab : jibCranePrefab;
        }
    }
}