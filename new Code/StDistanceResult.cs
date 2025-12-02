using System;
using System.Collections.Generic;
using UnityEngine;

namespace PortSystem.Data
{
    public class StDistanceData
    {
        public float distance;
        public int level;
        public Vector3 pointCluster;
        public Vector3 pointCrane;
    }

    public class StDistanceResult
    {
        public const int TYPE = 1;
        public const int CODE = 14;

        public List<StDistanceData> risks = new List<StDistanceData>();

        public StDistanceResult() { }

        public StDistanceResult(byte[] buffer)
        {
            if (buffer.Length < 921)
            {
                Debug.LogError($"[StDistanceResult] 패킷이 너무 작습니다. (Size: {buffer.Length})");
                return;
            }

            try
            {
                // 헤더 정보 읽기
                int posInfo = BitConverter.ToInt32(buffer, 880);
                int sizeInfo = BitConverter.ToInt32(buffer, 884);

                int startInfo = 921 + posInfo;
                int count = sizeInfo / 30; // Info 하나당 30바이트

                // [검문소 2] 파싱 정보 출력
                // Debug.Log($"[검문소 2] 데이터 분석 시작 -> 개수: {count}개 (Offset: {posInfo}, Size: {sizeInfo})");

                if (count == 0) return;

                // 데이터 파싱 루프
                for (int i = 0; i < count; i++)
                {
                    int p = startInfo + (i * 30);

                    // 범위 체크
                    if (p + 30 > buffer.Length) break;

                    StDistanceData data = new StDistanceData();
                    data.distance = BitConverter.ToSingle(buffer, p);

                    // 거리값 확인용 로그 (너무 많이 뜨면 주석 처리)
                    // if (i == 0) Debug.Log($"[데이터 샘플] 첫 번째 거리: {data.distance}m");

                    // 좌표 파싱
                    float cx = BitConverter.ToSingle(buffer, p + 6);
                    float cy = BitConverter.ToSingle(buffer, p + 10);
                    float cz = BitConverter.ToSingle(buffer, p + 14);

                    float mx = BitConverter.ToSingle(buffer, p + 18);
                    float my = BitConverter.ToSingle(buffer, p + 22);
                    float mz = BitConverter.ToSingle(buffer, p + 26);

                    // * 0.01f를 해야 하는지 확인 필요 (일단 원본 그대로 받음)
                    // 전임자 코드(StDistanceSocket)에는 Info 좌표에 배율 보정이 없었음.
                    data.pointCluster = new Vector3(cx, cz, cy);
                    data.pointCrane = new Vector3(mx, mz, my);

                    // 임시: Level은 일단 1로 고정 (나중에 CollisionInfo와 연결)
                    data.level = 1;

                    // 필터링: 유효한 거리만 추가
                    if (data.distance > 0 && data.distance < 200.0f) // 200m 이내만
                    {
                        risks.Add(data);
                    }
                }

                // Debug.Log($"[검문소 2-완료] 파싱 끝! 유효한 리스크 개수: {risks.Count}개");
            }
            catch (Exception e)
            {
                Debug.LogError($"[StDistanceResult] 파싱 에러: {e.Message}");
            }
        }
    }
}