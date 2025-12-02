using System;
using System.Collections.Generic;
using UnityEngine;

namespace PortSystem.Data
{
    public class StFullPointCloud
    {
        public const int TYPE = 1;
        public const int CODE = 4;

        public List<Vector3> points = new List<Vector3>();
        public List<StDistanceData> risks = new List<StDistanceData>();

        public StFullPointCloud(byte[] buffer)
        {
            FromByte(buffer);
        }

        public void FromByte(byte[] buffer)
        {
            // 헤더 크기(921)보다 작으면 패스
            if (buffer.Length < 921) return;

            try
            {
                // --- [헤더 정보 확인] ---
                int posDistInfo = BitConverter.ToInt32(buffer, 880); // 충돌정보 데이터 시작 위치
                int sizeDistInfo = BitConverter.ToInt32(buffer, 884); // 충돌정보 전체 크기
                int countDistInfo = sizeDistInfo / 30; // 개수 (270 / 30 = 9)

                // [디버그] 패킷 정보 출력
                // Debug.Log($"[StFullPointCloud] 충돌 정보 크기: {sizeDistInfo} Byte (개수: {countDistInfo})");

                // 1. 포인트 클라우드 파싱 (기존 로직 유지)
                int posPoints = BitConverter.ToInt32(buffer, 848);
                int sizePoints = BitConverter.ToInt32(buffer, 852);
                int startPos = 921 + posPoints;
                int endPos = startPos + sizePoints;

                if (startPos >= 0 && endPos <= buffer.Length)
                {
                    for (int pos = startPos; pos < endPos; pos += 6)
                    {
                        float x = BitConverter.ToInt16(buffer, pos) * 0.01f;
                        float y = BitConverter.ToInt16(buffer, pos + 2) * 0.01f;
                        float z = BitConverter.ToInt16(buffer, pos + 4) * 0.01f;
                        points.Add(new Vector3(x, z, y));
                    }
                }

                // 2. 충돌 정보 파싱 (Risk Parsing)
                if (sizeDistInfo > 0)
                {
                    int startDist = 921 + posDistInfo; // 좌표 데이터 시작점
                    int posColInfo = BitConverter.ToInt32(buffer, 901); // 레벨 데이터 위치 정보인듯 함
                    int startCol = 921 + posColInfo; // 레벨 데이터 실제 시작점

                    for (int i = 0; i < countDistInfo; i++)
                    {
                        // (1) 위험 레벨 읽기
                        int level = 0;
                        if (startCol + i < buffer.Length)
                        {
                            level = buffer[startCol + i];
                        }

                        // ★★★ [수정됨] 범인 제거 ★★★
                        // 전임자가 안전한거(0)는 필요 없다고 판단해서 continue로 버렸기 때문에
                        // 서버가 9개를 보내도 다 0이면 리스트가 0개가 되었던 것임.
                        // 테스트를 위해 일단 주석 처리함 -> 이제 0이어도 리스트에 들어감
                        // if (level == 0) continue; 

                        // (2) 좌표 데이터 읽기 (30바이트 단위)
                        int p = startDist + (i * 30);
                        if (p + 30 > buffer.Length) break; // 버퍼 오버플로우 방지

                        StDistanceData data = new StDistanceData();
                        data.distance = BitConverter.ToSingle(buffer, p);

                        // 좌표 파싱
                        float cx = BitConverter.ToSingle(buffer, p + 6);
                        float cy = BitConverter.ToSingle(buffer, p + 10);
                        float cz = BitConverter.ToSingle(buffer, p + 14);

                        float mx = BitConverter.ToSingle(buffer, p + 18);
                        float my = BitConverter.ToSingle(buffer, p + 22);
                        float mz = BitConverter.ToSingle(buffer, p + 26);

                        data.pointCluster = new Vector3(cx, cz, cy);
                        data.pointCrane = new Vector3(mx, mz, my);
                        data.level = level;

                        // 리스트에 추가
                        risks.Add(data);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[FullPacket] 파싱 에러: {e.Message}");
            }
        }
    }
}