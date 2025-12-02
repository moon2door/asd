using System;
using System.Collections.Generic;
using UnityEngine;

namespace PortSystem.Data
{
    public class StPointCloud
    {
        public const int TYPE = 1;
        public const int CODE = 4;

        public int pierId;
        public int craneId;
        public List<Vector3> points = new List<Vector3>();

        public StPointCloud()
        {
        }

        public StPointCloud(byte[] buffer)
        {
            FromByte(buffer);
        }

        public void FromByte(byte[] buffer)
        {
            // [1] 파싱 시작 로그
            // Debug.Log($"[StPointCloud] 데이터 파싱 시작 (패킷 크기: {buffer.Length} 바이트)");

            if (buffer.Length < 921)
            {
                Debug.LogWarning("[StPointCloud] 패킷이 너무 작아서 파싱 실패");
                return;
            }

            try
            {
                this.pierId = BitConverter.ToInt32(buffer, 16);
                this.craneId = BitConverter.ToInt32(buffer, 20);

                int posPointsOffset = BitConverter.ToInt32(buffer, 848);
                int sizePointsBytes = BitConverter.ToInt32(buffer, 852);
                int startPos = 921 + posPointsOffset;

                // [2] 점 개수 및 루프 시작 확인
                int expectedPoints = sizePointsBytes / 6;
                // Debug.Log($"[StPointCloud] 파싱 정보 - Pier:{pierId}, Crane:{craneId}, 예상 점 개수:{expectedPoints}개");

                if (startPos + sizePointsBytes <= buffer.Length)
                {
                    for (int pos = startPos; pos < startPos + sizePointsBytes; pos += 6)
                    {
                        float x = BitConverter.ToInt16(buffer, pos) * 0.0005f;
                        float y = BitConverter.ToInt16(buffer, pos + 2) * 0.0005f;
                        float z = BitConverter.ToInt16(buffer, pos + 4) * 0.0005f;

                        points.Add(new Vector3(x, z, y));
                    }
                }

                // [3] 파싱 완료 로그
                Debug.Log($"<color=yellow>[StPointCloud] 파싱 완료! 변환된 점 개수: {points.Count}개</color>");
            }
            catch (Exception e)
            {
                Debug.LogError($"[StPointCloud] 파싱 중 에러 발생: {e.Message}");
            }
        }
    }
}