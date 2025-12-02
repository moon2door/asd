using System.Collections.Generic;
using UnityEngine;
using PortSystem.Data;
using System.Linq; // 리스트 합치기용

public class PointCloudManager : MonoBehaviour
{
    public static PointCloudManager Instance;

    [Header("Renderer")]
    public PointCloudRenderer rendererPrefab;
    private PointCloudRenderer currentRenderer;

    [Header("Settings")]
    [Range(1, 50)]
    public int accumulateFrames = 20; // ★비밀의 열쇠: 몇 프레임을 쌓을 것인가? (기본 20장)

    // 과거의 점들을 기억할 대기열
    private Queue<Vector3[]> pointHistory = new Queue<Vector3[]>();

    private void Awake()
    {
        Instance = this;
    }

    private void Start()
    {
        if (rendererPrefab != null)
        {
            currentRenderer = Instantiate(rendererPrefab, transform);
            currentRenderer.transform.localPosition = Vector3.zero;
        }

        if (NetworkManager.Instance != null)
        {
            NetworkManager.Instance.OnPointCloudReceived += HandlePointCloud;
        }
    }

    private void OnDestroy()
    {
        if (NetworkManager.Instance != null)
        {
            NetworkManager.Instance.OnPointCloudReceived -= HandlePointCloud;
        }
    }

    private void HandlePointCloud(StPointCloud data)
    {
        if (currentRenderer != null && data.points.Count > 0)
        {
            // 1. 새로운 점 데이터를 역사책(Queue)에 기록
            pointHistory.Enqueue(data.points.ToArray());

            // 2. 너무 오래된 데이터는 폐기 (accumulateFrames 개수 유지)
            while (pointHistory.Count > accumulateFrames)
            {
                pointHistory.Dequeue();
            }

            // 3. 역사책에 있는 모든 점을 한 방에 합치기! (1,500개 * 20장 = 30,000개)
            // 퍼포먼스를 위해 List<Vector3>로 평탄화
            List<Vector3> combinedPoints = new List<Vector3>();
            foreach (var frame in pointHistory)
            {
                combinedPoints.AddRange(frame);
            }

            // 4. 합쳐진 3만 개의 점을 그림
            currentRenderer.DrawPoints(combinedPoints.ToArray());

            // 디버그: 실제로 몇 개나 그려지는지 확인
            // Debug.Log($"[Accumulation] 프레임 {pointHistory.Count}장 누적됨 -> 총 {combinedPoints.Count}개 그리는 중");
        }
    }
}