using System.Collections.Generic;
using UnityEngine;
using PortSystem.Data;

public class ServerRiskManager : MonoBehaviour
{
    public static ServerRiskManager Instance;

    [Header("Settings")]
    public GameObject labelPrefab; // 아까 만든 RiskLabel 프리팹
    public Transform labelRoot;

    private List<RiskLabel> pool = new List<RiskLabel>();

    private void Awake()
    {
        Instance = this;
    }

    private void Start()
    {
        if (NetworkManager.Instance != null)
        {
            NetworkManager.Instance.OnDistanceResultReceived += HandleDistanceResult;
            Debug.Log("[ServerRiskManager] 이벤트 구독 성공! 데이터 대기 중..."); // <--- 이 로그가 뜨는지 확인
        }
        else
        {
            Debug.LogError("[ServerRiskManager] NetworkManager를 찾을 수 없음!");
        }
    }

    private void OnDestroy()
    {
        if (NetworkManager.Instance != null)
        {
            NetworkManager.Instance.OnDistanceResultReceived -= HandleDistanceResult;
        }
    }

    // 서버가 보내준 정답지를 받아서 그리기만 함
    private void HandleDistanceResult(StDistanceResult result)
    {
        // [검문소 4] 매니저 도착 확인
        Debug.Log($"[검문소 4] 리스크 매니저 도착! 그릴 개수: {result.risks.Count}");

        // [추가] 좌표 확인용 로그
        if (result.risks.Count > 0)
        {
            var first = result.risks[0];
            Debug.Log($"[좌표 확인] 시작점: {first.pointCrane}, 끝점: {first.pointCluster}, 거리: {first.distance}m");
        }

        // 1. 그릴 개수 파악
        int count = result.risks.Count;

        // 2. 풀링 (개수 맞추기)
        while (pool.Count < count)
        {
            GameObject obj = Instantiate(labelPrefab, labelRoot);
            pool.Add(obj.GetComponent<RiskLabel>());
        }

        // 3. 데이터 입력
        for (int i = 0; i < count; i++)
        {
            var data = result.risks[i];

            // Show 함수 호출 (시작점, 끝점, 거리, 위험도)
            // level이 높을수록 위험 (0:안전 ~ 3:정지)
            pool[i].Show(data.pointCrane, data.pointCluster, data.distance, data.level);
        }

        // 4. 남는 거 끄기
        for (int i = count; i < pool.Count; i++)
        {
            pool[i].Hide();
        }
    }
}