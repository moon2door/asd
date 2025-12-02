using System.Collections.Generic;
using UnityEngine;
using PortSystem.Data; 

public class CraneManager : MonoBehaviour
{
    public static CraneManager Instance;

    [Header("Prefabs")]
    public CraneDatabase craneDB;
    public Transform worldRoot;       // 크레인들이 생성될 부모 오브젝트

    // 생성된 크레인들을 관리하는 장부 (Key: "9_5" 같은 문자열 ID)
    private Dictionary<string, CraneController> activeCranes = new Dictionary<string, CraneController>();

    private void Awake()
    {
        Instance = this;
    }

    private void Start()
    {
        // NetworkManager가 데이터를 받으면 나한테도 알려달라고 등록
        if (NetworkManager.Instance != null)
        {
            NetworkManager.Instance.OnCraneDataReceived += HandleCraneData;
        }
    }

    private void OnDestroy()
    {
        if (NetworkManager.Instance != null)
        {
            NetworkManager.Instance.OnCraneDataReceived -= HandleCraneData;
        }
    }

    // 데이터가 도착했을 때 실행되는 함수
    private void HandleCraneData(StCraneAttitude data)
    {
        // 1. 고유 ID 생성 (예: "9_5") -> ENI부두(9)의 JIB5(5)
        string key = $"{data.pierId}_{data.craneId}";

        // 2. 이미 있는 크레인인가?
        if (activeCranes.ContainsKey(key))
        {
            // 있으면 움직임 업데이트
            activeCranes[key].UpdateMovement(data);
        }
        else
        {
            // 없으면 새로 생성 (스폰)
            SpawnCrane(key, data);
        }
    }

    private void SpawnCrane(string key, StCraneAttitude data)
    {
        if (craneDB == null)
        {
            Debug.LogError("CraneDB가 연결되지 않았습니다!");
            return;
        }

        // DB에서 알맞은 프리팹 가져오기
        GameObject prefabToUse = craneDB.GetPrefab(data.pierId, data.craneId);

        if (prefabToUse == null) return;

        GameObject newObj = Instantiate(prefabToUse, worldRoot);
        newObj.name = $"Crane_{PortCode.GetPierName(data.pierId)}_{PortCode.GetCraneName(data.pierId, data.craneId)}";

        // 생성 위치 초기화 (데이터에 GPS 값이 있다면 변환해서 넣어야겠지만, 일단 0,0,0)
        newObj.transform.localPosition = Vector3.zero;

        CraneController controller = newObj.GetComponent<CraneController>();
        if (controller == null) controller = newObj.AddComponent<CraneController>();

        controller.Initialize(data.pierId, data.craneId);

        activeCranes.Add(key, controller);
        Debug.Log($"[Manager] 크레인 생성 완료: {newObj.name} (타입: {prefabToUse.name})");

        // [테스트] 크레인 바닥에 빨간색 위험 구역 그리기
        if (PointManager.Instance != null)
        {
            List<Vector3> zonePoints = new List<Vector3>();
            // 크레인 위치 기준으로 10m 정사각형
            Vector3 center = newObj.transform.position;
            zonePoints.Add(center + new Vector3(-10, 0, -10));
            zonePoints.Add(center + new Vector3(10, 0, -10));
            zonePoints.Add(center + new Vector3(10, 0, 10));
            zonePoints.Add(center + new Vector3(-10, 0, 10));

            // "CraneZone_9_5" 같은 ID로 그림 요청
            string drawId = $"CraneZone_{data.pierId}_{data.craneId}";
            PointManager.Instance.DrawPolygon(drawId, zonePoints, Color.red);
        }
    }
}