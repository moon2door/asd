using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GPS2Map : MonoBehaviour
{
    static readonly float MPerDegLat = 111320.0f;
    static readonly Vector2 startGps =new Vector2(34.906412f, 128.589578f) ;
    static float ratioDistance = 1.4f;
    static readonly float ratioRotate = -179f;
    static Vector3 startPosition =Vector3.zero;
    public Transform startMap;

    public GameObject testGPS;
    public Vector2 inputGPS;

    private void Awake()
    {
        startPosition = startMap.position;
    }

    // 기준 GPS와 기준 Unity 좌표를 연결해서 GPS를 Unity 좌표로 변환
    public static Vector3 GpsToUnity(Vector2 gps)
    {
        // GPS 기준점 (실제 위치)
        Vector2 gpsOrigin = startGps;

        // Unity 맵에서 대응되는 기준점
        Vector3 unityOrigin = startPosition;
        float mPerDegLon = MPerDegLat * Mathf.Cos(gpsOrigin.x * Mathf.Deg2Rad);

        // 위도/경도 차이 → 미터 단위
        float dLat = (gps.x - gpsOrigin.x) * MPerDegLat;   // 북(+)
        float dLon = (gps.y - gpsOrigin.y) * mPerDegLon;   // 동(+)

        // 스케일 보정 (맵이 1.5배 늘어나 있음 → 0.6667 곱하기)
        dLat *= ratioDistance;
        dLon *= ratioDistance;

        float rad = ratioRotate * Mathf.Deg2Rad;
        float cos = Mathf.Cos(rad);
        float sin = Mathf.Sin(rad);

        float x = cos * dLon - sin * dLat;
        float z = sin * dLon + cos * dLat;

        // 180° 뒤집힘 보정 (X축 반전)

        // Unity 원점 적용
        return unityOrigin + new Vector3(x, 0f, z);
    }
    void Start()
    {
    }
    private void Update()
    {
        if(testGPS != null && testGPS.activeSelf == true)
        {
            testGPS.transform.position = GpsToUnity(inputGPS);
        }
    }
}
