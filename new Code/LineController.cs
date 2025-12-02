using System.Collections.Generic;
using UnityEngine;

public class LineController : MonoBehaviour
{
    private LineRenderer lr;

    private void Awake()
    {
        lr = GetComponent<LineRenderer>();
        if (lr == null) lr = gameObject.AddComponent<LineRenderer>();

        // 기본 스타일 설정
        lr.useWorldSpace = true; // 월드 좌표 사용
        lr.startWidth = 0.1f;
        lr.endWidth = 0.1f;
        lr.material = new Material(Shader.Find("Sprites/Default")); // 기본 쉐이더
    }

    public void Draw(List<Vector3> points, Color color, bool loop = true)
    {
        if (points == null || points.Count < 2)
        {
            lr.positionCount = 0;
            return;
        }

        lr.startColor = color;
        lr.endColor = color;
        lr.loop = loop; // 시작점과 끝점 연결 여부
        lr.positionCount = points.Count;
        lr.SetPositions(points.ToArray());

        gameObject.SetActive(true);
    }

    public void Hide()
    {
        gameObject.SetActive(false);
    }
}