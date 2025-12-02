using UnityEngine;
using TMPro; // TextMeshPro가 없다면 UnityEngine.UI.Text로 대체 가능

public class RiskLabel : MonoBehaviour
{
    public LineRenderer line;
    public TextMeshPro textMesh; // 3D TextMeshPro 사용

    // 초기화: 컴포넌트 자동 찾기
    private void Awake()
    {
        if (line == null) line = GetComponentInChildren<LineRenderer>();
        if (textMesh == null) textMesh = GetComponentInChildren<TextMeshPro>();
    }

    public void Show(Vector3 startPos, Vector3 targetPos, float distance, int rank)
    {
        gameObject.SetActive(true);

        // 1. 선 그리기
        if (line != null)
        {
            line.positionCount = 2;
            line.SetPosition(0, startPos);
            line.SetPosition(1, targetPos);

            // 색상 (1등 빨강, 나머지 노랑)
            Color color = (rank == 0) ? Color.red : Color.yellow;
            line.startColor = color;
            line.endColor = color;
        }

        // 2. 거리 텍스트 표시
        if (textMesh != null)
        {
            textMesh.text = $"{distance:0.0}m";
            textMesh.transform.position = targetPos + Vector3.up * 0.5f;
            textMesh.transform.rotation = Camera.main.transform.rotation; // 빌보드
            textMesh.color = (rank == 0) ? Color.red : Color.yellow;
        }
    }

    public void Hide()
    {
        gameObject.SetActive(false);
    }
}