using System.Collections.Generic;
using UnityEngine;

public class PointManager : MonoBehaviour
{
    public static PointManager Instance;

    [Header("Settings")]
    public GameObject linePrefab; // LineController가 붙은 프리팹
    public Transform drawRoot;    // 그려진 것들이 모일 부모 폴더

    // 관리 장부 (ID -> LineController)
    private Dictionary<string, LineController> activeLines = new Dictionary<string, LineController>();
    // 재사용을 위한 풀 (Pool)
    private Queue<LineController> linePool = new Queue<LineController>();

    private void Awake()
    {
        Instance = this;
    }

    // 외부에서 부르는 함수: 이 ID로, 이 점들을, 이 색깔로 그려줘
    public void DrawPolygon(string id, List<Vector3> points, Color color, bool loop = true)
    {
        LineController line = GetLine(id);
        line.Draw(points, color, loop);
    }

    // 해당 ID의 그림 끄기
    public void ClearDraw(string id)
    {
        if (activeLines.ContainsKey(id))
        {
            LineController line = activeLines[id];
            line.Hide();
            activeLines.Remove(id);
            linePool.Enqueue(line); // 풀에 반납
        }
    }

    // 모든 그림 지우기
    public void ClearAll()
    {
        foreach (var kvp in activeLines)
        {
            kvp.Value.Hide();
            linePool.Enqueue(kvp.Value);
        }
        activeLines.Clear();
    }

    // 내부 함수: ID에 해당하는 라인 가져오기 (없으면 새로 만듦)
    private LineController GetLine(string id)
    {
        if (activeLines.ContainsKey(id))
        {
            return activeLines[id];
        }

        LineController newLine;
        if (linePool.Count > 0)
        {
            newLine = linePool.Dequeue();
        }
        else
        {
            // 풀이 비었으면 새로 생성
            GameObject go = Instantiate(linePrefab, drawRoot);
            newLine = go.GetComponent<LineController>();
            if (newLine == null) newLine = go.AddComponent<LineController>();
        }

        newLine.gameObject.name = $"Line_{id}";
        activeLines.Add(id, newLine);
        return newLine;
    }
}