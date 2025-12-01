using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class DrawLiner : MonoBehaviour
{
    public int maxDraw = 5;

    List<GameObject> drawObject = new List<GameObject>();

    void Awake()
    {
        int count = drawObject.Count;
        for (int i=0;i< maxDraw - count; i++)
        {
            GameObject draw = new GameObject("drawObject");
            LineRenderer lineRenderer=draw.AddComponent<LineRenderer>();
            lineRenderer.useWorldSpace = true;
            lineRenderer.loop = true; // 마지막 점과 첫 점 자동 연결
            draw.AddComponent<MeshFilter>();
            draw.AddComponent<MeshRenderer>();
            draw.transform.SetParent(transform);
            draw.SetActive(false);
            drawObject.Add(draw);
        }
    }
    public GameObject GetDrawObect()
    {
        for(int i = 0; i < drawObject.Count; i++)
        {
            if (drawObject[i].activeSelf == false)
                return drawObject[i];
        }
        return null;
    }
    
    public void ResetDraw()
    {
        for (int i = 0; i < drawObject.Count; i++)
        {
            drawObject[i].SetActive(false);
            drawObject[i].transform.localPosition = Vector3.zero;
        }
    }

    public void Draw(List<Vector3> points, Transform basePos, Color lineColor )
    {
        GameObject draw = GetDrawObect();
        if (draw == null)
            return;
        draw.SetActive(true);
        draw.transform.position = basePos.position;
        draw.transform.rotation = basePos.rotation;
        LineRenderer lineRenderer = draw.GetComponent<LineRenderer>();
        if (points.Count < 2) return;

        // 점들을 월드 좌표로 변환
        Vector3[] worldPoints = new Vector3[points.Count];
        for (int i = 0; i < points.Count; i++)
            worldPoints[i] = basePos.TransformPoint(points[i]);

        // --- 선 그리기 ---
        lineRenderer.positionCount = worldPoints.Length;
        lineRenderer.SetPositions(worldPoints);
        lineRenderer.startColor = lineColor;
        lineRenderer.endColor = lineColor;
        lineRenderer.widthMultiplier = 0.05f;

        //// --- 면 채우기 ---
        ///// 제대로 면이 안채워지고 있음.
        //if (points.Count >= 3)
        //{
        //    Mesh mesh = new Mesh();

        //    // 로컬 좌표로 폴리곤 생성 (2D triangulation)
        //    Vector3[] verts = new Vector3[points.Count];
        //    for (int i = 0; i < points.Count; i++)
        //        verts[i] = basePos.InverseTransformPoint(worldPoints[i]);

        //    int[] tris = Triangulate(verts);

        //    mesh.vertices = verts;
        //    mesh.triangles = tris;
        //    mesh.RecalculateNormals();

        //    MeshFilter meshFilter = draw.GetComponent<MeshFilter>();
        //    MeshRenderer meshRenderer = draw.GetComponent<MeshRenderer>();

        //    meshFilter.mesh = mesh;
        //    meshRenderer.material = new Material(Shader.Find("Standard"))
        //    {
        //        color = new Color(lineColor.r, lineColor.g, lineColor.b, 0.25f) // 선색 기반 반투명
        //    };
        //}
    }
    // 간단한 triangulation (Convex polygon 기준)
    private int[] Triangulate(Vector3[] verts)
    {
        List<int> indices = new List<int>();
        for (int i = 1; i < verts.Length - 1; i++)
        {
            indices.Add(0);
            indices.Add(i);
            indices.Add(i + 1);
        }
        return indices.ToArray();
    }
    /// <summary>
    /// min/max 좌표로 Bounding Box 생성
    /// </summary>
    public void DrawBoundingBox(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, Transform basePos, Color color)
    {
        List<Vector3> points = new List<Vector3>();

        // 8 꼭짓점 (로컬 좌표 기준)
        Vector3[] corners =
        {
            new Vector3(minX, minY, minZ),
            new Vector3(maxX, minY, minZ),
            new Vector3(maxX, minY, maxZ),
            new Vector3(minX, minY, maxZ),

            new Vector3(minX, maxY, minZ),
            new Vector3(maxX, maxY, minZ),
            new Vector3(maxX, maxY, maxZ),
            new Vector3(minX, maxY, maxZ)
        };

        // 선 연결용 (12 모서리 순서)
        int[] edges =
        {
            0,1, 1,2, 2,3, 3,0, // 아래 사각형
            4,5, 5,6, 6,7, 7,4, // 위 사각형
            0,4, 1,5, 2,6, 3,7  // 수직선
        };

        // 라인 그리기용 localPoints 채우기
        foreach (int idx in edges)
            points.Add(corners[idx]);

        Draw(points, basePos,color); // 기존 Draw() 호출
    }

}
