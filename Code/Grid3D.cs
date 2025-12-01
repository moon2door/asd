using UnityEngine;

[ExecuteAlways]
public class Grid3D : MonoBehaviour
{
    public float size = 20f;
    public float step = 1f;
    public Color gridColor = new Color(0.75f, 0.75f, 0.75f, 0.6f);
    public float gridNearOffset = 2f;

    static Material _lineMat;

    public void OnGrid()
    {
        EnsureMaterial();
        Camera.main.clearFlags = CameraClearFlags.SolidColor;
        Camera.main.backgroundColor = Color.white;
        gameObject.SetActive(true);
    }
    public void OffGrid()
    {
        Camera.main.clearFlags = CameraClearFlags.Skybox;
        Camera.main.backgroundColor = Color.white;
        gameObject.SetActive(false);
    }

    static void EnsureMaterial()
    {
        if (_lineMat != null) return;
        var shader = Shader.Find("Hidden/Internal-Colored");
        _lineMat = new Material(shader)
        {
            hideFlags = HideFlags.HideAndDontSave
        };
        _lineMat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        _lineMat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        _lineMat.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
        _lineMat.SetInt("_ZWrite", 0);
    }

    void OnRenderObject()
    {
        if (_lineMat == null) return;
        _lineMat.SetPass(0);
        DrawGridXZ();
    }

    void DrawGridXZ()
    {
        float half = Mathf.Max(step, size);
        int lines = Mathf.FloorToInt(half / step);

        Vector3 camPos = Camera.main != null ? Camera.main.transform.position : Vector3.zero;

        GL.PushMatrix();
        GL.MultMatrix(transform.localToWorldMatrix); // 오브젝트 좌표 기준으로 변환

        GL.Begin(GL.LINES);
        GL.Color(gridColor);

        for (int i = -lines; i <= lines; i++)
        {
            float x = i * step;
            Vector3 p1 = new Vector3(x, 0f, -half);
            Vector3 p2 = new Vector3(x, 0f, half);

            if (Vector3.Distance(camPos, transform.TransformPoint(p1)) > gridNearOffset &&
                Vector3.Distance(camPos, transform.TransformPoint(p2)) > gridNearOffset)
            {
                GL.Vertex(p1);
                GL.Vertex(p2);
            }

            float z = i * step;
            Vector3 p3 = new Vector3(-half, 0f, z);
            Vector3 p4 = new Vector3(half, 0f, z);

            if (Vector3.Distance(camPos, transform.TransformPoint(p3)) > gridNearOffset &&
                Vector3.Distance(camPos, transform.TransformPoint(p4)) > gridNearOffset)
            {
                GL.Vertex(p3);
                GL.Vertex(p4);
            }
        }

        GL.End();
        GL.PopMatrix();
    }
    private void LateUpdate()
    {
        if(ProcessManager.instance != null && ProcessManager.instance.craneManager != null)
        {
            CraneObject crane = ProcessManager.instance.craneManager.CurCrane();
            if(crane != null)
            {
                if (crane.pierReferancePosition != null)
                    transform.position = crane.pierReferancePosition.gameObject.transform.position;
                else
                    transform.position = crane.gameObject.transform.position;
            }
        }
    }
}
