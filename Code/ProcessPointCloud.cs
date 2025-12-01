using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System;

public enum Colormode
{
    None, soliod, hight
}
public class PointCloudRenderer
{
    public PointCloudRenderer(int pier, int crane, string name, GameObject pointOrigin)
    {
        this.pointOrigin = pointOrigin;
        this.pier = pier;
        this.crane = crane;
        pointPoolGroup = new GameObject("pointPool");
        pointPoolGroup.transform.SetParent(pointOrigin.transform);
        pointPoolGroup.transform.localPosition = Vector3.zero;
        pointPoolGroup.transform.localRotation = Quaternion.identity;

        bUpdate = false;
    }

    public void UpdatePoints(Vector3[] points, Color[] colors, int[] indices)
    {
        this.points = points;
        this.colors = colors;
        this.indices = indices;
        bUpdate = true;
    }

    public void ResetUpdatd()
    {
        bUpdate = false;
    }

    public bool IsUpdated()
    {
        return bUpdate;
    }

    public int pier;
    public int crane;

    // PointCloud
    bool bUpdate;
    public uint limitPoints = 650000;
    public Vector3[] points;
    public Color[] colors;
    public int[] indices;

    public GameObject pointOrigin;
    public GameObject pointPoolGroup;
    public Queue<GameObject> pointGroups = new Queue<GameObject>();
    public Queue<GameObject> pointGroupsPool = new Queue<GameObject>();
}
public class ProcessPointCloud : MonoBehaviour
{
    //public List<CraneObject> cranes;//pjh
    public Material material;
    public int queueSize = 10;
    public int poolSize = 100;
    private List<PointCloudRenderer> cloudRenderer = new List<PointCloudRenderer>();

    [Range(0.1f, 10.0f)]
    public float pointSize = 2;

    public Color pointColor;

    private Colormode colormode = Colormode.hight;
    private float minHeight = -100.0f;
    private float maxHeight = 100.0f;

    private GameObject pointFrame;

    private ColorTable.ColorTables currentColorTable = global::ColorTable.ColorTables.TableViridis;

    readonly WaitForSecondsRealtime wait = new WaitForSecondsRealtime(1);//pjh

    public void SetColorMode(Colormode mode)
    {
        colormode = mode;
    }

    public void SetColorTable(ColorTable.ColorTables table)
    {
        currentColorTable = table;
    }

    public void SetHeightRange(float minHeight, float maxHeight)
    {
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
    }

    private void Awake()
    {
    }

    //pjh
    private void Start()
    {
        if (ProcessManager.instance == null && ProcessManager.instance.craneManager == null)
            return;
        //pjh
        List<CraneObject> cranes = ProcessManager.instance.craneManager.GetCranes();
        foreach (CraneObject crane in cranes)
        {
            cloudRenderer.Add(new PointCloudRenderer(crane.pierNum, crane.craneNum, crane.craneName, crane.pointOrigin));
        }
        //~pjh

        for (int i = 0; i < cloudRenderer.Count; i++)
        {
            for (int j = 0; j < poolSize; j++)
            {
                PointCloudRenderer pointManager = cloudRenderer[i];
                Mesh mesh = new Mesh();

                pointFrame = new GameObject("point cloud");
                pointFrame.AddComponent<MeshFilter>();
                pointFrame.AddComponent<MeshRenderer>();
                pointFrame.GetComponent<Renderer>().material = material;
                pointFrame.GetComponent<MeshFilter>().mesh = mesh;
                pointFrame.transform.SetParent(pointManager.pointPoolGroup.transform);
                pointFrame.SetActive(false);

                pointManager.pointGroupsPool.Enqueue(pointFrame);
            }
        }

        string sColorTable = CsCore.Configuration.ReadConfigIni("PointColor", "TableIndex");
        int idxColorTable = 0;
        int.TryParse(sColorTable, out idxColorTable);

        switch (idxColorTable)
        {
            case 0:
                currentColorTable = global::ColorTable.ColorTables.TableViridis;
                break;
            case 1:
                currentColorTable = global::ColorTable.ColorTables.TableRgb;
                break;
            case 2:
                currentColorTable = global::ColorTable.ColorTables.TableFlare;
                break;
            case 3:
                currentColorTable = global::ColorTable.ColorTables.TableRoyal;
                break;
            case 4:
                currentColorTable = global::ColorTable.ColorTables.TableSiemens;
                break;
            case 5:
                currentColorTable = global::ColorTable.ColorTables.TableBlueOrange;
                break;
            case 6:
                currentColorTable = global::ColorTable.ColorTables.TableNeonGreen;
                break;
        }


        string sColorMinHeight = CsCore.Configuration.ReadConfigIni("PointColor", "MinHeight");
        string sColorMaxHeight = CsCore.Configuration.ReadConfigIni("PointColor", "MaxHeight");
        float minHeight, maxHeight;
        if (float.TryParse(sColorMinHeight, out minHeight) == false)
        {
            minHeight = -100.0f;
        }
        if (float.TryParse(sColorMaxHeight, out maxHeight) == false)
        {
            maxHeight = 100.0f;
        }
        if (minHeight <= maxHeight)
        {
            this.minHeight = minHeight;
            this.maxHeight = maxHeight;
        }

        StartCoroutine(UpdateCoroutine());
    }

    
    public void UpdatePoints(int pier, int crane, Vector3[] points)
    {
        foreach (PointCloudRenderer mgr in cloudRenderer)
        {
            if(mgr.pier == pier && mgr.crane == crane)
            {
                Color[] colors = new Color[points.Length];
                int[] indices = new int[points.Length];
                if (colormode == Colormode.hight)
                {
                    for (uint i = 0; i < points.Length; i++)
                    {
                        indices[i] = (int)i;
                        colors[i] = ColorTable.GetTable(points[i].z, minHeight, maxHeight, currentColorTable);
                    }
                    mgr.UpdatePoints(points, colors, indices);
                    break;
                }
                else if (colormode == Colormode.soliod)
                {
                    for (uint i = 0; i < points.Length; i++)
                    {
                        indices[i] = (int)i;
                        colors[i] = pointColor;
                    }
                    mgr.UpdatePoints(points, colors, indices);
                    break;
                }
                else if (colormode == Colormode.None)
                {

                }
            }
        }
    }

    IEnumerator UpdateCoroutine()
    {
        while(true)
        {
            if(cloudRenderer.Count > 0 )
            {
                for (int i = 0; i < cloudRenderer.Count; i++)
                {
                    PointCloudRenderer renderer = cloudRenderer[i];

                    if (renderer.IsUpdated())
                    {
                        //pjh
                        var points = renderer.points;
                        var colors = renderer.colors;
                        var indices = renderer.indices;
                        //~pjh
                        renderer.ResetUpdatd();

                        try
                        {
                            
                            material.SetFloat("_PointSize", pointSize); // 포인트 사이즈 조정
                            GameObject pointFrame = renderer.pointGroupsPool.Dequeue();
                            if(points.Length != colors.Length) continue; //pjh
                            
                            pointFrame.SetActive(true);

                            Mesh mesh = pointFrame.GetComponent<MeshFilter>().mesh;
                            mesh.Clear();
                            mesh.vertices = points;//pjh
                            mesh.colors = colors;//pjh
                            mesh.SetIndices(indices, MeshTopology.Points, 0);//pjh
                            pointFrame.GetComponent<MeshFilter>().mesh = mesh;
                            //pjh
                            pointFrame.transform.SetParent(renderer.pointOrigin.transform);
                            pointFrame.transform.localPosition = Vector3.zero;
                            pointFrame.transform.localRotation = Quaternion.identity;
                            pointFrame.transform.localScale = Vector3.one;

                            // CYS
                            if(ProcessManager.instance != null && ProcessManager.instance.pointManager != null)
                                pointFrame.transform.SetParent(ProcessManager.instance.pointManager.gameObject.transform);

                            //~pjh
                            renderer.pointGroups.Enqueue(pointFrame);

                            while (renderer.pointGroups.Count > queueSize)
                            {
                                GameObject gameObject = renderer.pointGroups.Dequeue();
                                gameObject.SetActive(false);
                                gameObject.transform.SetParent(renderer.pointPoolGroup.transform);
                                gameObject.transform.localPosition = Vector3.zero;
                                gameObject.transform.localRotation = Quaternion.identity;
                                gameObject.transform.localScale = Vector3.one;

                                renderer.pointGroupsPool.Enqueue(pointFrame);
                            }
                        }
                        catch (System.InvalidOperationException e)
                        {
                            Debug.LogError(e.ToString());
                        }
                    }
                }
            }
            yield return wait;
        }
    }

    // private void Update()
    // {
    //     for (int i = 0; i < cloudRenderer.Count; i++)
    //     {
    //         PointCloudRenderer renderer = cloudRenderer[i];

    //         if (renderer.IsUpdated())
    //         {
    //             renderer.ResetUpdatd();

    //             try
    //             {
                    
    //                 material.SetFloat("_PointSize", pointSize); // 포인트 사이즈 조정

    //                 GameObject pointFrame = renderer.pointGroupsPool.Dequeue();

    //                 var points = renderer.points;
    //                 var colors = renderer.colors;
    //                 var indices = renderer.indices;
    //                 if(points.Length != colors.Length) continue; //pjh
                    
    //                 pointFrame.SetActive(true);

    //                 Mesh mesh = pointFrame.GetComponent<MeshFilter>().mesh;
    //                 mesh.Clear();
    //                 mesh.vertices = points;
    //                 mesh.colors = colors;
    //                 mesh.SetIndices(indices, MeshTopology.Points, 0);
    //                 pointFrame.GetComponent<MeshFilter>().mesh = mesh;
    //                 pointFrame.transform.SetParent(renderer.pointOrigin.transform);
    //                 //pjh
    //                 pointFrame.transform.localPosition = Vector3.zero;
    //                 pointFrame.transform.localRotation = Quaternion.identity;
    //                 pointFrame.transform.localScale = Vector3.one;
    //                 //~pjh
    //                 renderer.pointGroups.Enqueue(pointFrame);

    //                 while (renderer.pointGroups.Count > queueSize)
    //                 {
    //                     GameObject gameObject = renderer.pointGroups.Dequeue();
    //                     gameObject.transform.SetParent(renderer.pointPoolGroup.transform);
    //                     gameObject.SetActive(false);
    //                     renderer.pointGroupsPool.Enqueue(pointFrame);
    //                 }
    //             }
    //             catch (System.InvalidOperationException e)
    //             {
    //                 Debug.LogError(e.ToString());
    //             }
    //         }
    //     }

    // }
    //~pjh
    
}
