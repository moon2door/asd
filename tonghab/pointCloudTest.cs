using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PointManager
{
    public PointManager(CraneParts parts)//pjh
    {
        this.pier = parts.pierCode;
        this.crane = parts.craneCode;

        group =  GameObject.Find("pointCloudGroups");
        pointPoolGroup = new GameObject(parts.craneName + "pool");
        //pjh
        pointGroup = parts.pointCloudTransform;
        pointPoolGroup.transform.SetParent(parts.gameObject.transform);
        pointPoolGroup.transform.localPosition = Vector3.zero;
        pointPoolGroup.transform.localRotation = Quaternion.identity;
        //~pjh
        
        bUpdate = false;
    }

    public void UpdatePoints(uint numPoints, Vector3[] points, Color[] colors, int[] indices)
    {
        numPoints = System.Math.Min(numPoints, limitPoints);
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
    public uint numPoints;
    public Vector3[] points;
    public Color[] colors;
    public int[] indices;

    //GameObject
    public GameObject group;
    public GameObject pointGroup;
    public GameObject pointPoolGroup;
    public Queue<GameObject> pointGroups = new Queue<GameObject>();
    public Queue<GameObject> pointGroupsPool = new Queue<GameObject>();
}
public class pointCloudTest : MonoBehaviour
{
    public Material matVertex;
    public int queueSize = 10;
    public int poolSize = 100;
    private List<PointManager> manager = new List<PointManager>();
    private string textPointSize = "";
    private float pointSize = 0.3f;


    void Start()
    {
        //pjh
        Application.targetFrameRate = 20;//int.Parse(CsCore.Configuration.ReadConfigIni("TargetFrameRate" , "Frame", "30"));


        var  craneInfo = GetComponent<CraneInfo>();
        for(int i =0; i < craneInfo.keys.Count; i++)
        {
            manager.Add(new PointManager(craneInfo.craneGameObject[i].GetComponent<CraneParts>()));
        }
        //~pjh

        for (int i = 0; i < manager.Count; i++)
        {
            for(int j = 0; j< poolSize; j++)
            {
                PointManager pointManager = manager[i];
                Mesh mesh = new Mesh();

                GameObject pointFrame = new GameObject("point cloud");
                pointFrame.AddComponent<MeshFilter>();
                pointFrame.AddComponent<MeshRenderer>();
                pointFrame.GetComponent<Renderer>().material = matVertex;
                pointFrame.GetComponent<MeshFilter>().mesh = mesh;
                pointFrame.transform.SetParent(pointManager.pointPoolGroup.transform);
                pointFrame.SetActive(false);

                pointManager.pointGroupsPool.Enqueue(pointFrame);
            }
        }
        StartCoroutine(UpdateCoroutine());
    }
    
    public void UpdatePoints(int pier, int crane, uint _numPoints, Vector3[] _vertices, Color[] _colors, int[] _indices)
    {
        foreach (PointManager mgr in manager)
        {
            if(mgr.pier == pier && mgr.crane == crane)
            {
                mgr.UpdatePoints(_numPoints, _vertices, _colors, _indices);
                break;
            }
        }
    }
    readonly WaitForSecondsRealtime wait = new WaitForSecondsRealtime(1);
    //pjh
    IEnumerator UpdateCoroutine()
    {
        while(true)
        {
            textPointSize = CsCore.Configuration.ReadConfigIni("PointSize", "Value");
            float.TryParse(textPointSize, out pointSize);

            for (int i = 0; i < manager.Count; i++)
            {
                PointManager pointManager = manager[i];
                if (pointManager.IsUpdated())
                {
                    //pjh
                    var points = pointManager.points;
                    var colors = pointManager.colors;
                    var indices = pointManager.indices;
                    //~pjh
                    pointManager.ResetUpdatd();

                    try
                    {
                        if(points.Length != colors.Length)continue;//pjh

                        GameObject pointFrame = pointManager.pointGroupsPool.Dequeue();

                        matVertex.SetFloat("_PointSize", pointSize);
                        pointFrame.SetActive(true);

                        Mesh mesh = pointFrame.GetComponent<MeshFilter>().mesh;
                        mesh.Clear();
                        mesh.vertices = points;//pjh
                        mesh.colors = colors;//pjh
                        mesh.SetIndices(indices, MeshTopology.Points, 0);//pjh
                        pointFrame.GetComponent<MeshFilter>().mesh = mesh;
                        pointFrame.transform.SetParent(pointManager.pointGroup.transform);
                        //pjh
                        pointFrame.transform.localPosition = Vector3.zero;
                        pointFrame.transform.localRotation = Quaternion.identity;
                        pointFrame.transform.localScale = Vector3.one;
                        //

                        // CES
                        if (pointManager.group != null )
                            pointFrame.transform.SetParent(pointManager.group.transform);

                        pointManager.pointGroups.Enqueue(pointFrame);

                        while (pointManager.pointGroups.Count > queueSize)
                        {
                            GameObject gameObject = pointManager.pointGroups.Dequeue();
                            gameObject.transform.SetParent(pointManager.pointPoolGroup.transform);
                            gameObject.SetActive(false);

                            // CES
                            gameObject.transform.localPosition = Vector3.zero;
                            gameObject.transform.localRotation = Quaternion.identity;
                            gameObject.transform.localScale = Vector3.one;

                            pointManager.pointGroupsPool.Enqueue(pointFrame);
                        }
                    }
                    catch(System.InvalidOperationException e)
                    {
                        Debug.Log(e.ToString());
                    }
                }
            }
            yield return wait;
        }   
    }

    // private float updateTime = 0;
    // void Update()
    // {
    //     if(updateTime + 5 < UnityEngine.Time.time)
    //     {
    //         textPointSize = CsCore.Configuration.ReadConfigIni("PointSize", "Value");
    //         float.TryParse(textPointSize, out pointSize);
    //         updateTime = UnityEngine.Time.time;
    //     }

    //     for (int i = 0; i < manager.Count; i++)
    //     {
    //         PointManager pointManager = manager[i];
    //         if (pointManager.IsUpdated())
    //         {
    //             pointManager.ResetUpdatd();

    //             try
    //             {
    //                 if(pointManager.points.Length != pointManager.colors.Length)continue;//pjh
                    
    //                 GameObject pointFrame = pointManager.pointGroupsPool.Dequeue();

    //                 matVertex.SetFloat("_PointSize", pointSize);
    //                 pointFrame.SetActive(true);

    //                 Mesh mesh = pointFrame.GetComponent<MeshFilter>().mesh;
    //                 mesh.Clear();
    //                 mesh.vertices = pointManager.points;
    //                 if(pointManager.colors.Length != mesh.vertices.Length) continue; //pjh
    //                 mesh.colors = pointManager.colors;
    //                 mesh.SetIndices(pointManager.indices, MeshTopology.Points, 0);
    //                 pointFrame.GetComponent<MeshFilter>().mesh = mesh;
    //                 pointFrame.transform.SetParent(pointManager.pointGroup.transform);
    //                 //pjh
    //                 pointFrame.transform.localPosition = Vector3.zero;
    //                 pointFrame.transform.localRotation = Quaternion.identity;
    //                 pointFrame.transform.localScale = Vector3.one;
    //                 //
    //                 pointManager.pointGroups.Enqueue(pointFrame);

    //                 while (pointManager.pointGroups.Count > queueSize)
    //                 {
    //                     GameObject gameObject = pointManager.pointGroups.Dequeue();
    //                     gameObject.transform.SetParent(pointManager.pointPoolGroup.transform);
    //                     gameObject.SetActive(false);
    //                     pointManager.pointGroupsPool.Enqueue(pointFrame);
    //                 }
    //             }
    //             catch(System.InvalidOperationException e)
    //             {
    //                 Debug.Log(e.ToString());
    //             }
    //         }
    //     }
    // }
    //~pjh
}
