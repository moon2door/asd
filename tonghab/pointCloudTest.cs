using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PointManager
{
    public PointManager(int pier, int crane, string name)
    {
        group =  GameObject.Find("pointCloudGroups");
        pointGroup = new GameObject(name);
        pointPoolGroup = new GameObject(name + "pool");
        pointGroup.transform.SetParent(group.transform);
        pointPoolGroup.transform.SetParent(group.transform);

        this.pier = pier;
        this.crane = crane;
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
        manager.Add(new PointManager(2, 0, "JIB1"));
        manager.Add(new PointManager(2, 1, "JIB2"));
        manager.Add(new PointManager(2, 2, "JIB3"));
        manager.Add(new PointManager(2, 3, "TTC23"));
        manager.Add(new PointManager(2, 4, "LLC18"));
        manager.Add(new PointManager(2, 5, "LLC19"));
        manager.Add(new PointManager(3, 0, "GC1"));
        manager.Add(new PointManager(3, 1, "GC2"));
        manager.Add(new PointManager(3, 2, "TC1"));
        manager.Add(new PointManager(3, 3, "TC2"));
        manager.Add(new PointManager(3, 4, "TC3"));
        manager.Add(new PointManager(3, 5, "TTC4"));
        manager.Add(new PointManager(3, 6, "TC5"));
        manager.Add(new PointManager(3, 7, "TC6"));
        manager.Add(new PointManager(4, 0, "LLC7"));
        manager.Add(new PointManager(4, 1, "LLC23"));
        manager.Add(new PointManager(1, 0, "LLC24"));
        manager.Add(new PointManager(1, 1, "LLC11"));
        manager.Add(new PointManager(1, 2, "LLC8"));
        manager.Add(new PointManager(1, 3, "LLC9"));
        manager.Add(new PointManager(5, 0, "LLC12"));
        manager.Add(new PointManager(5, 1, "LLC13"));
        manager.Add(new PointManager(6, 0, "LLC19"));
        manager.Add(new PointManager(6, 1, "LLC20"));
        manager.Add(new PointManager(7, 0, "LLC25"));
        manager.Add(new PointManager(7, 1, "LLC26"));

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

    private float updateTime = 0;

    void Update()
    {
        if(updateTime + 5 < UnityEngine.Time.time)
        {
            textPointSize = CsCore.Configuration.ReadConfigIni("PointSize", "Value");
            float.TryParse(textPointSize, out pointSize);
            updateTime = UnityEngine.Time.time;
        }

        for (int i = 0; i < manager.Count; i++)
        {
            PointManager pointManager = manager[i];
            if (pointManager.IsUpdated())
            {
                pointManager.ResetUpdatd();

                try
                {
                    GameObject pointFrame = pointManager.pointGroupsPool.Dequeue();

                    matVertex.SetFloat("_PointSize", pointSize);
                    pointFrame.SetActive(true);

                    Mesh mesh = pointFrame.GetComponent<MeshFilter>().mesh;
                    mesh.Clear();
                    mesh.vertices = pointManager.points;
                    mesh.colors = pointManager.colors;
                    mesh.SetIndices(pointManager.indices, MeshTopology.Points, 0);
                    pointFrame.GetComponent<MeshFilter>().mesh = mesh;
                    pointFrame.transform.SetParent(pointManager.pointGroup.transform);
                    pointManager.pointGroups.Enqueue(pointFrame);

                    while (pointManager.pointGroups.Count > queueSize)
                    {
                        GameObject gameObject = pointManager.pointGroups.Dequeue();
                        gameObject.transform.SetParent(pointManager.pointPoolGroup.transform);
                        gameObject.SetActive(false);
                        pointManager.pointGroupsPool.Enqueue(pointFrame);
                    }
                }
                catch(System.InvalidOperationException e)
                {
                    Debug.Log(e.ToString());
                }
            }
        }
    }
}
