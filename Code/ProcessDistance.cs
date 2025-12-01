using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using CsCore;

public class DistanceRenderer
{
    public DistanceRenderer(int pier, int crane, string name, Transform parent)
    {
        this.parent = parent;
        this.pier = pier;
        this.crane = crane;

        bUpdate = false;
    }

    public void UpdateDistance(DistanceObject.StDistance[] arrstDistance)
    {
        this.arrstDistance = arrstDistance;
        bUpdate = true;
    }

    public void ResetUpdate()
    {
        bUpdate = false;
    }

    public bool IsUpdated()
    {
        return bUpdate;
    }

    public void ClearDistanceObjects()
    {
        for (int i = 0; i < distanceObjects.Count; i++)
        {
            GameObject.DestroyImmediate(distanceObjects[i]);
        }
        distanceObjects.Clear();
    }

    public void AddDistanceObject(GameObject distanceObject)
    {
        distanceObjects.Add(distanceObject);
    }

    public int pier;
    public int crane;

    public bool bUpdate;
    public Transform parent;
    public DistanceObject.StDistance[] arrstDistance = null;
    public List<GameObject> distanceObjects = new List<GameObject>();
}

public class ProcessDistance : MonoBehaviour
{
    public GameObject myPrefab;
    public int numDrawDistance = 5;

    [SerializeField]
    private TabOperation tabOperation;
    [SerializeField]
    private TabStatus tabStatus;

    private List<DistanceRenderer> distanceRenderer = new List<DistanceRenderer>();

    //string initSet = Configuration.ReadConfigIni("InitSet", "set");
    private void Start()
    {
        if (ProcessManager.instance == null && ProcessManager.instance.craneManager == null)
            return;
        foreach (CraneObject crane in ProcessManager.instance.craneManager.GetCranes())//pjh
        {
            distanceRenderer.Add(new DistanceRenderer(crane.pierNum, crane.craneNum, crane.craneName, crane.pointOrigin.transform));
        }
    }

    private GameObject InstantiateDistance(GameObject prefab, Transform parent, DistanceObject.StDistance distance)
    {
        GameObject gameObject = Instantiate(myPrefab, parent);
        DistanceObject distanceObject = gameObject.GetComponent<DistanceObject>();
        distanceObject.UpdateDistance(distance);
        return gameObject;
    }
    void Update()
    {
        foreach (DistanceRenderer renderer in distanceRenderer)
        {
            if(renderer.IsUpdated())
            {
                renderer.ClearDistanceObjects();

                string[] textLines = new string[renderer.arrstDistance.Length];
                int countDistance = 0;
                int[] countPart = new int[5];
                for (int i = 0; i < renderer.arrstDistance.Length; i++)
                {
                    if (renderer.arrstDistance[i].level > 0)
                    {
                        GameObject gameObject = InstantiateDistance(myPrefab, renderer.parent, renderer.arrstDistance[i]);
                        renderer.AddDistanceObject(gameObject);
                        textLines[countDistance] = $"{renderer.arrstDistance[i].distance:0.0}m";
                        if (renderer.arrstDistance[i].level == 1)
                        {
                            textLines[countDistance] += "(주의)";
                        }
                        else if (renderer.arrstDistance[i].level == 2)
                        {
                            textLines[countDistance] += "(경고)";
                        }
                        else if (renderer.arrstDistance[i].level == 3)
                        {
                            textLines[countDistance] += "(정지)";
                        }
                        else
                        {
                        }

                        int craneIndex = renderer.arrstDistance[i].craneIndex;
                        countPart[craneIndex]++;
                        countDistance++;
                    }
                }

                List<DistanceObject.StDistance> remained = new List<DistanceObject.StDistance>();
                foreach (DistanceObject.StDistance d in renderer.arrstDistance)
                {
                    if (countDistance >= numDrawDistance) break;

                    int craneIndex = d.craneIndex;
                    if (d.level == 0 && countPart[craneIndex] == 0)
                    {
                        GameObject gameObject = InstantiateDistance(myPrefab, renderer.parent, d);
                        renderer.AddDistanceObject(gameObject);
                        //Distance List
                        textLines[countDistance] = $"{d.distance:0.00}m ";
                        countDistance++;
                        countPart[craneIndex]++;
                    }
                    else
                    {
                        remained.Add(d);
                    }
                }

                foreach (DistanceObject.StDistance d in remained)
                {
                    if (countDistance >= numDrawDistance) break;

                    int craneIndex = d.craneIndex;
                    if (d.level == 0)
                    {
                        GameObject gameObject = InstantiateDistance(myPrefab, renderer.parent, d);
                        renderer.AddDistanceObject(gameObject);
                        //Distance List
                        textLines[countDistance] = $"{d.distance:0.00}m ";
                        countDistance++;
                        countPart[craneIndex]++;
                    }
                }
                
                string textDistance = "";
                int maxCollision = 0;
                float minDistance = float.MaxValue;
                for (int i = 0; i < renderer.arrstDistance.Length; i++)
                {
                    float d = renderer.arrstDistance[i].distance;
                    int collision = renderer.arrstDistance[i].level;
                    if (maxCollision < collision)
                    {
                        maxCollision = collision;
                        minDistance = d;
                    }
                    else if (maxCollision == collision && minDistance > d)
                    {
                        minDistance = d;
                    }
                    else
                    {
                    }
                }
                
                if (minDistance < 100)
                {
                    //Distance Text
                    textDistance += $"{minDistance.ToString("0.00")}";
                    textDistance += "m";
                }
                tabOperation.UpdateTextDistance(textDistance, textLines);
                renderer.ResetUpdate();
            }
        }
    }

    public void UpdateDistance(int pier, int crane, DistanceObject.StDistance[] distance)
    {
        foreach (DistanceRenderer renderer in distanceRenderer)
        {
            if (renderer.pier == pier && renderer.crane == crane)
            {
                renderer.UpdateDistance(distance);
            }
        }
    }
}
