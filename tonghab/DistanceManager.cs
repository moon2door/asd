using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DistanceManager : MonoBehaviour
{
    public CraneInfo craneInfo;

    public GameObject prefab;

    private GameObject manager;

    private Dictionary<int, DistanceData[]> distanceData = new Dictionary<int, DistanceData[]>();

    public Dictionary<int, GameObject> distanceObjects = new Dictionary<int, GameObject>();

    public Dictionary<int, bool> bUpdate = new Dictionary<int, bool>();

    private Dictionary<int, GameObject> groups = new Dictionary<int, GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        manager = new GameObject("distanceData");

        foreach (int key in craneInfo.keys)
        {
            string name = string.Format("Distance{0}{1}", craneInfo.dictPier[key], craneInfo.dictCraneName[key]);

            GameObject group = new GameObject(name);
            group.transform.SetParent(manager.transform);
            
            GameObject gameObject = Instantiate(prefab, new Vector3(0, 0, 0), Quaternion.identity);
            gameObject.transform.SetParent(group.transform);
            gameObject.transform.position = craneInfo.dictCraneGameObject[key].transform.position;
            gameObject.transform.rotation = craneInfo.dictCraneGameObject[key].transform.rotation;
            gameObject.SetActive(false);

            distanceData.Add(key, new DistanceData[0]);
            distanceObjects.Add(key, gameObject);
            bUpdate.Add(key, false);
            groups.Add(key, group);
        }
    }

    void Update()
    {
        foreach (int key in craneInfo.keys)
        {
            if (bUpdate[key] == true)
            {
                if (distanceData[key].Length > 0)
                {
                    distanceObjects[key].SetActive(true);
                    DistanceObject distanceObject = distanceObjects[key].GetComponent<DistanceObject>();
                    distanceObject.UpdateDistance(distanceData[key][0]);
                    distanceObject.transform.SetParent(groups[key].transform);
                    distanceObject.transform.position = craneInfo.dictCraneGameObject[key].transform.position;
                    distanceObject.transform.rotation = craneInfo.dictCraneGameObject[key].transform.rotation;
                }
                else
                {
                    distanceObjects[key].SetActive(false);
                }

                bUpdate[key] = false;
            }
        }
    }

    public void UpdateDistance(int pier, int crane, DistanceData[] distance)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);
        distanceData[key] = distance;
        bUpdate[key] = true;
    }
}
