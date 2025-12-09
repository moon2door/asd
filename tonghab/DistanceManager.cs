using System.Collections.Generic;
using UnityEngine;

public class DistanceManager : MonoBehaviour
{
    private CraneInfo craneInfo;

    public GameObject prefab;

    private GameObject manager;

    private Dictionary<int, DistanceData[]> distanceData = new Dictionary<int, DistanceData[]>();

    public Dictionary<int, DistanceObject> distanceObjects = new Dictionary<int, DistanceObject>();

    public Dictionary<int, bool> bUpdate = new Dictionary<int, bool>();

    private Dictionary<int, GameObject> groups = new Dictionary<int, GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        craneInfo = GetComponent<CraneInfo>();
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
            distanceObjects.Add(key, gameObject.GetComponent<DistanceObject>());
            bUpdate.Add(key, false);
            //pjh
            //if(craneInfo.dictPier[key]<8)
            //    groups.Add(key, group);
            //else
            //    {
                    groups.Add(key, craneInfo.dictCraneGameObject[key].GetComponent<CraneParts>().CraneBody.GetChild(0).gameObject);
             //   }
            //~pjh

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
                    DistanceObject distanceObject = distanceObjects[key];
                    distanceObject.gameObject.SetActive(true);
                    distanceObject.UpdateDistance(distanceData[key][0]);
                    distanceObject.transform.SetParent(groups[key].transform);
                    //pjh
                    // if(craneInfo.dictPier[key]<8)
                    // {
                    //     distanceObject.transform.position = craneInfo.dictCraneGameObject[key].transform.position;
                    //     distanceObject.transform.rotation = craneInfo.dictCraneGameObject[key].transform.rotation;
                    // }else
                    // {
                        distanceObject.transform.localPosition = Vector3.zero;
                        distanceObject.transform.localRotation = Quaternion.identity;
                    //}
                    //~pjh
                }
                else
                {
                    distanceObjects[key].gameObject.SetActive(false);
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
