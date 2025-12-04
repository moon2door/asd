using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using CraneKey = System.Int32;

public class CraneInfo : MonoBehaviour
{
    public List<int> cranePierCode = new List<int>();
    public List<int> craneCode = new List<int>();
    public List<string> cranePierName = new List<string>();
    public List<string> craneName = new List<string>();
    public List<int> numSensors = new List<int>();
    public List<GameObject> craneGameObject = new List<GameObject>();

    public Dictionary<CraneKey, int> dictCrane = new Dictionary<int, int>();
    public Dictionary<CraneKey, int> dictPier = new Dictionary<CraneKey, int>();
    public Dictionary<CraneKey, string> dictPierName = new Dictionary<CraneKey, string>();
    public Dictionary<CraneKey, string> dictCraneName = new Dictionary<CraneKey, string>();
    public Dictionary<CraneKey, int> dictNumSensors = new Dictionary<CraneKey, int>();
    public Dictionary<CraneKey, GameObject> dictCraneGameObject = new Dictionary<CraneKey, GameObject>();
    public List<GameObject> pierCenter;

    [HideInInspector]
    public List<CraneKey> keys = new List<CraneKey>();

    void Awake()
    {
        for (int i = 0; i < cranePierCode.Count; i++)
        {
            int key = GetCraneKeycode(cranePierCode[i], craneCode[i]);
            dictPier.Add(key, cranePierCode[i]);
            dictCrane.Add(key, craneCode[i]);
            dictPierName.Add(key, cranePierName[i]);
            dictCraneName.Add(key, craneName[i]);
            dictNumSensors.Add(key, numSensors[i]);
            dictCraneGameObject.Add(key, craneGameObject[i]);
            keys.Add(key);
        }
    }
    void Start()
    {
    }
    
    public int GetCraneKeycode(int pier, int crane)
    {
        return (pier * 100 + crane);
    }
    

    public int GetNumTotalCranes()
    {
        return keys.Count;
    }
    public bool Contains(int pier, int crane)
    {
        bool ret = false;
        for (int i = 0; i < cranePierCode.Count; i++)
        {
            if (cranePierCode[i] == pier && craneCode[i] == crane)
            {
                ret = true;
                break;
            }
        }
        return ret;
    }

}
