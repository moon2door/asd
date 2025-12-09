using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using CraneKey = System.Int32;

public class CraneInfo : MonoBehaviour
{
    [ReadOnly] public List<int> cranePierCode = new List<int>();
    [ReadOnly] public List<int> craneCode = new List<int>();
    [ReadOnly] public List<string> cranePierName = new List<string>();
    [ReadOnly] public List<string> craneName = new List<string>();
    [ReadOnly] public List<int> numSensors = new List<int>();
   [ReadOnly] public List<GameObject> craneGameObject = new List<GameObject>();

    public Dictionary<CraneKey, int> dictCrane = new Dictionary<int, int>();
    public Dictionary<CraneKey, int> dictPier = new Dictionary<CraneKey, int>();
    public Dictionary<CraneKey, string> dictPierName = new Dictionary<CraneKey, string>();
    public Dictionary<CraneKey, string> dictCraneName = new Dictionary<CraneKey, string>();
    public Dictionary<CraneKey, int> dictNumSensors = new Dictionary<CraneKey, int>();
    public Dictionary<CraneKey, GameObject> dictCraneGameObject = new Dictionary<CraneKey, GameObject>();
    [ReadOnly] public List<GameObject> pierCenter;

    [ReadOnly] public List<CraneKey> keys = new List<CraneKey>();

    void Awake()
    {
        //pjh
        var craneParts = GameObject.FindObjectsOfType<CraneParts>();
        List<(int index, CraneParts parts)> indexedParts = new List<(int, CraneParts)>();

        // craneParts를 인덱스와 함께 리스트에 추가
        for (int i = 0; i < craneParts.Length; i++)
        {
            indexedParts.Add((craneParts[i].craneIndex, craneParts[i]));
        }
        indexedParts.Sort((x, y) => x.index.CompareTo(y.index));

        cranePierCode.Clear();
        craneCode.Clear();
        cranePierName.Clear();
        craneName.Clear();
        numSensors.Clear();
        craneGameObject.Clear();
        
        foreach (var (index, parts) in indexedParts)
        {
            int key = GetCraneKeycode(parts.pierCode, parts.craneCode);

            cranePierCode.Add(parts.pierCode);
            dictPier.Add(key, parts.pierCode);

            craneCode.Add(parts.craneCode);
            dictCrane.Add(key, parts.craneCode);

            cranePierName.Add(parts.pierName);
            dictPierName.Add(key, parts.pierName);

            craneName.Add(parts.craneName);
            dictCraneName.Add(key, parts.craneName);

            numSensors.Add(parts.sensorCount);
            dictNumSensors.Add(key, parts.sensorCount);

            craneGameObject.Add(parts.gameObject);
            dictCraneGameObject.Add(key, parts.gameObject);

            keys.Add(key);
        }

        // for (int i = 0; i < cranePierCode.Count; i++)
        // {
        //     int key = GetCraneKeycode(cranePierCode[i], craneCode[i]);
        //     dictPier.Add(key, cranePierCode[i]);
        //     dictCrane.Add(key, craneCode[i]);
        //     dictPierName.Add(key, cranePierName[i]);
        //     dictCraneName.Add(key, craneName[i]);
        //     dictNumSensors.Add(key, numSensors[i]);
        //     dictCraneGameObject.Add(key, craneGameObject[i]);
        //     keys.Add(key);
        // }
        //~pjh
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
