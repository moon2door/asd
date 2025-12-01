using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PierManager : MonoBehaviour
{
    public List<PierObject> pierList;

    public void SetPiers(List<PierObject> list)
    {
        pierList = list;
    }

    public void SetAllPier()
    {
        for(int i = 0; i < pierList.Count; i++)
        {
            PierObject pier = pierList[i];
            pier.gameObject.SetActive(false);
            for (int j = 0; j < pier.withCranes.Count; j++)
            {
                if(ProcessManager.instance != null)
                {
                    if(pier.withCranes[j] == ProcessManager.instance.craneManager.CurCrane())
                    {
                        pier.gameObject.SetActive(true);
                        break;
                    }
                }
            }
        }
    }
    public void SetGridMode()
    {
        for (int i = 0; i < pierList.Count; i++)
        {
            PierObject pier = pierList[i];
            pier.gameObject.SetActive(false);
        }
    }

}
