using System.Collections.Generic;
using UnityEngine;

//pjh
public class CraneManager : MonoBehaviour
{
    public void PierReset()
    {
        for (int i = 0; i < cranes.Count; i++)
        {
            cranes[i].transform.parent.gameObject.SetActive(false);
        }
    }
    // 하버사인 거리 (미터)
    List<CraneObject> cranes = new List<CraneObject>();
    public void SetCranes(List<CraneObject> list)
    {
        cranes = list;
    }
   
    public List<CraneObject> GetCranes(){
        return cranes;
    }
    public int GetCountCrans()
    {
        if (cranes == null)
            return 0;
        return cranes.Count;
    }

    public CraneObject GetCrane(int index){
        if(index < cranes.Count && index >= 0)
            return cranes[index];
        return null;
    }

    public CraneObject GetCraneByPierIdAndCraneId(int pier, int craneNum)
    {
        for(int i=0;i< cranes.Count; i++)
        {
            if(cranes[i].pierNum == pier && cranes[i].craneNum == craneNum)
            {
                return cranes[i];
            }
        }
        return null;
    }

    public void ActivePier(CraneUtility.PierType type)
    {
        if (cranes == null)
            return;
        for(int i = 0; i < cranes.Count; i++)
        {
            if(cranes[i].pierNum == (int)type)
            {
                cranes[i].transform.parent.gameObject.SetActive(true);
                return;
            }
        }
    }
    public CraneObject CurCrane()
    {
        CraneObject curCrane = null; 
        if(ProcessManager.instance != null)
        {
            if (ProcessManager.instance.craneMonitoring.logControlMenuBar.IsLogPlaying() == true)
            {
                curCrane = ProcessManager.instance.craneMonitoring.logControlMenuBar.currentCrane;
            }
            else
            {
                curCrane = ProcessManager.instance.craneMonitoring.currentCrane;
            }
        }
        return curCrane;
    }
}
//~pjh