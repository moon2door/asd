using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProcessManager : MonoBehaviour
{
    public enum Tag
    {
        Pier,
        Crane,
        Dock
    }
    public ProcessCraneMonitoring craneMonitoring;
    public static ProcessManager instance;
    public GameObject World;
    public List<GameObject> mapFactorList;
    public Grid3D grid3D;
    // 시스템적인 부분에서는 CraneObject가 ProcessCraneMonitoring에서 가지고 오고 있어 간다히 가능하지만 UI와 오브젝트 단계에서 해당 위치를 접근이 불가능한 점에서 해당 코드 작성.
    public CraneManager craneManager;
    public PointManager pointManager;
    public DrawLiner drawLiner;
    public PierManager pierManager;
    public CraneGpsCalibrator_RadiusAngle gpsOffsetSetting;

    private void Awake()
    {
        if(instance == null)
            instance = this;

        if (World != null)
        {
            List<CraneObject> craneList = FindChildrenCrane(World);
            craneManager.SetCranes(craneList);

            List<PierObject> pierList = FindChildrenPier(World);
            pierManager.SetPiers(pierList);
        }
        else
        {
            Debug.LogError("ProcesManager: None World Setting");
        }

        // 나중에 컨피그로 넘겨야함.
        if (grid3D != null)
            grid3D.gameObject.SetActive(false);
    }

    void EnableMapFactor(bool active)
    {
        if(mapFactorList !=null && mapFactorList.Count > 0)
        {
            foreach(GameObject obj in mapFactorList)
            {
                obj.SetActive(active);
            }
        }
    }
    public void OnChangeGrid()
    {
        bool curStatus = grid3D.gameObject.activeSelf;
        if (curStatus == true)
        {
            OffGrid();
            pierManager.SetAllPier();
        }
        else
        {
            OnGrid();
            pierManager.SetGridMode();
        }
    }
    void OnGrid()
    {
        if(craneManager.CurCrane() != null)
        {
            EnableMapFactor(false);
            grid3D.OnGrid();
        }
    }
    void OffGrid()
    {
        if (craneManager.CurCrane() != null)
        {
            EnableMapFactor(true);
            grid3D.OffGrid();
        }
    }

    public List<CraneObject> FindChildrenCrane(GameObject parent)
    {
        List<GameObject> pierList = FindChildrenWithTag<GameObject>(World, Tag.Pier.ToString());

        List<CraneObject> craneList = new List<CraneObject>();
        foreach (GameObject pier in pierList)
        {
            List<CraneObject> pierInCraneList = FindChildrenWithTag<CraneObject>(pier, Tag.Crane.ToString());
            craneList.AddRange(pierInCraneList);
        }

        return craneList;
    }
    public List<PierObject> FindChildrenPier(GameObject parent)
    {
        List<GameObject> pierList = FindChildrenWithTag<GameObject>(World, Tag.Pier.ToString());

        List<PierObject> dockList = new List<PierObject>();
        foreach (GameObject pier in pierList)
        {
            List<PierObject> pierInCraneList = FindChildrenWithTag<PierObject>(pier, Tag.Dock.ToString());
            dockList.AddRange(pierInCraneList);
        }

        return dockList;
    }
    // 하위 자식 중에 특정 스크립트 검색(단 해당 스크립트를 가진 오브젝트 하위를 찾지는 않음)
    public List<T> FindChildrenWithTag<T>(GameObject parent, string tag)
    {
        List<T> classList = new List<T>();
        foreach (Transform child in parent.transform)
        {
            GameObject found = null;

            // 자기 자신이나 하위에서 태그 찾기
            if (child.CompareTag(tag))
            {
                found = child.gameObject;
            }
            else
            {
                found = FindFirstInChildBranch(child, tag);
            }

            // 찾았으면 결과에 추가 (한 개만)
            if (found != null)
            {
                if (typeof(T) == typeof(GameObject))
                {
                    // GameObject로 캐스팅 후 추가
                    classList.Add((T)(object)found);
                }
                else
                {
                    T comp = child.GetComponent<T>();
                    if (comp != null)
                        classList.Add(comp);
                }
            }
        }
        return classList;
    }
    GameObject FindFirstInChildBranch(Transform parent, string tag)
    {
        foreach (Transform child in parent)
        {
            // 1) 자기 자신 검사
            if (child.CompareTag(tag))
                return child.gameObject;

            // 2) 자식 트리 검사 (재귀)
            GameObject found = FindFirstInChildBranch(child, tag);
            if (found != null)
                return found;
        }

        return null;
    }

}
