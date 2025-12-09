using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using CraneKey = System.Int32;
public struct CraneColorMode
{
    public CraneColorMode(List<MeshRenderer> meshRendererList)
    {
        this.meshRendererList = meshRendererList;
        bFlickering = false;
        bActive = true;
        flickeringLevel = 0;
        originalColor = Color.black;
        if(meshRendererList.Count > 0)
        {
            originalColor = meshRendererList[0].material.color;
        }
    }

    public List<MeshRenderer> meshRendererList;
    public int flickeringLevel;
    public bool bFlickering;
    public bool bActive;
    public Color originalColor;
}

public class CraneColorControl : MonoBehaviour
{
    CraneInfo craneInfo;
    public Dictionary<CraneKey, MeshRenderer> meshes;
    public MeshRenderer craneMesh;
    public Color colorDisabled = new Color(0.1f, 0.1f, 0.1f, 1.0f);
    public Color colorLevel0 = new Color(0, 153.0f / 255.0f, 0, 1);
    public Color colorLevel1 = new Color(252.0f / 255.0f, 204.0f / 255.0f, 0, 1);
    public Color colorLevel2 = new Color(1, 0, 0);

    private Dictionary<CraneKey, CraneColorMode> craneList = new Dictionary<CraneKey, CraneColorMode>();
    //private Color originalColor;

    private int pierSelected = -1;
    private int craneSelected = -1;
    

    void Start()
    {
        craneInfo = GetComponent<CraneInfo>();
        //originalColor = craneMesh.material.color;

        foreach(int key in craneInfo.keys)
        {
            List<MeshRenderer> meshList = new List<MeshRenderer>();
            craneInfo.dictCraneGameObject[key].GetComponentsInChildren(true, meshList);
            craneList.Add(key, new CraneColorMode(meshList));
        }
    }

    // Update is called once per frame
    void Update()
    {
        foreach (int key in craneInfo.keys)
        {
            if (!craneList.ContainsKey(key)) continue;

            CraneColorMode colorMode = craneList[key];
            Color originalColor = colorMode.originalColor;

            if (!colorMode.bActive) // 크레인 비활성화 상태
            {
                for (int i = 0; i < colorMode.meshRendererList.Count; i++)
                {
                    colorMode.meshRendererList[i].material.color = colorDisabled;
                }
            }
            else if (colorMode.bFlickering) // 크레인 경고 상태
            {
                float flicker = Mathf.Abs(Mathf.Sin(Time.time * 4));
                flicker = flicker * 2;
                if (flicker > 1) flicker = 1;

                for (int i = 0; i < colorMode.meshRendererList.Count; i++)
                {
                    if (colorMode.flickeringLevel == 0)
                    {
                        colorMode.meshRendererList[i].material.color = originalColor * (1 - flicker) + colorLevel0 * (flicker);
                    }
                    if (colorMode.flickeringLevel == 1)
                    {
                        colorMode.meshRendererList[i].material.color = originalColor * (1 - flicker) + colorLevel1 * (flicker);
                    }
                    if (colorMode.flickeringLevel == 2)
                    {
                        colorMode.meshRendererList[i].material.color = originalColor * (1 - flicker) + colorLevel2 * (flicker);
                    }
                    else { }
                }
            }
            else // 일반 상태
            {
                for (int j = 0; j < colorMode.meshRendererList.Count; j++)
                {
                    if (colorMode.meshRendererList[j])
                    {
                        colorMode.meshRendererList[j].material.color = originalColor;
                    }
                }
            }

            if (key == craneInfo.GetCraneKeycode(pierSelected, craneSelected))
            {
                for (int i = 0; i < colorMode.meshRendererList.Count; i++)
                {
                    if (colorMode.meshRendererList[i])
                    {
                        colorMode.meshRendererList[i].material.SetColor("_OutlineColor", Color.green);
                    }
                }
            }
            else
            {
                for (int i = 0; i < colorMode.meshRendererList.Count; i++)
                {
                    if (colorMode.meshRendererList[i])
                    {
                        colorMode.meshRendererList[i].material.SetColor("_OutlineColor", Color.black);
                    }
                }
            }
        }
    }

    public void SetFlickering(int pier, int crane, bool flickering)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if (craneList.ContainsKey(key))
        {
            CraneColorMode craneColorMode = craneList[key];
            craneColorMode.bFlickering = flickering;
            craneList[key] = craneColorMode;
        }
    }

    public void SetActiveStatus(int pier, int crane, bool activeColor)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if(craneList.ContainsKey(key))
        {
            CraneColorMode craneColorMode = craneList[key];
            craneColorMode.bActive = activeColor;
            craneList[key] = craneColorMode;
        }
    }

    public void SetFlickeringColor(int pier, int crane, int flickeringLevel)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if (craneList.ContainsKey(key))
        {
            CraneColorMode craneColorMode = craneList[key];
            craneColorMode.flickeringLevel = flickeringLevel;
            craneList[key] = craneColorMode;
        }
    }

    public void SetSelectCrane(int pier, int crane)
    {
        pierSelected = pier;
        craneSelected = crane;
    }
}
