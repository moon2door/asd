using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DistanceObject : MonoBehaviour
{
    public Vector3 position1 = new Vector3(0, 0, 0);
    public Vector3 position2 = new Vector3(0, 0, 0);
    public float distance;
    public int level;

    private bool bUpdate = true;

    private GameObject targetObject;
    private GameObject canvasObject;
    private GameObject panelObject;
    private GameObject textObject;
    private GameObject lineObject;
    private GameObject panelLevel3;
    private GameObject panelLevel2;
    private GameObject panelLevel1;
    private Transform transformDistanceData;

    public void UpdateDistance(DistanceData d)
    {
        if (d != null)
        {
            position1 = d.p1;
            position2 = d.p2;
            distance = d.distance;
            level = d.level;
            bUpdate = true;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        panelObject = transform.GetChild(0).GetChild(0).gameObject;
        textObject = transform.GetChild(0).GetChild(0).GetChild(0).gameObject;
        lineObject = transform.GetChild(1).gameObject;
        targetObject = transform.GetChild(2).gameObject;
        panelLevel3 = transform.GetChild(0).GetChild(0).GetChild(1).gameObject;
        panelLevel2 = transform.GetChild(0).GetChild(0).GetChild(2).gameObject;
        panelLevel1 = transform.GetChild(0).GetChild(0).GetChild(3).gameObject;
        transformDistanceData = transform.parent.parent;
    }

    // Update is called once per frame
    void Update()
    {
        if(bUpdate)
        {
            bUpdate = false;
            
            // Update line
            LineRenderer lineRenderer = lineObject.GetComponent<LineRenderer>();
            lineRenderer.positionCount = 2;
            lineRenderer.SetPosition(0, position1);
            lineRenderer.SetPosition(1, position2);

            // Update text
            panelObject.SetActive(true);
            Text txt = textObject.GetComponent<Text>();
            txt.text = "" + distance.ToString("F1") + "m";
            
            targetObject.transform.localPosition = (position1 + position2) / 2 + new Vector3(0, 0, 2.5f);

            // Update panel
            switch (level)
            {
                case 1: // 주의
                    panelLevel1.SetActive(false);
                    panelLevel2.SetActive(false);
                    panelLevel3.SetActive(true);
                    break;
                case 2: //  경고
                    panelLevel1.SetActive(false);
                    panelLevel2.SetActive(true);
                    panelLevel3.SetActive(false);
                    break;
                case 3: //  정지
                    panelLevel1.SetActive(true);
                    panelLevel2.SetActive(false);
                    panelLevel3.SetActive(false);
                    break;
                default: // 정상
                    panelLevel1.SetActive(false);
                    panelLevel2.SetActive(false);
                    panelLevel3.SetActive(false);
                    break;
            }
        }
    }
}
