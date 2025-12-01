using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using CsCore;

public class DistanceObject : MonoBehaviour
{
    public struct StDistance
    {
        public Vector3 position1;
        public Vector3 position2;
        public float distance;
        public int level;
        public int craneIndex;
    }
    private StDistance distance;
     //pjh private bool bUpdate = true;

    private GameObject targetObject;
    private GameObject canvasObject;
    private GameObject panelObject;
    private GameObject textObject;
    private GameObject lineObject;
    private GameObject linePathObject;
    private GameObject panelLevel3;
    private GameObject panelLevel2;
    private GameObject panelLevel1;
    private Transform transformDistanceData;
    //pjh private ProcessDistance processDistance = new ProcessDistance();
    private Vector3 prePosition;
    private Vector3 tempPosition1;
    private Vector3 tempPosition2;


    public void UpdateDistance(StDistance distance)
    {
        this.distance = distance;
        //bUpdate = true; pjh
    }

    private string GetStringPart(int craneIndex)
    {
        string craneIndexStr = "";
        switch (craneIndex)
        {
            case 0:
                craneIndexStr = "B";
                break;
            case 1:
                craneIndexStr = "H";
                break;
            case 2:
                craneIndexStr = "L";
                break;
            default:
                break;
        }

        return craneIndexStr;
    }

    // Start is called before the first frame update
    void Awake()
    {
        canvasObject = transform.GetChild(0).gameObject;
        panelObject = transform.GetChild(0).GetChild(0).gameObject;
        textObject = transform.GetChild(0).GetChild(0).GetChild(0).gameObject;
        lineObject = transform.GetChild(1).gameObject;
        linePathObject= transform.GetChild(3).gameObject;
        targetObject = transform.GetChild(2).gameObject;
        panelLevel3 = transform.GetChild(0).GetChild(0).GetChild(1).gameObject;
        panelLevel2 = transform.GetChild(0).GetChild(0).GetChild(2).gameObject;
        panelLevel1 = transform.GetChild(0).GetChild(0).GetChild(3).gameObject;
        transformDistanceData = transform.parent.parent;

        panelObject.SetActive(false);
        lineObject.SetActive(false);
        linePathObject.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {   
        //if (prePosition == targetObject.gameObject.transform.position)
        //{
        //    tempPosition2= targetObject.gameObject.transform.localPosition;
        //}

        //if (bUpdate)
        {
            //bUpdate = false; //pjh
            prePosition = targetObject.gameObject.transform.localPosition;

            // Update line
            LineRenderer lineRenderer = lineObject.GetComponent<LineRenderer>();
            lineRenderer.positionCount = 2;
            lineRenderer.SetPosition(0, distance.position1);
            lineRenderer.SetPosition(1, distance.position2);

            LineRenderer linePathRenderer = linePathObject.GetComponent<LineRenderer>();
            linePathRenderer.positionCount = 2;

            // Update text
            Text txt = textObject.GetComponent<Text>();
            //String str = distance.distance.ToString("F2") + "m" + "{0}";
            
            txt.text = String.Format("{0:0.0}m:{1:0}",distance.distance, GetStringPart(distance.craneIndex));
            targetObject.transform.localPosition = (distance.position1) + new Vector3(0, 0, 2.5f);

            tempPosition1 = distance.position1 * 1 / 2 + distance.position2 * 1 / 2;
            //tempPosition2 = panelObject.gameObject.transform.position;
            tempPosition2 = targetObject.gameObject.transform.localPosition;
            linePathRenderer.SetPosition(0, tempPosition1);
            linePathRenderer.SetPosition(1, tempPosition2);


            // Update panel
            switch (distance.level)
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

            panelObject.SetActive(true);
            lineObject.SetActive(true);
            linePathObject.SetActive(true);
        }
    }
}
