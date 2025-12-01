using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProcessLabel : MonoBehaviour
{
    [SerializeField]
    public GameObject plane;
    [SerializeField]
    public GameObject prefabPinpoint;
    [SerializeField]
    public ProcessInterface processInterface;

    public bool bEnableAddPinpoint { get; private set; } = false;


    private bool bUpdatePoint = false;
    private List<Vector3> listPinpoints = new List<Vector3>();

    private List<GameObject> listPinpointObjects = new List<GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(bEnableAddPinpoint)
        {
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                RaycastHit hit;
                Collider collider = plane.GetComponent<Collider>();
                if (collider.Raycast(ray, out hit, 10000))
                {
                    listPinpoints.Add(hit.point);

                    CsCore.Data.CollisionProcessor.StMonitoringLabelInfo info = new CsCore.Data.CollisionProcessor.StMonitoringLabelInfo();
                    info.info = "";
                    info.xyz = new float[listPinpoints.Count * 3];
                    for(int i=0; i< listPinpoints.Count; i++)
                    {
                        info.xyz[i*3 + 0] = listPinpoints[i].x;
                        info.xyz[i*3 + 1] = listPinpoints[i].y;
                        info.xyz[i*3 + 2] = listPinpoints[i].z;
                    }
                    processInterface.SendMonitoringLabelInfo(info);
                }
            }
        }

        if (bUpdatePoint)
        {
            if(listPinpoints.Count != listPinpointObjects.Count)
            {
                for (int i = 0; i < listPinpointObjects.Count; i++)
                {
                    GameObject.Destroy(listPinpointObjects[i]);
                }
                listPinpointObjects.Clear();

                for (int i = 0; i < listPinpoints.Count; i++)
                {
                    GameObject pinpoint = GameObject.Instantiate(prefabPinpoint);
                    pinpoint.transform.position = listPinpoints[i];
                    listPinpointObjects.Add(pinpoint);

                    LabelObject label = pinpoint.GetComponent<LabelObject>();
                    label.UpdateLabel("#P" + i);
                }

                LineRenderer lineRenderer = GetComponent<LineRenderer>();
                lineRenderer.positionCount = listPinpoints.Count;
                for (int i = 0; i < listPinpoints.Count; i++)
                {
                    lineRenderer.SetPosition(i, listPinpoints[i]);
                }
            }

            bUpdatePoint = false;
        }
    }

    public void AddPinpoints(List<Vector3> points)
    {
        listPinpoints = points;
        bUpdatePoint = true;
    }

    public void EnableUserAdd(bool bEnable)
    {
        bEnableAddPinpoint = bEnable;
    }

    public void ClearLabel()
    {
        CsCore.Data.CollisionProcessor.StMonitoringLabelInfo info = new CsCore.Data.CollisionProcessor.StMonitoringLabelInfo();
        info.info = "";
        info.xyz = new float[0];
        processInterface.SendMonitoringLabelInfo(info);
    }
}
