using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CraneWireControl : MonoBehaviour
{
    public Material material;
    [Header("Wire Positions")]
    public Transform position1;
    public Transform position2;
    public int numWire;
    public float wireWidth = 0.01f;
    public float width1;
    public float width2;

    private List<GameObject> listGameObject = new List<GameObject>();

    // Start is called before the first frame update
    void Start()
    {        
        for(int i=0; i<numWire; i++)
        {
            GameObject obj = new GameObject("wire");
            obj.AddComponent<LineRenderer>();
            listGameObject.Add(obj);
            obj.transform.SetParent(transform);
        }
    }

    // Update is called once per frame
    void Update()
    {
        for(int i=0; i< listGameObject.Count; i++)
        {
            float offset = (i - (listGameObject.Count - 1) * 0.5f) / (listGameObject.Count - 1) * 0.5f;

            LineRenderer lineRanderer = listGameObject[i].GetComponent<LineRenderer>();
            lineRanderer.widthCurve = AnimationCurve.Linear(0, wireWidth, 1, wireWidth);
            lineRanderer.positionCount = 2;
            lineRanderer.SetPosition(0, position1.position + position1.right * offset * width1);
            lineRanderer.SetPosition(1, position2.position + position2.right * offset * width2);
            lineRanderer.material = material;
        }


    }
}
