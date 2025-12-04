using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class craneHookWire : MonoBehaviour
{
    public Material material;
    public List<Transform> position1;
    public List<Transform> position2;
    
    public float wireWidth = 0.1f;
    public float dx;
    public float dy;

    private int numWire;
    private List<Vector3> offset = new List<Vector3>();
    private List<GameObject> listGameObject = new List<GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        offset.Add(new Vector3(-dx, 0, -dy));
        offset.Add(new Vector3(-dx, 0, +dy));
        offset.Add(new Vector3(+dx, 0, -dy));
        offset.Add(new Vector3(+dx, +dy));

        numWire = position1.Count * offset.Count;

        for (int i = 0; i < numWire; i++)
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
        for (int idxPosition = 0; idxPosition < position1.Count; idxPosition++)
        {
            for(int idxOffset = 0; idxOffset < offset.Count; idxOffset++)
            {
                int wireIdx = idxPosition * 4 + idxOffset;

                Transform p1 = position1[idxPosition];
                Transform p2 = position2[idxPosition];

                LineRenderer lr = listGameObject[wireIdx].GetComponent<LineRenderer>();
                lr.widthCurve = AnimationCurve.Linear(0, wireWidth, 1, wireWidth);
                lr.positionCount = 2;
                lr.SetPosition(0, p1.position + offset[idxOffset]);
                lr.SetPosition(1, p2.position + offset[idxOffset]);
                lr.material = material;
            }
        }
    }
}