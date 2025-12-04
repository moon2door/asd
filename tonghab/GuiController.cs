using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GuiController : MonoBehaviour
{
    public float val = 0.005f;
    public GameObject cam;
    private GameObject[] objects;

    // Start is called before the first frame update
    void Start()
    {
        objects = GameObject.FindGameObjectsWithTag("2D Object");
    }

    // Update is called once per frame
    void Update()
    {
        objects = GameObject.FindGameObjectsWithTag("2D Object");
        if (0 < objects.Length)
        {
            foreach(GameObject obj in objects)
            {
                Transform tf = obj.transform;
                tf.LookAt(cam.transform.position);
                
                obj.transform.rotation = tf.transform.rotation;
                obj.transform.position = tf.transform.position;
            }
        }
    }
}
