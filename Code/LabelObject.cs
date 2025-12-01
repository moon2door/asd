using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LabelObject : MonoBehaviour
{
    private string text;
    private bool bUpdate = false;

    [SerializeField]
    private Text textObject;

    public void UpdateLabel(string text)
    {
        this.text = text;
        bUpdate = true;
    }

    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if(bUpdate)
        {
            textObject.text = text;
            bUpdate = false;
        }
    }
}
