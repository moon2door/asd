using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PanelLoading : MonoBehaviour
{
    public GameObject loadingImageObject;
    public GameObject textLoading;
    public float rotationSpeed = 100;
    public bool bLoading = false;

    private float angle = 0;

    public void SetLoading(bool bLoading)
    {
        angle = 0;
        this.bLoading = bLoading;
    }

    void Start()
    {
        angle = 0;
    }

    // Update is called once per frame
    void Update()
    {
        if (bLoading != textLoading.activeSelf)
        {
            textLoading.SetActive(bLoading);
        }
        if (bLoading != loadingImageObject.activeSelf)
        {
            loadingImageObject.SetActive(bLoading);
        }

        if (bLoading)
        {
            float dAngle = Time.deltaTime * rotationSpeed;
            RectTransform rectTransform = loadingImageObject.GetComponent<RectTransform>();
            rectTransform.Rotate(new Vector3(0, 0, dAngle));
        }
    }
}
