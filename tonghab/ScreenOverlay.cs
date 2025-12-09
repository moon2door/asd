using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenOverlay : MonoBehaviour
{
    public GameObject targetObject;
    private RectTransform rectTransform;
    public bool preventOverlapping = false;

    // Start is called before the first frame update
    void Start()
    {
        rectTransform = GetComponent<RectTransform>();
    }

    // Update is called once per frame
    void Update()
    {
        GameObject targetCrane = targetObject.transform.parent.gameObject;
        if (targetCrane.activeSelf)
        {
            Vector3 position = Camera.main.WorldToScreenPoint(targetObject.transform.position);
            if(position.z > 0)
            {
                rectTransform.position = position;
            }
            else
            {
                rectTransform.position = Vector3.one * 65536;
            }
        }
        else
        {
            rectTransform.position = Vector3.one * 65536;
        }

        // preventOverlapping
        if (preventOverlapping)
        {
            GameObject[] screenOverlayObjects = GameObject.FindGameObjectsWithTag("ScreenOverlay");
            foreach (GameObject screenOverlayObject in screenOverlayObjects)
            {
                int myId = GetInstanceID();
                int otherId = screenOverlayObject.GetInstanceID();

                RectTransform otherTransform = screenOverlayObject.GetComponent<RectTransform>();

                if (rectOverlaps(rectTransform, otherTransform) && myId > otherId)
                {
                    rectTransform.position = rectTransform.position + Vector3.down * rectTransform.sizeDelta.y;
                    break;
                }
            }
        }
    }
    private bool rectOverlaps(RectTransform rectTrans1, RectTransform rectTrans2)
    {
        Rect rect1 = new Rect(rectTrans1.localPosition.x, rectTrans1.localPosition.y, rectTrans1.rect.width, rectTrans1.rect.height);
        Rect rect2 = new Rect(rectTrans2.localPosition.x, rectTrans2.localPosition.y, rectTrans2.rect.width, rectTrans2.rect.height);

        return rect1.Overlaps(rect2);
    }
}
