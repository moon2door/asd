using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ProcessScreenOverlay : MonoBehaviour
{
    private static ProcessScreenOverlay _instance = null;
    private List<ScreenOverlay> overlayObjects = new List<ScreenOverlay>();

    void Awake()
    {
        if (null == _instance)
        {
            _instance = this;
            
            //DontDestroyOnLoad(this.gameObject);
        }
        else
        {
            Destroy(this.gameObject);
        }
    }

    public static ProcessScreenOverlay Instance
    {
        get
        {
            if (null == _instance)
            {
                return null;
            }
            else
            {
                return _instance;
            }
        }
    }

    public void AddOverlayObject(ScreenOverlay screenOverlay)
    {
        overlayObjects.Add(screenOverlay);
    }

    public void RemoveOverlayObject(ScreenOverlay screenOverlay)
    {
        overlayObjects.Remove(screenOverlay);
    }

    void Update()
    {
        List<RectTransform> listRects = new List<RectTransform>();

        foreach (ScreenOverlay screenOverlay in overlayObjects)
        {
            screenOverlay.rectTransform.localScale = new Vector3(1, 1, 1);

            GameObject targetObject = screenOverlay.targetObject.transform.parent.gameObject;
            if (targetObject.activeSelf)
            {
                Camera mainCamera = Camera.main;
                if (mainCamera != null)
                {
                    Vector3 position = mainCamera.WorldToScreenPoint(screenOverlay.targetObject.transform.position);
                    if (position.z > 0)
                    {
                        screenOverlay.rectTransform.position = position;
                    }

                    while (IsRectOverlaps(listRects, screenOverlay.rectTransform))
                    {
                        Vector3 newPosition = screenOverlay.rectTransform.position;
                        newPosition.y --;
                        screenOverlay.rectTransform.position = newPosition;
                    }

                    listRects.Add(screenOverlay.rectTransform);
                }
            }
        }
    }
    private bool IsRectOverlaps(RectTransform rectTrans1, RectTransform rectTrans2)
    {
        Rect rect1 = new Rect(rectTrans1.localPosition.x, rectTrans1.localPosition.y, rectTrans1.rect.width, rectTrans1.rect.height);
        Rect rect2 = new Rect(rectTrans2.localPosition.x, rectTrans2.localPosition.y, rectTrans2.rect.width, rectTrans2.rect.height);
        
        return rect1.Overlaps(rect2);
    }

    private bool IsRectOverlaps(List<RectTransform> referenceList, RectTransform compared)
    {
        bool ret = false;
        foreach (RectTransform rect in referenceList)
        {
            ret = IsRectOverlaps(rect, compared);
            if (ret == true) break;
        }

        return ret;
    }
}
