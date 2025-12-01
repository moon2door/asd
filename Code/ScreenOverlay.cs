using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenOverlay : MonoBehaviour
{
    public GameObject targetObject;
    public RectTransform rectTransform { get; private set; }
    private ProcessScreenOverlay manager = ProcessScreenOverlay.Instance;

    // Start is called before the first frame update
    void Awake()
    {
        rectTransform = GetComponent<RectTransform>();

    }
    private void Start()
    {
        manager.AddOverlayObject(this);
        rectTransform.localScale = new Vector3(0, 0);
    }

    void OnDestroy()
    {
        manager.RemoveOverlayObject(this);
    }
}
