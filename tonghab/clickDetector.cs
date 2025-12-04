using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class clickDetector : MonoBehaviour, IPointerClickHandler
{
    public int index = 0;
    public SelectionManager selectionManager;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void OnPointerClick(PointerEventData eventData)
    {
        selectionManager.ChangeCraneSelection(index);
    }
}
