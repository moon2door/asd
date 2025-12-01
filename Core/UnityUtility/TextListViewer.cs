using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TextListViewer : MonoBehaviour
{
    [SerializeField]
    private RectTransform rectTransformContents;


    private List<Text> textObjects = new List<Text>();
    private GameObject selectionBox;

    UnityEngine.Events.UnityAction action;

    bool bUpdateTextList = false;
    List<string> textList = new List<string>();

    bool bUpdateSelected = false;
    public int selection { get; private set; } = -1;

    void Start()
    {
    }
    // Update is called once per frame
    void LateUpdate()
    {
        if (bUpdateTextList)
        {
            for(int i=0; i< textObjects.Count; i++)
            {
                GameObject.Destroy(textObjects[i].gameObject);
            }
            textObjects.Clear();

            float contentHeight = 0;
            for (int i = 0; i < textList.Count; i++)
            {
                GameObject gameObject = new GameObject("text");
                gameObject.transform.parent = rectTransformContents.gameObject.transform;

                // Component Text
                Text text = GetComponent<Text>();
                Text textObject = gameObject.AddComponent<Text>();
                textObject.font = text.font;
                textObject.color = text.color;
                textObject.fontSize = text.fontSize;
                textObject.fontStyle = text.fontStyle;
                textObject.supportRichText = text.supportRichText;
                textObject.alignment = text.alignment;
                textObject.alignByGeometry = text.alignByGeometry;
                textObject.horizontalOverflow = text.horizontalOverflow;
                textObject.verticalOverflow = text.verticalOverflow;
                textObject.resizeTextForBestFit = text.resizeTextForBestFit;
                textObject.color = text.color;
                textObject.material = text.material;
                textObject.raycastTarget = text.raycastTarget;
                textObject.raycastPadding = text.raycastPadding;
                textObject.maskable = text.maskable;
                textObject.text = textList[i];
                textObjects.Add(textObject);

                // Component RectTransform
                RectTransform rt = gameObject.GetComponent<RectTransform>();
                rt.anchorMin = new Vector2(0f, 1f);
                rt.anchorMax = new Vector2(1f, 1f);

                rt.pivot = new Vector2(0f, 1);
                rt.sizeDelta = new Vector2(rectTransformContents.rect.width, textObject.preferredHeight + 5);

                contentHeight += rt.rect.height;

                // Component Button
                Button button = gameObject.AddComponent<Button>();
                int num = i;
                button.onClick.AddListener(delegate { OnClickContent(num); });
            }
            rectTransformContents.sizeDelta = new Vector2(rectTransformContents.sizeDelta.x, contentHeight);
            bUpdateTextList = false;
        }

        if(bUpdateSelected)
        {
            if(selectionBox == null)
            {
                selectionBox = new GameObject("Highlight");
                selectionBox.AddComponent<Image>();
            }

            selectionBox.transform.SetParent(textObjects[selection].gameObject.transform);

            Image image = selectionBox.GetComponent<Image>();
            image.color = new Color(0, 0, 0, 0.5f);

            RectTransform rectRef = textObjects[selection].gameObject.GetComponent<RectTransform>();
            RectTransform rectTransform = selectionBox.GetComponent<RectTransform>();
            rectTransform.offsetMin = rectRef.offsetMin;
            rectTransform.offsetMax = rectTransform.offsetMax;
            rectTransform.pivot = new Vector2(0, 1);
            rectTransform.anchorMin = new Vector2(0f, 1f);
            rectTransform.anchorMax = new Vector2(1f, 1f);
            rectTransform.anchoredPosition = new Vector2(0, 0);
            rectTransform.sizeDelta = rectRef.sizeDelta;

            bUpdateSelected = false;

            if(action != null)
            {
                action.Invoke();
            }
        }
    }

    public void UpdateSelected(int selection)
    {
        bUpdateSelected = true;
        this.selection = selection;
    }

    public void AddListener(UnityEngine.Events.UnityAction action)
    {
        this.action = action;
    }

    public void OnClickContent(int i)
    {
        UpdateSelected(i);
    }

    public void Clear()
    {
        textList.Clear();
        bUpdateTextList = true;
    }

    public string GetText(int i)
    {
        string ret = "";
        if(0 <= i && i < textList.Count)
        {
            ret = textList[i];
        }
        return ret;
    }

    public void UpdateTextList(List<string> textList)
    {
        this.textList = textList;
        bUpdateTextList = true;
    }
}
