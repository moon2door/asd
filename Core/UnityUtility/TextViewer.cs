using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TextViewer : MonoBehaviour
{
    [SerializeField]
    private RectTransform rectTransformContents;

    private bool bUpdateText = false;
    private string text;

    private List<GameObject> textObjects = new List<GameObject>();

    void LateUpdate()
    {
        if(bUpdateText)
        {
            bUpdateText = false;

            for (int i = 0; i < textObjects.Count; i++)
            {
                GameObject.Destroy(textObjects[i].gameObject);
            }
            textObjects.Clear();

            float contentHeight = 0;
            for(int offset = 0; offset < text.Length; offset += 8000)
            {
                string subText = text.Substring(offset, Mathf.Min(text.Length - offset, 8000));

                GameObject gameObject = new GameObject("text");
                gameObject.transform.parent = rectTransformContents.gameObject.transform;

                // Component Text
                Text t = GetComponent<Text>();
                Text textObject = gameObject.AddComponent<Text>();
                textObject.font = t.font;
                textObject.color = t.color;
                textObject.fontSize = t.fontSize;
                textObject.fontStyle = t.fontStyle;
                textObject.supportRichText = t.supportRichText;
                textObject.alignment = t.alignment;
                textObject.alignByGeometry = t.alignByGeometry;
                textObject.horizontalOverflow = t.horizontalOverflow;
                textObject.verticalOverflow = t.verticalOverflow;
                textObject.resizeTextForBestFit = t.resizeTextForBestFit;
                textObject.color = t.color;
                textObject.material = t.material;
                textObject.raycastTarget = t.raycastTarget;
                textObject.raycastPadding = t.raycastPadding;
                textObject.maskable = t.maskable;
                textObject.text = subText;
                textObjects.Add(gameObject);

                // Component RectTransform
                RectTransform rt = gameObject.GetComponent<RectTransform>();
                rt.anchorMin = new Vector2(0f, 1f);
                rt.anchorMax = new Vector2(0f, 1f);
                rt.pivot = new Vector2(0, 1);
                rt.sizeDelta = new Vector2(rectTransformContents.rect.width, 0);
                rt.sizeDelta = new Vector2(rectTransformContents.rect.width, textObject.preferredHeight);

                contentHeight += rt.sizeDelta.y;
            }
            rectTransformContents.sizeDelta = new Vector2(rectTransformContents.sizeDelta.x, contentHeight);
        }
    }

    public void UpdateText(string text)
    {
        bUpdateText = true;
        this.text = text;
    }
    public void Clear()
    {
        this.text = "";
        bUpdateText = true;
    }
}
