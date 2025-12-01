using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public enum OnOffLedStatus
{
    None, On, Off, Ext
}
public class OnOffLed : MonoBehaviour
{
    public Sprite imageOn;
    public Sprite imageOff;
    public Sprite imageExt;
    private Image image;

    public OnOffLedStatus state { get; private set; } = OnOffLedStatus.Off;

    private void Awake()
    {
        image = GetComponent<Image>();
    }

    private void Update()
    {
    }

    public void SetChange(OnOffLedStatus state)
    {
        if(image == null)
            image = GetComponent<Image>();
        if (state == OnOffLedStatus.Off)
        {
            image.sprite = imageOff;
        }
        else if (state == OnOffLedStatus.On)
        {
            image.sprite = imageOn;
        }
        else if (state == OnOffLedStatus.Ext)
        {
            image.sprite = imageExt;
        }
        else
        {
        }
        this.state = state;
    }
}


