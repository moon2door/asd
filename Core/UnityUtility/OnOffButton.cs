using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;

public enum OnOffButtonStatus
{
    None, On, Off, Flickering
}
public class OnOffButton : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    public Sprite imageOn;
    public Sprite imageOff;
    public OnOffButtonStatus state { get; private set; } = OnOffButtonStatus.Off;

    private float tm = 0;
    public float mouseDownTime = 5;

    public ButtonLongClickedEvent onLongClick { get; set; } = new ButtonLongClickedEvent();
    public ButtonClickedEvent onClick { get; set; } = new ButtonClickedEvent();

    private Image image;
    private Image Image { get { if (image != null) return image; image = GetComponent<Image>(); return image; } }

    private void Awake()
    {
        image = GetComponent<Image>();
    }
    void IPointerDownHandler.OnPointerDown(PointerEventData eventData)
    {
        tm = Time.time;
    }

    void IPointerUpHandler.OnPointerUp(PointerEventData eventData)
    {
        if (tm > 0 && tm + 0.5f > Time.time)
        {
            onClick.Invoke();
        }
        tm = 0;
    }

    public void AddListener(UnityEngine.Events.UnityAction call)
    {
        onClick.AddListener(call);
    }

    private void Update()
    {
        if (state == OnOffButtonStatus.Flickering)
        {
            if ((int)(UnityEngine.Time.fixedTime * 1000) % 1000 < 500)
            {
                Image.sprite = imageOn;
            }
            else
            {
                Image.sprite = imageOff;
            }
        }

        if (tm > 0 && tm + mouseDownTime < Time.time)
        {
            onLongClick.Invoke();
            tm = 0;
        }
    }

    public void SetChange(OnOffButtonStatus state)
    {
        if (state == OnOffButtonStatus.Off)
        {
            Image.sprite = imageOff;
        }
        else if (state == OnOffButtonStatus.On)
        {
            Image.sprite = imageOn;
        }
        else
        {
        }
        this.state = state;
    }

    public void SetSprite(Sprite sprite)
    {
        Image.sprite = sprite;
    }

    public class ButtonClickedEvent : UnityEvent
    {
        public ButtonClickedEvent()
        {
        }
    }
    public class ButtonLongClickedEvent : UnityEvent
    {
        public ButtonLongClickedEvent()
        {
        }
    }
}


