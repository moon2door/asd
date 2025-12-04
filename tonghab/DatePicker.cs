using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;

public class DatePicker : MonoBehaviour
{
    private RectTransform rectTransform;
    public delegate void Callback(DateTime date);
    private List<Callback> callbackList = new List<Callback>();
    public InputField inputYear;
    public InputField inputMonth;
    public InputField inputday;
    public Button buttonOk;
    public Button buttonCancle;
    public Button buttonToday;

    public Button buttonIncreaseYear;
    public Button buttonIncreaseMonth;
    public Button buttonIncreaseDay;
    public Button buttonDecreaseYear;
    public Button buttonDecreaseMonth;
    public Button buttonDecreaseDay;

    public void AddCallback(Callback callback)
    {
        callbackList.Add(callback);
    }

    // Start is called before the first frame update
    void Start()
    {
        rectTransform = GetComponent<RectTransform>();

        buttonOk.onClick.AddListener(delegate
        {
            OnClickButtonOk();
        });

        buttonCancle.onClick.AddListener(delegate
        {
            OnClickButtonCancle();
        });

        buttonToday.onClick.AddListener(delegate
        {
            OnClickButtonToday();
        });

        buttonIncreaseYear.onClick.AddListener(delegate
        {
            int year = int.Parse(inputYear.text);
            year = Mathf.Clamp(++year, 2020, 2050);
            inputYear.text = year.ToString();
        });

        buttonIncreaseMonth.onClick.AddListener(delegate
        {
            int month = int.Parse(inputMonth.text);
            month = Mathf.Clamp(++month, 1, 12);
            inputMonth.text = month.ToString();
        });

        buttonIncreaseDay.onClick.AddListener(delegate
        {
            int day = int.Parse(inputday.text);
            day = Mathf.Clamp(++day, 1, 31);
            inputday.text = day.ToString();
        });

        buttonDecreaseYear.onClick.AddListener(delegate
        {
            int year = int.Parse(inputYear.text);
            year = Mathf.Clamp(--year, 2020, 2050);
            inputYear.text = year.ToString();
        });

        buttonDecreaseMonth.onClick.AddListener(delegate
        {
            int month = int.Parse(inputMonth.text);
            month = Mathf.Clamp(--month, 1, 12);
            inputMonth.text = month.ToString();
        });

        buttonDecreaseDay.onClick.AddListener(delegate
        {
            int day = int.Parse(inputday.text);
            day = Mathf.Clamp(--day, 1, 31);
            inputday.text = day.ToString();
        });

        System.DateTime dayTime = System.DateTime.Today;
        inputYear.text = dayTime.Year.ToString();
        inputMonth.text = dayTime.Month.ToString();
        inputday.text = dayTime.Day.ToString();
    }

    private void OnClickButtonOk()
    {
        int year = int.Parse(inputYear.text);
        int month = int.Parse(inputMonth.text);
        int day = int.Parse(inputday.text);
        DateTime date = new DateTime(year, month, day);
        
        for (int i=0; i< callbackList.Count; i++)
        {
            callbackList[i](date);
        }

        gameObject.SetActive(false);
    }

    private void OnClickButtonCancle()
    {
        gameObject.SetActive(false);
    }

    private void OnClickButtonToday()
    {
        System.DateTime dayTime = System.DateTime.Today;
        inputYear.text = dayTime.Year.ToString();
        inputMonth.text = dayTime.Month.ToString();
        inputday.text = dayTime.Day.ToString();
    }

    // Update is called once per frame
    void Update()
    {
        if( Input.GetMouseButton(0) )
        {
            Vector3 mousePosition = Input.mousePosition;
            if( !RectTransformUtility.RectangleContainsScreenPoint(rectTransform, new Vector2(mousePosition.x, mousePosition.y)) )
            {
                gameObject.SetActive(false);
            }
        }
    }
}
