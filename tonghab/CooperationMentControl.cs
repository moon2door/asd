using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;

public class CooperationMentControl : MonoBehaviour
{
    public Button buttonClose;

    [Header("[스크립트 참조]")]
    public MenuController menuController;
    public MySocket socketObject;
    public CraneInfo craneInfo;
    public DatePicker datePicker;

    [Header("[게임오브젝트 참조]")]
    public GameObject subPanelStatusGC1;
    public GameObject subPanelStatusGC2;
    public GameObject subPanelStatusTTC4;
    public GameObject subPanelReqList;
    public GameObject subPanelAcceptList;
    [Space(10)]
    public GameObject subPanelError;
    public Text textError;
    public Button buttonCloseErrorPanel;

    [Header("[협업 이력 탭]")]
    public Text textSelectedDay;
    public Button buttonNextDay;
    public Button buttonPrevDay;
    public Button buttonToday;
    public Button buttonSelectDate;
    public Button buttonSaveCsv;

    private bool bUpdateDay = true;
    private DateTime selectedTime = DateTime.Today;

    [Header("[협업 입력 탭]")]
    public Dropdown dropdownCrane;
    [Space(10)]
    public InputField inputStartYear;
    public InputField inputStartMonth;
    public InputField inputStartDay;
    public InputField inputStartHour;
    public InputField inputStartMinute;
    [Space(10)]
    public InputField inputEndYear;
    public InputField inputEndMonth;
    public InputField inputEndDay;
    public InputField inputEndHour;
    public InputField inputEndMinute;
    [Space(10)]
    public Button buttonSetCooperation;
    
    private bool bUpdateCoopList = false;
    private bool bUpdateCoopStatus = false;
    List<StructDefines.StCooperationMessage> listReqCoop = new List<StructDefines.StCooperationMessage>();
    Dictionary<string, List<StructDefines.StCooperationMessage>> listAcceptCoop = new Dictionary<string, List<StructDefines.StCooperationMessage>>();
    Dictionary<string, List<StructDefines.StCooperationMessage>> listRefusedCoop = new Dictionary<string, List<StructDefines.StCooperationMessage>>();
    private Dictionary<int, uint> cooperationState = new Dictionary<int, uint>();


    void Awake()
    {
        datePicker.AddCallback(OnDatePicked);

        buttonClose.onClick.AddListener(delegate
        {
            OnClickButtonClose();
        });

        buttonCloseErrorPanel.onClick.AddListener(delegate
        {
            OnClickButtonCloseErrorPanel();
        });

        DateTime dateTime = DateTime.Now;
        inputStartYear.text = dateTime.Year.ToString();
        inputStartMonth.text = dateTime.Month.ToString();
        inputStartDay.text = dateTime.Day.ToString();
        inputStartHour.text = dateTime.Hour.ToString();
        inputStartMinute.text = dateTime.Minute.ToString();

        TimeSpan t = new TimeSpan(1, 0, 0);
        dateTime += t;

        inputEndYear.text = dateTime.Year.ToString();
        inputEndMonth.text = dateTime.Month.ToString();
        inputEndDay.text = dateTime.Day.ToString();
        inputEndHour.text = dateTime.Hour.ToString();
        inputEndMinute.text = dateTime.Minute.ToString();

        buttonSetCooperation.onClick.AddListener(delegate
        {
            OnClickButtonSetCooperation();
        });

        // 협업 입력 탭
        buttonNextDay.onClick.AddListener(delegate
        {
            OnClickButtonNextDay();
        });
        buttonPrevDay.onClick.AddListener(delegate
        {
            OnClickButtonPrevDay();
        });
        buttonToday.onClick.AddListener(delegate
        {
            OnClickButtonToday();
        });
        buttonSelectDate.onClick.AddListener(delegate
        {
            OnClickButtonSelectDay();
        });
        buttonSaveCsv.onClick.AddListener(delegate
        {
            OnClickButtonExportCsv();
        });

        // Initialize value
        cooperationState.Add(craneInfo.GetCraneKeycode(3, 0), 0);
        cooperationState.Add(craneInfo.GetCraneKeycode(3, 1), 0);
        cooperationState.Add(craneInfo.GetCraneKeycode(3, 5), 0);
    }

    // Update is called once per frame
    void Update()
    {
        if(bUpdateDay)
        {
            textSelectedDay.text = selectedTime.ToShortDateString();
            bUpdateDay = false;
        }

        if(bUpdateCoopStatus)
        {
            int keyGC1 = craneInfo.GetCraneKeycode(3, 0);
            int keyGC2 = craneInfo.GetCraneKeycode(3, 1);
            int keyTTC4 = craneInfo.GetCraneKeycode(3, 5);

            Text textGC1 = subPanelStatusGC1.transform.Find("Status").GetComponent<Text>();
            if(cooperationState.ContainsKey(keyGC1))
            {
                //COOP_INDICATING_NONE = 0x10,
			    //COOP_INDICATING_REQUEST = 0x11,
			    //COOP_INDICATING_REQUEST_ACCEPTED = 0x12,
			    //COOP_INDICATING_ON = 0x13,
                if (cooperationState[keyGC1] == 0x13)
                {
                    textGC1.text = "협업중";
                }
                else if (cooperationState[keyGC1] == 0x12)
                {
                    textGC1.text = "협업 예정";
                }
                else if (cooperationState[keyGC1] == 0x11)
                {
                    textGC1.text = "협업 요청";
                }
                else
                {
                    textGC1.text = "협업 해제";
                }
            }

            Text textGC2 = subPanelStatusGC2.transform.Find("Status").GetComponent<Text>();
            if (cooperationState.ContainsKey(keyGC2))
            {
                if (cooperationState[keyGC2] == 0x13)
                {
                    textGC2.text = "협업중";
                }
                else if (cooperationState[keyGC2] == 0x12)
                {
                    textGC2.text = "협업 예정";
                }
                else if (cooperationState[keyGC2] == 0x11)
                {
                    textGC2.text = "협업 요청";
                }
                else
                {
                    textGC2.text = "협업 해제";
                }
            }

            Text textTTC4 = subPanelStatusTTC4.transform.Find("Status").GetComponent<Text>();
            if (cooperationState.ContainsKey(keyTTC4))
            {
                if (cooperationState[keyTTC4] == 0x13)
                {
                    textTTC4.text = "협업중";
                }
                else if (cooperationState[keyTTC4] == 0x12)
                {
                    textTTC4.text = "협업 예정";
                }
                else if (cooperationState[keyTTC4] == 0x11)
                {
                    textTTC4.text = "협업 요청";
                }
                else
                {
                    textTTC4.text = "협업 해제";
                }
            }
            
            bUpdateCoopStatus = false;
        }

        if(bUpdateCoopList)
        {
            Transform parentReq = subPanelReqList.transform.parent;
            Transform parentAccept = subPanelAcceptList.transform.parent;
            
            if (parentReq.childCount > 2)
            {
                for (int i = 2; i <parentReq.childCount ; i++)
                {
                    GameObject gameObject = parentReq.GetChild(i).gameObject;
                    Destroy(gameObject);
                }
            }

            if (parentAccept.childCount > 1)
            {
                for (int i = 1; i < parentAccept.childCount; i++)
                {
                    GameObject gameObject = parentAccept.GetChild(i).gameObject;
                    Destroy(gameObject);
                }
            }
            int countReq = 0;
            int countAccept = 0;
            for (int i = 0; i < listReqCoop.Count; i++)
            {
                GameObject listItem = Instantiate(subPanelReqList);
                listItem.SetActive(true);
                listItem.transform.SetParent(parentReq);
                Text text1 = listItem.transform.Find("Text1").GetComponent<Text>();
                Text text2 = listItem.transform.Find("Text2").GetComponent<Text>();
                Text text3 = listItem.transform.Find("Text3").GetComponent<Text>();
                int cranekey = craneInfo.GetCraneKeycode((int)listReqCoop[i].pier, (int)listReqCoop[i].crane);
                text1.text = craneInfo.dictPierName[cranekey] + " " + craneInfo.dictCraneName[cranekey];
                text2.text = listReqCoop[i].id;
                text3.text = string.Format("{0}-{1}-{2} {3}:{4} ~ {5}-{6}-{7} {8}:{9}",
                    listReqCoop[i].startYear, listReqCoop[i].startMonth, listReqCoop[i].startDate, listReqCoop[i].startHour, listReqCoop[i].startMin,
                    listReqCoop[i].endYear, listReqCoop[i].endMonth, listReqCoop[i].endDate, listReqCoop[i].endHour, listReqCoop[i].endMin);

                if(menuController.bAdminAuthorized)
                {
                    uint pier = listReqCoop[i].pier;
                    uint crane = listReqCoop[i].crane;
                    int num = countReq;
                    Button button1 = listItem.transform.Find("Button1").GetComponent<Button>();
                    Button button2 = listItem.transform.Find("Button2").GetComponent<Button>();
                    button1.onClick.AddListener(delegate
                    {
                        OnClickButtonListAccept(pier, crane, num);
                    });

                    button2.onClick.AddListener(delegate
                    {
                        OnClickButtonListRefuse(pier, crane, num);
                    });
                    button1.enabled = true;
                    button2.enabled = true;
                }
                else
                {
                    Button button1 = listItem.transform.Find("Button1").GetComponent<Button>();
                    Button button2 = listItem.transform.Find("Button2").GetComponent<Button>();
                    button1.gameObject.SetActive(false);
                    button2.gameObject.SetActive(false);
                }
                countReq++;
            }
            
            string key = selectedTime.ToShortDateString();
            if(listAcceptCoop.ContainsKey(key))
            {
                List<StructDefines.StCooperationMessage> list = listAcceptCoop[key];
                for (int i = 0; i < list.Count; i++)
                {
                    GameObject listItem = Instantiate(subPanelAcceptList);
                    listItem.SetActive(true);
                    listItem.transform.SetParent(parentAccept);
                    Text text1 = listItem.transform.Find("textDate").GetComponent<Text>();
                    Text text2 = listItem.transform.Find("textCrane").GetComponent<Text>();
                    Text text3 = listItem.transform.Find("textDetailed").GetComponent<Text>();
                    Text text4 = listItem.transform.Find("textDetailed2").GetComponent<Text>();
                    int cranekey = craneInfo.GetCraneKeycode((int)list[i].pier, (int)list[i].crane);
                    text1.text = string.Format("{0}.{1} {2}:{3} ~ {4}.{5} {6}:{7}",
                        list[i].startMonth, list[i].startDate, list[i].startHour, list[i].startMin,
                        list[i].endMonth, list[i].endDate, list[i].endHour, list[i].endMin);
                    text2.text = craneInfo.dictPierName[cranekey] + " " + craneInfo.dictCraneName[cranekey];

                    DateTime startTime = new DateTime(list[i].startYear, list[i].startMonth, list[i].startDate, list[i].startHour, list[i].startMin, list[i].startSec);
                    DateTime endTime = new DateTime(list[i].endYear, list[i].endMonth, list[i].endDate, list[i].endHour, list[i].endMin, list[i].endSec);

                    Debug.Log("Now " + DateTime.Now + "Start " + startTime + "End " + endTime);
                    if (DateTime.Now < startTime)
                    {
                        text3.text = "협업 대기";
                    }
                    else if (DateTime.Now < endTime)
                    {
                        text3.text = "협업중";
                    }
                    else
                    {
                        text3.text = "협업 종료";
                    }

                    text4.text = list[i].id;

                    if (menuController.bAdminAuthorized)
                    {
                        uint pier = list[i].pier;
                        uint crane = list[i].crane;
                        int num = countAccept;
                        Button buttonCancle = listItem.transform.Find("Button").GetComponent<Button>();
                        buttonCancle.onClick.AddListener(delegate
                        {
                            OnClickButtonListCencel(pier, crane, num);
                        });
                    }
                    else
                    {
                        Button buttonCancle = listItem.transform.Find("Button").GetComponent<Button>();
                        buttonCancle.gameObject.SetActive(false);
                    }
                    countAccept++;
                }
            }

            if (listRefusedCoop.ContainsKey(key))
            {
                List<StructDefines.StCooperationMessage> list = listRefusedCoop[key];
                for (int i = 0; i < list.Count; i++)
                {
                    GameObject listItem = Instantiate(subPanelAcceptList);
                    listItem.SetActive(true);
                    listItem.transform.SetParent(parentAccept);
                    Text text1 = listItem.transform.Find("textDate").GetComponent<Text>();
                    Text text2 = listItem.transform.Find("textCrane").GetComponent<Text>();
                    Text text3 = listItem.transform.Find("textDetailed").GetComponent<Text>();
                    Text text4 = listItem.transform.Find("textDetailed2").GetComponent<Text>();
                    int cranekey = craneInfo.GetCraneKeycode((int)list[i].pier, (int)list[i].crane);
                    text1.text = string.Format("{0}.{1} {2}:{3} ~ {4}.{5} {6}:{7}",
                        list[i].startMonth, list[i].startDate, list[i].startHour, list[i].startMin,
                        list[i].endMonth, list[i].endDate, list[i].endHour, list[i].endMin);
                    text2.text = craneInfo.dictPierName[cranekey] + " " + craneInfo.dictCraneName[cranekey];
                    text3.text = "협업 취소";
                    text4.text = list[i].id;

                    Button buttonCancle = listItem.transform.Find("Button").GetComponent<Button>();
                    buttonCancle.gameObject.SetActive(false);

                    countAccept++;
                }
            }

            // Resize
            RectTransform rectTransform1 = parentReq.parent.GetComponent<RectTransform>();
            Vector2 size1 = rectTransform1.sizeDelta;
            size1.y = 50 + 40 * countReq;
            rectTransform1.sizeDelta = size1;

            RectTransform rectTransform2 = parentAccept.parent.GetComponent<RectTransform>();
            Vector2 size2 = rectTransform2.sizeDelta;
            size2.y = 100 + 40 * countAccept;
            rectTransform2.sizeDelta = size2;

            bUpdateCoopList = false;
        }
    }
    void OnDisable()
    {
    }

    void OnEnable()
    {
        StructDefines.StRequestCooperationList reqMessage = new StructDefines.StRequestCooperationList(3, 1, (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST, selectedTime.Year, selectedTime.Month, selectedTime.Day);
        socketObject.SendRequestCooperationList(reqMessage);
    }

    public void UpdateCooperationList(StructDefines.StCooperationMessage[] list)
    {
        List<StructDefines.StCooperationMessage> listReqCoop = new List<StructDefines.StCooperationMessage>();
        List<StructDefines.StCooperationMessage> listAcceptCoop = new List<StructDefines.StCooperationMessage>();
        List<StructDefines.StCooperationMessage> listRefusedCoop = new List<StructDefines.StCooperationMessage>();
        string keyAccepted = selectedTime.ToShortDateString();
        string keyRefused = selectedTime.ToShortDateString();

        foreach (StructDefines.StCooperationMessage message in list)
        {
            if (message.cooperationMode == 1)
            {
                listReqCoop.Add(message);
            }
            else if (message.cooperationMode == 2)
            {
                listAcceptCoop.Add(message);

                DateTime dateTime = new DateTime(message.startYear, message.startMonth, message.startDate);
                keyAccepted = dateTime.ToShortDateString();
            }
            else if(message.cooperationMode == 3)
            {
                listRefusedCoop.Add(message);

                DateTime dateTime = new DateTime(message.startYear, message.startMonth, message.startDate);
                keyRefused = dateTime.ToShortDateString();
            }
            else
            {
            }
        }
        
        if (CheckListEquals(this.listReqCoop, listReqCoop))
        {
        }
        else
        {
            this.listReqCoop = listReqCoop;
            bUpdateCoopList = true;
        }

        if (this.listAcceptCoop.ContainsKey(keyAccepted))
        {
            if (CheckListEquals(this.listAcceptCoop[keyAccepted], listAcceptCoop))
            {
            }
            else
            {
                this.listAcceptCoop[keyAccepted] = listAcceptCoop;
                bUpdateCoopList = true;
            }
        }
        else
        {
            this.listAcceptCoop[keyAccepted] = listAcceptCoop;
            bUpdateCoopList = true;
        }

        if (this.listRefusedCoop.ContainsKey(keyRefused))
        {
            if (CheckListEquals(this.listRefusedCoop[keyRefused], listRefusedCoop))
            {
            }
            else
            {
                this.listRefusedCoop[keyRefused] = listRefusedCoop;
                bUpdateCoopList = true;
            }
        }
        else
        {
            this.listRefusedCoop[keyRefused] = listRefusedCoop;
            bUpdateCoopList = true;
        }
    }

    public void UpdateCooperationState(int pierId, int craneId, uint mode)
    {
        int key = craneInfo.GetCraneKeycode(pierId, craneId);
        if (cooperationState.ContainsKey(key))
        {
            cooperationState[key] = mode;
            bUpdateCoopStatus = true;
        }
    }

    public void ResetUpdate()
    {
        bUpdateCoopList = true;
        bUpdateCoopStatus = true;
    }

    private void OnClickButtonClose()
    {
        menuController.OnButtonCooperation();
    }

    private void OnClickButtonCloseErrorPanel()
    {
        HidePanelError();
    }

    private void OnClickButtonSetCooperation()
    {
        int pier = 3;
        int crane = dropdownCrane.value;
        if(dropdownCrane.value == 2)
        {
            crane = 5;
        }
        
        StructDefines.StCooperationMessage message = new StructDefines.StCooperationMessage();
        message.pier = (uint)pier;
        message.crane = (uint)crane;
        
        message.cooperationMode = (int)StructDefines.StCooperationMessage.Cooperation.COOP_REQUEST;
        if (menuController.bAdminAuthorized)
        {
            message.cooperationMode = (int)StructDefines.StCooperationMessage.Cooperation.COOP_ON;
        }

        message.startYear = int.Parse(inputStartYear.text);
        message.startMonth = int.Parse(inputStartMonth.text);
        message.startDate = int.Parse(inputStartDay.text);
        message.startHour = int.Parse(inputStartHour.text);
        message.startMin = int.Parse(inputStartMinute.text);
        message.startSec = 0;
        message.endYear = int.Parse(inputEndYear.text);
        message.endMonth = int.Parse(inputEndMonth.text);
        message.endDate = int.Parse(inputEndDay.text);
        message.endHour = int.Parse(inputEndHour.text);
        message.endMin = int.Parse(inputEndMinute.text);
        message.endSec = 0;

        message.authority = 2;
        message.id = menuController.loginID;
        message.name = menuController.loginName;
        message.reserved = menuController.loginReserved;


        DateTime startTime = new DateTime(message.startYear, message.startMonth, message.startDate, message.startHour, message.startMin, 0);
        DateTime endTime = new DateTime(message.endYear, message.endMonth, message.endDate, message.endHour, message.endMin, 0);
        if(startTime > endTime)
        {
            ShowPanelError("시작 시간이 종료 시간보다 빨라야 합니다.", 5);
        }
        else if(DateTime.Now > endTime)
        {
            ShowPanelError("지난 일정은 추가할 수 없습니다.", 5);
        }
        else
        {
            socketObject.SendCooperation(message);

            Invoke("InvokeRequestCooperationList", 0.1f);
        }

    }

    private void ShowPanelError(string text, float sec)
    {
        textError.text = text;
        subPanelError.SetActive(true);
        Invoke("HidePanelError", sec);
    }
    private void HidePanelError()
    {
        subPanelError.SetActive(false);
        CancelInvoke("HidePanelError");
    }

    void InvokeRequestCooperationList()
    {
        StructDefines.StRequestCooperationList reqMessage = new StructDefines.StRequestCooperationList(3, 1, (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST, selectedTime.Year, selectedTime.Month, selectedTime.Day);
        socketObject.SendRequestCooperationList(reqMessage);
    }

    private void OnClickButtonListAccept(uint pier, uint crane, int idx)
    {
        Debug.Log("OnClickButtonListAccept" + idx);
        StructDefines.StCooperationAccept message = new StructDefines.StCooperationAccept(pier, crane, (uint)idx, (uint)StructDefines.StCooperationAccept.AcceptCode.COOP_ACCEPT, menuController.loginID);
        socketObject.SendCooperationAccept(message);

        Invoke("InvokeRequestCooperationList", 0.1f);
    }
    private void OnClickButtonListRefuse(uint pier, uint crane, int idx)
    {
        Debug.Log("OnClickButtonListRefuse" + idx);
        StructDefines.StCooperationAccept message = new StructDefines.StCooperationAccept(pier, crane, (uint)idx, (uint)StructDefines.StCooperationAccept.AcceptCode.COOP_REFUSE, menuController.loginID);
        socketObject.SendCooperationAccept(message);

        Invoke("InvokeRequestCooperationList", 0.1f);
    }

    private void OnClickButtonListCencel(uint pier, uint crane, int idx)
    {
        Debug.Log("OnClickButtonListCancel" + selectedTime.ToString() + idx);
        StructDefines.StCooperationRemove message = new StructDefines.StCooperationRemove(pier, crane, (uint)selectedTime.Year, (uint)selectedTime.Month, (uint)selectedTime.Day, (uint)idx, menuController.loginID);
        socketObject.SendCooperationRemove(message);

        Invoke("InvokeRequestCooperationList", 0.1f);
    }

    private void OnClickButtonNextDay()
    {
        selectedTime = selectedTime.AddDays(1);
        bUpdateDay = true;
        bUpdateCoopList = true;

        StructDefines.StRequestCooperationList reqMessage = new StructDefines.StRequestCooperationList(3, 1, (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST, selectedTime.Year, selectedTime.Month, selectedTime.Day);
        socketObject.SendRequestCooperationList(reqMessage);
    }

    private void OnClickButtonPrevDay()
    {
        selectedTime = selectedTime.AddDays(-1);
        bUpdateDay = true;
        bUpdateCoopList = true;

        StructDefines.StRequestCooperationList reqMessage = new StructDefines.StRequestCooperationList(3, 1, (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST, selectedTime.Year, selectedTime.Month, selectedTime.Day);
        socketObject.SendRequestCooperationList(reqMessage);
    }

    private void OnClickButtonToday()
    {
        selectedTime = DateTime.Today;
        bUpdateDay = true;
        bUpdateCoopList = true;
        
        StructDefines.StRequestCooperationList reqMessage = new StructDefines.StRequestCooperationList(3, 1, (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST, selectedTime.Year, selectedTime.Month, selectedTime.Day);
        socketObject.SendRequestCooperationList(reqMessage);
    }

    private void OnClickButtonExportCsv()
    {

    }

    private void OnClickButtonSelectDay()
    {
        datePicker.gameObject.SetActive(true);
    }

    public void OnDatePicked(DateTime dateTime)
    {
        selectedTime = dateTime;
        bUpdateDay = true;
        bUpdateCoopList = true;

        StructDefines.StRequestCooperationList reqMessage = new StructDefines.StRequestCooperationList(3, 1, (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST, selectedTime.Year, selectedTime.Month, selectedTime.Day);
        socketObject.SendRequestCooperationList(reqMessage);
    }

    private bool CheckListEquals(List<StructDefines.StCooperationMessage> value, List<StructDefines.StCooperationMessage> tempValue)
    {
        bool isEqual = true;

        if (object.ReferenceEquals(value, tempValue))
        {
            //같은 인스턴스면 true
            isEqual = true;
        }
        else if (value == null || tempValue == null
            || value.Count != tempValue.Count)
        {
            //어느 한 쪽이 null이거나, 요소의 수가 다를 때는 false
            isEqual = false;
        }
        else
        {
            //1개 1개씩 요소 비교
            for (int i = 0; i < value.Count; i++)
            {
                //ary1의 요소의 Equals메소드에서, ary2의 요소와 같은지를 비교
                if (!value[i].Equals(tempValue[i]))
                {
                    //1개라도 같지 않은 요소가 있으면 false
                    isEqual = false;
                    break;
                }
            }
        }
        return isEqual;
    }
}


