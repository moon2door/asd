using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;

using CraneKey = System.Int32;

using UnityEngine.EventSystems;// Required when using Event data.


public class MenuController : MonoBehaviour, IPointerUpHandler
{
    public GameObject panelInfo;
    public PanelLoading panelLoading;
    public DatePicker datePicker;
    public MySocket socketObject;
    public CraneInfo craneInfo;
    public SelectionManager selectionManager;
    public Toggle toggleStatus;
    public GameObject statusMenu;

    public Toggle toggleDetailed;
    public GameObject detailedMenu;
    public GameObject cooperationMenu;
    public GameObject panelAddLoginInfo;
    public GameObject panelLogin;

    public Button buttonLogin;
    public Button buttonCooperation;
    public Button buttonAddLoginInfo;
    public Button buttonLogPlayMode;

    public Text textLoginId;
    public Text textWindSpeed;
    public Text textCooperation;

    private bool bUpdateStatusInfo = true;
    private bool bUpdateTimestamp = true;
    private Dictionary<CraneKey, string> strStatus = new Dictionary<CraneKey, string>();
    private Dictionary<CraneKey, string> strDistance = new Dictionary<CraneKey, string>();
    private Dictionary<CraneKey, string> strComment = new Dictionary<CraneKey, string>();
    private Dictionary<CraneKey, Color> textColors = new Dictionary<CraneKey, Color>();
    private int currentPier;

    private bool bUpdateCooperationText = false;
    private string cooperationText = "";

    private bool bUpdateWindText = false;
    private string windText = "";

    private bool bEnableCooperationText = true;

    private bool bEnableAddUserButton = false;
    private bool bEnableCooperationButton = false;

    private string loginInfoText = "";


    Dictionary<int, bool> cooperationState = new Dictionary<int, bool>();
    private Dictionary<int, StructDefines.StCooperationMessage> listAccept = new Dictionary<int, StructDefines.StCooperationMessage>();
    private bool bUpdateCoopList = false;
    public Text textCooperationList;

    [Header("[로그 재생 게임 오브젝트]")]
    public GameObject panelLogPlay;
    public Button btnOpenLog;
    public Button btnPlayLog;
    public Button btnStopLog;
    public Button btnCloseLog;
    public Slider sliderLog;
    public Text textLog;
    public Text textLog2;

    private bool bUpdateLogInfo = false;
    private DateTime logtimeStart;
    private DateTime logtimeEnd;
    private DateTime logtimeCur;

    public bool bAdminAuthorized = false;
    public string loginID = "user";
    public string loginName = "user";
    public string loginReserved = "";

    // Start is called before the first frame update
    void Start()
    {
        EventTrigger trigger = sliderLog.gameObject.GetComponent<EventTrigger>();
        EventTrigger.Entry entryPointerUp = new EventTrigger.Entry();
        entryPointerUp.eventID = EventTriggerType.PointerUp;
        entryPointerUp.callback.AddListener((data) => { OnPointerUp((PointerEventData)data); });
        trigger.triggers.Add(entryPointerUp);

        EventTrigger.Entry entryPointerDown = new EventTrigger.Entry();
        entryPointerDown.eventID = EventTriggerType.PointerDown;
        entryPointerDown.callback.AddListener((data) => { OnPointerDown((PointerEventData)data); });
        trigger.triggers.Add(entryPointerDown);
        
        datePicker.AddCallback(OnDatePicked);

        toggleStatus.onValueChanged.AddListener(delegate
        {
            ToggleStatusChanged(toggleStatus);
        });

        statusMenu.SetActive(toggleStatus.isOn);

        toggleDetailed.onValueChanged.AddListener(delegate
        {
            ToggleDetailedChanged(toggleDetailed);
        });
        detailedMenu.SetActive(toggleDetailed.isOn);

        buttonLogin.onClick.AddListener(delegate
        {
            OnButtonLogin();
        });

        buttonAddLoginInfo.onClick.AddListener(delegate
        {
            OnButtonAddLoginInfo();
        });

        buttonCooperation.onClick.AddListener(delegate
        {
            OnButtonCooperation();
        });

        buttonLogPlayMode.onClick.AddListener(delegate
        {
            OnButtonLogPlayMode();
        });

        btnOpenLog.onClick.AddListener(delegate
        {
            OnButtonOpenLog();
        });

        btnPlayLog.onClick.AddListener(delegate
        {
            OnButtonPlayLog();
        });

        btnStopLog.onClick.AddListener(delegate
        {
            OnButtonStopLog();
        });

        btnCloseLog.onClick.AddListener(delegate
        {
            OnButtonCloseLog();
        });

        foreach (int key in craneInfo.keys)
        {
            strStatus.Add(key, string.Empty);
            strDistance.Add(key, string.Empty);
            strComment.Add(key, string.Empty);
            textColors.Add(key, Color.black);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(bUpdateLogInfo)
        {
            DateTime start = logtimeStart;
            DateTime end = logtimeEnd;
            DateTime cur = logtimeCur;

            sliderLog.minValue = start.Hour * 60 * 60 + start.Minute * 60 + start.Second;
            sliderLog.maxValue = end.Hour * 60 * 60 + end.Minute * 60 + end.Second;
            sliderLog.value = cur.Hour * 60 * 60 + cur.Minute * 60 + cur.Second;

            textLog.text = string.Format("로그 시간: {0:0000}년 {1:00}월 {2:00}일 {3:00}:{4:00} {5:00}", cur.Year, cur.Month, cur.Day, cur.Hour, cur.Minute, cur.Second);
            textLog2.text = string.Format("{0:0000}-{1:00}-{2:00} {3:00}:{4:00} {5:00} ~ {6:00}:{7:00} {8:00}", start.Year, start.Month, start.Day, start.Hour, start.Minute, start.Second, end.Hour, end.Minute, end.Second);
            bUpdateLogInfo = false;
        }

        if (loginInfoText != textLoginId.text)
        {
            textLoginId.text = loginInfoText;
        }

        if (bUpdateCoopList)
        {
            int keyGC1 = craneInfo.GetCraneKeycode(3, 0);
            int keyGC2 = craneInfo.GetCraneKeycode(3, 1);
            int keyTTC4 = craneInfo.GetCraneKeycode(3, 5);

            string cooperationList = "";
            if (listAccept.ContainsKey(keyGC1) && cooperationState.ContainsKey(keyGC1))
            {
                if(cooperationState[keyGC1] == true)
                {
                    StructDefines.StCooperationMessage msg = listAccept[keyGC1];

                    DateTime startTime = new DateTime(msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin, msg.startSec);
                    DateTime endTime = new DateTime(msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin, msg.endSec);
                    if (startTime <= DateTime.Now && DateTime.Now <= endTime)
                    {
                        cooperationList += string.Format("GC1 협업중({0}.{1}.{2} {3}:{4} ~ {5}.{6}.{7} {8}:{9}) \n",
                            msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin,
                            msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin);
                    }
                }
            }
            if (listAccept.ContainsKey(keyGC2) && cooperationState.ContainsKey(keyGC2))
            {
                if (cooperationState[keyGC2] == true)
                {
                    StructDefines.StCooperationMessage msg = listAccept[keyGC2];

                    DateTime startTime = new DateTime(msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin, msg.startSec);
                    DateTime endTime = new DateTime(msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin, msg.endSec);
                    if (startTime <= DateTime.Now && DateTime.Now <= endTime)
                    {
                        cooperationList += string.Format("GC2 협업중({0}.{1}.{2} {3}:{4} ~ {5}.{6}.{7} {8}:{9}) \n",
                            msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin,
                            msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin);
                    }
                }
            }
            if (listAccept.ContainsKey(keyTTC4) && cooperationState.ContainsKey(keyTTC4))
            {
                if (cooperationState[keyTTC4] == true)
                {
                    StructDefines.StCooperationMessage msg = listAccept[keyTTC4];

                    DateTime startTime = new DateTime(msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin, msg.startSec);
                    DateTime endTime = new DateTime(msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin, msg.endSec);
                    if (startTime <= DateTime.Now && DateTime.Now <= endTime)
                    {
                        cooperationList += string.Format("TTC4 협업중({0}.{1}.{2} {3}:{4} ~ {5}.{6}.{7} {8}:{9}) \n",
                            msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin,
                            msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin);
                    }
                }
            }
            textCooperationList.text = cooperationList;

            bUpdateCoopList = false;
        }

        if (bEnableAddUserButton != buttonAddLoginInfo.gameObject.activeSelf)
        {
            buttonAddLoginInfo.gameObject.SetActive(bEnableAddUserButton);
        }

        if (bEnableCooperationButton != buttonCooperation.gameObject.activeSelf)
        {
            buttonCooperation.gameObject.SetActive(bEnableCooperationButton);
            buttonLogPlayMode.gameObject.SetActive(bEnableCooperationButton);

            Text textLogin = buttonLogin.transform.Find("Text").GetComponent<Text>();
            if (buttonCooperation.gameObject.activeSelf)
            {
                textLogin.text = "로그오프";
            }
            else
            {
                textLogin.text = "로그인";
            }
        }

        if (bUpdateWindText)
        {
            textWindSpeed.text = windText;
            bUpdateWindText = false;
        }

        if (bUpdateCooperationText)
        {
            textCooperation.text = cooperationText;
            textCooperation.gameObject.SetActive(bEnableCooperationText);
            bUpdateCooperationText = false;
        }

        if (currentPier != selectionManager.currentPier)
        {
            currentPier = selectionManager.currentPier;
            bUpdateStatusInfo = true;
            bUpdateTimestamp = true;
        }

        if (craneInfo.cranePierCode.Contains(currentPier))
        {

            if (bUpdateStatusInfo)
            {
                GameObject menu = statusMenu.transform.GetChild(1).gameObject;
                for (int i = 1; i < menu.transform.childCount; i++)
                {
                    GameObject gameObjectCrane = menu.transform.GetChild(i).Find("Crane").gameObject;
                    GameObject gameObjectStatus = menu.transform.GetChild(i).Find("Status").gameObject;
                    GameObject gameObjectDistance = menu.transform.GetChild(i).Find("Distance").gameObject;

                    Text textCrane = gameObjectCrane.GetComponent<Text>();
                    Text textStatus = gameObjectStatus.GetComponent<Text>();
                    Text textDistance = gameObjectDistance.GetComponent<Text>();

                    int key = craneInfo.GetCraneKeycode(currentPier, i - 1);
                    if (craneInfo.dictCraneName.ContainsKey(key) &&
                        strStatus.ContainsKey(key) &&
                        strDistance.ContainsKey(key) &&
                        textColors.ContainsKey(key))
                    {
                        textCrane.text = i.ToString() + " " + craneInfo.dictCraneName[key];
                        textStatus.text = strStatus[key];
                        textDistance.text = strDistance[key];
                        textDistance.color = textColors[key];
                    }
                    else
                    {
                        textCrane.text = string.Empty;
                        textStatus.text = string.Empty;
                        textDistance.text = string.Empty;
                    }
                }
                bUpdateStatusInfo = false;
            }

            if (bUpdateTimestamp)
            {
                GameObject menu = statusMenu.transform.GetChild(1).gameObject;
                for (int i = 1; i < menu.transform.childCount; i++)
                {
                    GameObject gameObjectComment = menu.transform.GetChild(i).Find("Comment").gameObject;
                    Text textComment = gameObjectComment.GetComponent<Text>();

                    //Debug.Log("#####################" + craneInfo.dictCraneName[key] + " " + strComment[key]);
                    int key = craneInfo.GetCraneKeycode(currentPier, i - 1);
                    if (strComment.ContainsKey(key))
                    {
                        //Debug.Log(string.Format("$$$$$$ {0} -- {1} => {2}", currentPier, i - 1, strComment[key]));
                        textComment.text = strComment[key];
                    }
                    else
                    {
                        textComment.text = string.Empty;
                    }
                }
                bUpdateTimestamp = false;
            }
        }
    }

    void ToggleStatusChanged(Toggle changed)
    {
        statusMenu.SetActive(changed.isOn);
    }

    void ToggleDetailedChanged(Toggle changed)
    {
        detailedMenu.SetActive(changed.isOn);
    }

    public void OnButtonCooperation()
    {
        cooperationMenu.SetActive(!cooperationMenu.activeSelf);
    }

    public void OnButtonLogin()
    {
        loginInfoText = "";
        loginID = "user";
        loginName = "user";
        loginReserved = "";

        if (buttonCooperation.gameObject.activeSelf)
        {
            bEnableAddUserButton = false;
            bEnableCooperationButton = false;
            bAdminAuthorized = false;

            CooperationMentControl ctrl = cooperationMenu.GetComponent<CooperationMentControl>();
            ctrl.ResetUpdate();
        }
        else
        {
            panelLogin.SetActive(!panelLogin.activeSelf);
        }
    }

    public void OnReplyLoginAdmin()
    {
        loginInfoText = "Login: admin";
        loginID = "admin";
        loginName = "admin";
        loginReserved = "";

        panelLogin.SetActive(!panelLogin.activeSelf);
        bEnableAddUserButton = true;
        bEnableCooperationButton = true;
        bAdminAuthorized = true;
    }

    public void OnButtonAddLoginInfo()
    {
        panelAddLoginInfo.SetActive(!panelAddLoginInfo.activeSelf);
    }

    public void UpdateStatusInfo(int pier, int crane, string status, string distance, string comment, Color textColor)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);
        strStatus[key] = status;
        strDistance[key] = distance;
        textColors[key] = textColor;
        bUpdateStatusInfo = true;
    }

    public void UpdateTimestamp(int pier, int crane, System.TimeSpan timeSpan)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);
        strComment[key] = string.Format("{0}시간 {1}분", timeSpan.Hours, timeSpan.Minutes);
        bUpdateTimestamp = true;
    }

    public void UpdateCooperationText(string text)
    {
        bUpdateCooperationText = true;
        cooperationText = text;
    }

    public void UpdateWindText(string text)
    {
        bUpdateWindText = true;
        windText = text;
    }

    public void SetEnableCooperationText(bool enable)
    {
        bEnableCooperationText = enable;
        bUpdateCooperationText = true;
    }

    public void OnReplyLogin(StructDefines.StReplyLogin repMessage, string loginId)
    {
        if (repMessage.code == 0)
        {
            bEnableAddUserButton = false;
            bEnableCooperationButton = true;
            loginInfoText = "Login: " + loginId;
            loginID = loginId;
            loginName = "";
            loginReserved = "";

            bAdminAuthorized = false;
        }
        else if (repMessage.code == 2)
        {
            bEnableAddUserButton = false;
            bEnableCooperationButton = true;
            loginInfoText = "Login(Admin): " + loginId;
            loginID = loginId;
            loginName = "";
            loginReserved = "";

            bAdminAuthorized = true;
        }
        else
        {
            bEnableAddUserButton = false;
            bEnableCooperationButton = false;
            loginInfoText = "";
            loginID = "user";
            loginName = "";
            loginReserved = "";

            bAdminAuthorized = false;
        }
    }

    public void UpdateCooperationState(int pier, int crane, bool bCooperation)
    {
        int craneKey = craneInfo.GetCraneKeycode(pier, crane);
        cooperationState[craneKey] = bCooperation;
        bUpdateCoopList = true;
    }

    public void UpdateCooperationList(StructDefines.StCooperationMessage[] list)
    {
        Dictionary<int, StructDefines.StCooperationMessage> messages = new Dictionary<int, StructDefines.StCooperationMessage>();
        foreach (StructDefines.StCooperationMessage msg in list)
        {
            int craneKey = craneInfo.GetCraneKeycode((int)msg.pier, (int)msg.crane);
            DateTime start = new DateTime(msg.startYear, msg.startMonth, msg.startDate, msg.startHour, msg.startMin, 0);
            DateTime end = new DateTime(msg.endYear, msg.endMonth, msg.endDate, msg.endHour, msg.endMin, 0);
            
            if (DateTime.Now < start) continue;

            if (end < DateTime.Now) continue;

            if (msg.cooperationMode != 2) continue;

            if (messages.ContainsKey(craneKey))
            {
                if (messages[craneKey] > msg)
                {
                    messages[craneKey] = msg;
                }
            }
            else
            {
                messages[craneKey] = msg;
            }
        }

        foreach (int craneKey in messages.Keys)
        {
            listAccept[craneKey] = messages[craneKey];
            bUpdateCoopList = true;
        }
    }
    public void UpdateLogPlayInfo(DateTime start, DateTime end, DateTime cur)
    {
        bUpdateLogInfo = true;
        logtimeStart = start;
        logtimeEnd = end;
        logtimeCur = cur;

        panelLoading.SetLoading(false);
    }
    
    public void OnButtonOpenLog()
    {
        datePicker.gameObject.SetActive(true);
    }
    public void OnButtonPlayLog()
    {
        if(2020 < logtimeCur.Year)
        {
            StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
            message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_START;
            message.pier = (uint)selectionManager.currentPier;
            message.playDay = logtimeCur;
            socketObject.SendRequestLogPlay(message);

            panelLoading.gameObject.SetActive(true);
            panelLoading.SetLoading(true);
        }
    }
    public void OnButtonStopLog()
    {
        if (2020 < logtimeCur.Year)
        {
            StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
            message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_PAUSE;
            message.pier = (uint)selectionManager.currentPier;
            message.playDay = DateTime.Today;
            socketObject.SendRequestLogPlay(message);
        }
        panelLoading.gameObject.SetActive(false);
        panelLoading.SetLoading(false);
    }
    public void OnButtonLogPlayMode()
    {
        toggleStatus.isOn = false;
        toggleDetailed.isOn = false;
        panelLogPlay.gameObject.SetActive(true);
        panelInfo.gameObject.SetActive(false);

        StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
        message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_READY;
        message.pier = (uint)selectionManager.currentPier;
        message.playDay = logtimeCur;
        socketObject.SendRequestLogPlay(message);
    }

    public void OnButtonCloseLog()
    {
        panelLogPlay.gameObject.SetActive(false);
        panelInfo.gameObject.SetActive(true);

        StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
        message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_CLOSE;
        message.pier = (uint)selectionManager.currentPier;
        message.playDay = logtimeCur;
        socketObject.SendRequestLogPlay(message);
    }    

    public void OnDatePicked(DateTime dateTime)
    {
        if(panelLogPlay.gameObject.activeSelf)
        {
            logtimeCur = dateTime;

            StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
            message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_OPEN;
            message.pier = (uint)selectionManager.currentPier;
            message.playDay = logtimeCur;
            socketObject.SendRequestLogPlay(message);

            panelLoading.gameObject.SetActive(true);
            panelLoading.SetLoading(true);
        }
    }
    public void OnPointerUp(PointerEventData eventData)
    {
        int value = (int)sliderLog.value;
        TimeSpan dt = new TimeSpan(0, 0, value);
        DateTime selectedTime = new DateTime(logtimeStart.Year, logtimeStart.Month, logtimeStart.Day, 0, 0, 0);
        selectedTime += dt;
        
        if (selectedTime.Year > 2020)
        {
            StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
            message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_START;
            message.pier = (uint)selectionManager.currentPier;
            message.playDay = selectedTime;
            socketObject.SendRequestLogPlay(message);

            panelLoading.gameObject.SetActive(true);
            panelLoading.SetLoading(true);
        }
        else
        {
            sliderLog.value = 0;
        }
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        int value = (int)sliderLog.value;
        TimeSpan dt = new TimeSpan(0, 0, value);
        DateTime selectedTime = new DateTime(logtimeStart.Year, logtimeStart.Month, logtimeStart.Day, 0, 0, 0);
        selectedTime += dt;

        StructDefines.StRequestLogPlay message = new StructDefines.StRequestLogPlay();
        message.code = (int)StructDefines.StRequestLogPlay.AcceptCode.LOGPLAY_PAUSE;
        message.pier = (uint)selectionManager.currentPier;
        message.playDay = selectedTime;
        socketObject.SendRequestLogPlay(message);
    }
}