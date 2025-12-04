using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using SimpleFileBrowser;
using System.Text;

using YearKey = System.Int32;
using CraneKey = System.Int32;

class ErrorText
{
    public static string[] text =
    {
        "0x00(정상)",
        "0x01(제어 udp bind 실패)",
        "0x02(ZeroSet 응답 없음)",
        "0x03(구동시작 응답 없음)",
        "0x04(rpm변경 응답 없음)",
        "0x05(udp bind 실패)",
        "0x06(udp 수신 안됨)",
        "0x07(udp 수신률 비정상)",
        "0x08(BIT에서 1개이상 비정상)",
        "0x09(센서동기화 비정상)",
        "0x0A(제어기간 통신 불능)",
        "0x0B(거리성능 저하)"
    };
}

public class DetailedMenuControl : MonoBehaviour
{
    public CraneInfo craneInfo;
    public SelectionManager selectManager;
    public MenuController menuController;

    public Button buttonClose;
    public List<Button> buttonSelectCrane;
    public List<Button> buttonDatePick;
    public List<Button> buttonTodayPick;
    public GameObject datePickerPanel;
    public Text textPier;
    public Text textCrane;
    public Text textDriver;
    public Text textDriverInfo;
    public Text TextLabelDate1;
    public BarCumulativeGraph graphCollisionHistoryDaily;
    public Text TextLabelDate2;
    public OperationtimeGraph graphOperationTime;
    public Text TextLabelMonth1;
    public BarCumulativeGraph graphCollisionHistoryMonthly;
    private DateTime today;
    private bool bUpdateGraph;
    private bool bUpdateInfo;
    private Dictionary<CraneKey, Dictionary<YearKey, CollisionHistory>> collisionHistory = new Dictionary<CraneKey, Dictionary<YearKey, CollisionHistory>>();
    private Dictionary<CraneKey, Dictionary<YearKey, OperationHistory>> operationHistory = new Dictionary<CraneKey, Dictionary<YearKey, OperationHistory>>();
    public Text textStartTime;
    public Text textLastTime;
    public List<GameObject> sensorStatusPanel;
    public List<Text> textRotorFps;
    public List<Text> textRotorRpm;
    public List<Text> textLidarFps;
    public List<Text> textErrorCode;

    public List<Button> buttonPrevMonth;
    public List<Button> buttonNextMonth;
    public List<Button> buttonPrevDay;
    public List<Button> buttonNextDay;
    public Button buttonExportCsvDaily;
    public Button buttonExportCsvMonthly;
    public Button buttonExportCsvOperationTime;
    public Text textTotalOperationTime;

    public Text textWindSpeed;
    public Text textWindDirection;
    public Dictionary<int, float> windSpeed = new Dictionary<int, float>();
    private Dictionary<int, float> windDirection = new Dictionary<int, float>();
    private bool bUpdateWindInfo = false;

    [Header("[협업 탭]")]
    public Text cooperationText1;
    public Text cooperationText2;
    private Dictionary<CraneKey, string> cooperationTexts1 = new Dictionary<CraneKey, string>();
    private Dictionary<CraneKey, string> cooperationTexts2 = new Dictionary<CraneKey, string>();
    private bool bUpdateCooperation;

    [Header("[PLC 정보 탭]")]
    public GameObject plcPanel;
    public GameObject plcTitlePanel;
    public Text Hoist1DecelControl;
    public Text Hoist2DecelControl;
    public Text Hoist3DecelControl;
    public Text Trolley1DecelControl;
    public Text Trolley2DecelControl;
    public Text GantryDecelControl;
    public Text AuxHoistDecelControl;
    public Text SlewingDecelControl;
    [Space(10)]
    public Text textTtcSlewing;
    public Text textTtcSlewingInv;
    public Text bUserEnableSwich;
    public Text bGantryCommandFoward;
    public Text bGantryCommandReverse;
    public Text bTrolley1CommandRight;
    public Text bTrolley1CommandLeft;
    public Text bTrolley2CommandRight;
    public Text bTrolley2CommandLeft;
    public Text bHoist1CommandUp;
    public Text bHoist1CommandDown;
    public Text bHoist2CommandUp;
    public Text bHoist2CommandDown;
    public Text bHoist3CommandUp;
    public Text bHoist3CommandDown;
    public Text bAuxHoistCommandUp;
    public Text bAuxHoistCommandDown;
    [Space(10)]
    public Text Trolley1Position;
    public Text Trolley2Position;
    public Text GoliathPosition1;
    public Text GoliathPosition2;
    public Text Hoist1Position;
    public Text Hoist2Position;
    public Text Hoist3Position;
    public Text Hoist1Weight;
    public Text Hoist2Weight;
    public Text Hoist3Weight;
    public Text AuxHoistWeight;
    [Space(10)]
    public Color controlColor;

    private bool bUpdatePlcInfo;

    private int currentCrane;
    private int currentPier;
    private Dictionary<CraneKey, string> opertorName = new Dictionary<CraneKey, string>();
    private Dictionary<CraneKey, string> opertorInfo = new Dictionary<CraneKey, string>();
    private Dictionary<CraneKey, int[]> rotorFps = new Dictionary<YearKey, YearKey[]>();
    private Dictionary<CraneKey, int[]> rotorRpm = new Dictionary<YearKey, YearKey[]>();
    private Dictionary<CraneKey, int[]> lidarFps = new Dictionary<YearKey, YearKey[]>();
    private Dictionary<CraneKey, int[]> errorCode = new Dictionary<YearKey, YearKey[]>();
    private Dictionary<CraneKey, StructDefines.StPlcInfoMessage> plcUpdateMessage = new Dictionary<CraneKey, StructDefines.StPlcInfoMessage>();
    // Start is called before the first frame update
    void Start()
    {
        bUpdateGraph = true;
        bUpdateInfo = true;
        bUpdatePlcInfo = true;
        bUpdateCooperation = true;
        today = DateTime.Now;

        foreach (CraneKey key in craneInfo.keys)
        {
            uint pier = (uint)craneInfo.dictPier[key];
            uint crane = (uint)craneInfo.dictCrane[key];

            // Initialize collision history
            collisionHistory.Add(key, new Dictionary<YearKey, CollisionHistory>());
            for (int dy = 0; dy < 5; dy++)
            {
                int year = DateTime.Now.Year - dy;
                CollisionHistory history = new CollisionHistory(0, 0, 0, new CollisionHistoryDaily[366], new CollisionHistoryMonthly[12]);
                for (int i = 0; i < 366; i++)
                {
                    history.dayly[i] = new CollisionHistoryDaily();
                }
                for (int i = 0; i < 12; i++)
                {
                    history.monthly[i] = new CollisionHistoryMonthly();
                }

                collisionHistory[key].Add(year, history);
            }

            // Initialize operation history
            operationHistory.Add(key, new Dictionary<YearKey, OperationHistory>());
            for (int dy = 0; dy < 5; dy++)
            {
                int year = DateTime.Now.Year - dy;
                OperationHistory history = new OperationHistory(0, 0, 0, new OperationHistoryDaily[366]);
                for (int j = 0; j < history.daily.Length; j++)
                {
                    history.daily[j] = new OperationHistoryDaily();
                }

                operationHistory[key].Add(year, history);
            }

            opertorName.Add(key, string.Empty);
            opertorInfo.Add(key, string.Empty);
            
            int numSensors = craneInfo.dictNumSensors[key];
            rotorFps.Add(key, new int[numSensors]);
            rotorRpm.Add(key, new int[numSensors]);
            lidarFps.Add(key, new int[numSensors]);
            errorCode.Add(key, new int[numSensors]);
            for (int i = 0; i < numSensors; i++)
            {
                rotorFps[key][i] = -1;
                rotorRpm[key][i] = -1;
                lidarFps[key][i] = -1;
                errorCode[key][i] = -1;
            }

            plcUpdateMessage.Add(key, new StructDefines.StPlcInfoMessage(pier, crane));

            cooperationTexts1.Add(key, "");
            cooperationTexts2.Add(key, "");

            windSpeed.Add(key, 0);
            windDirection.Add(key, 0);
        }

        currentPier = selectManager.currentPier;
        currentCrane = selectManager.currentCrane;

        buttonClose.onClick.AddListener(delegate
        {
            OnClickButtonClose();
        });

        for (int i = 0; i < buttonSelectCrane.Count; i++)
        {
            int num = i;
            buttonSelectCrane[i].onClick.AddListener(delegate
            {
                OnClickButtonSelectCrane(num);
            });
        }

        buttonExportCsvDaily.onClick.AddListener(delegate
        {
            OnClickButtonExportDailyCsv();
        });

        buttonExportCsvMonthly.onClick.AddListener(delegate
        {
            OnClickButtonExportMonthlyCsv();
        });

        buttonExportCsvOperationTime.onClick.AddListener(delegate
        {
            OnClickButtonExportOperationTimeCsv();
        });

        for (int i = 0; i < buttonDatePick.Count; i++)
        {
            buttonDatePick[i].onClick.AddListener(delegate
            {
                OnClickButtonDatePicker();
            });
        }

        DatePicker datePicker = datePickerPanel.GetComponent<DatePicker>();
        datePicker.AddCallback(OnDatePicked);

        for (int i = 0; i < buttonPrevMonth.Count; i++)
        {
            buttonPrevMonth[i].onClick.AddListener(delegate
            {
                OnClickButtonPrevMonth();
            });
        }

        for (int i = 0; i < buttonNextMonth.Count; i++)
        {
            buttonNextMonth[i].onClick.AddListener(delegate
            {
                OnClickButtonNextMonth();
            });
        }

        for (int i = 0; i < buttonPrevDay.Count; i++)
        {
            buttonPrevDay[i].onClick.AddListener(delegate
            {
                OnClickButtonPrevDay();
            });
        }

        for (int i = 0; i < buttonNextDay.Count; i++)
        {
            buttonNextDay[i].onClick.AddListener(delegate
            {
                OnClickButtonNextDay();
            });
        }

        for (int i = 0; i < buttonTodayPick.Count; i++)
        {
            buttonTodayPick[i].onClick.AddListener(delegate
            {
                OnClickButtonTodayPick();
            });
        }
    }

    private void OnClickButtonTodayPick()
    {
        today = DateTime.Now;
        bUpdateGraph = true;
        bUpdateInfo = true;
    }

    private void OnClickButtonPrevMonth()
    {
        today = today.AddMonths(-1);
        bUpdateGraph = true;
        bUpdateInfo = true;
    }

    private void OnClickButtonNextMonth()
    {
        today = today.AddMonths(1);
        bUpdateGraph = true;
        bUpdateInfo = true;
    }

    private void OnClickButtonPrevDay()
    {
        today = today.AddDays(-1);
        bUpdateGraph = true;
        bUpdateInfo = true;
    }

    private void OnClickButtonNextDay()
    {
        today = today.AddDays(1);
        bUpdateGraph = true;
        bUpdateInfo = true;
    }

    private void OnClickButtonClose()
    {
        menuController.toggleDetailed.isOn = !menuController.toggleDetailed.isOn;
    }
    private void OnClickButtonSelectCrane(int crane)
    {
        selectManager.ChangeCraneSelection(currentPier, crane);
        UpdateCurrentCrane(currentPier, crane);
    }

    private void OnClickButtonDatePicker()
    {
        datePickerPanel.SetActive(true);
    }

    private void OnClickButtonExportDailyCsv()
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            FileBrowser.SetFilters(false, new FileBrowser.Filter("collision log(.csv)", ".csv"));
            FileBrowser.SetDefaultFilter(".csv");
            FileBrowser.ShowSaveDialog(OnSaveFileDaily, null);
        }
    }

    private void OnClickButtonExportMonthlyCsv()
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            FileBrowser.SetFilters(false, new FileBrowser.Filter("collision log(.csv)", ".csv"));
            FileBrowser.SetDefaultFilter(".csv");
            FileBrowser.ShowSaveDialog(OnSaveFileMonthly, null);
        }
    }

    private void OnClickButtonExportOperationTimeCsv()
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            FileBrowser.SetFilters(false, new FileBrowser.Filter("operation log(.csv)", ".csv"));
            FileBrowser.SetDefaultFilter(".csv");
            FileBrowser.ShowSaveDialog(OnSaveFileOperationTime, null);
        }
    }

    private void OnSaveFileOperationTime(string[] files)
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            string logText = "Date,StartTime,LastTime,OperationTime\r\n";
            CraneKey key = craneInfo.GetCraneKeycode(currentPier, currentCrane);

            for (int dy = 4; dy >= 0; dy--)
            {
                int year = DateTime.Now.Year - dy;
                DateTime dateTimeFirst = new DateTime(year, 1, 1);
                for (int day = 0; day < operationHistory[key][year].daily.Length; day++)
                {
                    int startHour = operationHistory[key][year].daily[day].startHour;
                    int startMin = operationHistory[key][year].daily[day].startMin;
                    int lastHour = operationHistory[key][year].daily[day].lastHour;
                    int lastMin = operationHistory[key][year].daily[day].lastMin;
                    if (lastHour != 0 && lastMin != 0)
                    {
                        DateTime dateTime = dateTimeFirst.AddDays(day);
                        DateTime t1 = new DateTime(dateTime.Year, dateTime.Month, dateTime.Day, startHour, startMin, 0);
                        DateTime t2 = new DateTime(dateTime.Year, dateTime.Month, dateTime.Day, lastHour, lastMin, 0);
                        TimeSpan span = t2 - t1;

                        logText += string.Format("{0}-{1}-{2},", dateTime.Year, dateTime.Month, dateTime.Day);
                        logText += string.Format("{0}:{1},", startHour, startMin);
                        logText += string.Format("{0}:{1},", lastHour, lastMin);
                        logText += string.Format("{0}:{1}", span.Hours, span.Minutes);
                        logText += "\r\n";
                    }
                }
            }

            FileBrowserHelpers.WriteTextToFile(files[0], logText);
        }
    }

    private void OnSaveFileMonthly(string[] files)
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            CraneKey key = craneInfo.GetCraneKeycode(currentPier, currentCrane);
            string logText = "Date,Caution,Warning,Stop,Minimum distance\r\n";

            for (int dy = 4; dy >= 0; dy--)
            {
                int year = DateTime.Today.Year - dy;
                for (int month = 0; month < collisionHistory[key][year].monthly.Length; month++)
                {
                    for (int day = 0; day < collisionHistory[key][year].monthly[month].minDistance.Length; day++)
                    {
                        float distance = collisionHistory[key][year].monthly[month].minDistance[day];
                        int level1 = collisionHistory[key][year].monthly[month].level1[day];
                        int level2 = collisionHistory[key][year].monthly[month].level2[day];
                        int level3 = collisionHistory[key][year].monthly[month].level3[day];
                        if (0 < distance && distance < 50)
                        {
                            logText += string.Format("{0}-{1}-{2},{3},{4},{5},{6}\r\n", year, (month + 1).ToString("00"), (day + 1).ToString("00"), level1, level2, level3, distance);
                        }
                    }
                }
            }

            FileBrowserHelpers.WriteTextToFile(files[0], logText);
        }
    }

    private void OnSaveFileDaily(string[] files)
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            CraneKey key = craneInfo.GetCraneKeycode(currentPier, currentCrane);
            string logText = "Date,Time,Caution,Warning,Stop,Minimum distance\r\n";

            for (int dy = 4; dy >= 0; dy--)
            {
                int year = DateTime.Today.Year - dy;
                DateTime dateTimeFirst = new DateTime(year, 1, 1);
                for (int day = 0; day < collisionHistory[key][year].dayly.Length; day++)
                {
                    for (int time = 0; time < collisionHistory[key][year].dayly[day].minDistance.Length; time++)
                    {
                        float distance = collisionHistory[key][year].dayly[day].minDistance[time];
                        int level1 = collisionHistory[key][year].dayly[day].level1[time];
                        int level2 = collisionHistory[key][year].dayly[day].level2[time];
                        int level3 = collisionHistory[key][year].dayly[day].level3[time];
                        if (0 < distance && distance < 50)
                        {
                            DateTime dateTime = dateTimeFirst.AddDays(day);
                            logText += string.Format("{0}-{1}-{2},{3}:00,", dateTime.Year, dateTime.Month.ToString("00"), dateTime.Day.ToString("00"), time.ToString("00"));
                            logText += string.Format("{0},{1},{2},{3}\r\n", level1, level2, level3, distance);
                        }
                    }
                }
            }

            FileBrowserHelpers.WriteTextToFile(files[0], logText);
        }
    }

    public void OnDatePicked(DateTime dateTime)
    {
        today = dateTime;
        bUpdateGraph = true;
        bUpdateInfo = true;
    }

    // Update is called once per frame
    void Update()
    {
        int key = craneInfo.GetCraneKeycode(currentPier, currentCrane);

        if (bUpdateWindInfo)
        {
            if (windSpeed.ContainsKey(key) && windDirection.ContainsKey(key))
            {
                textWindSpeed.text = string.Format("풍속: {0:0.0}m/s", windSpeed[key]);
                textWindDirection.text = string.Format("풍향: {0:0.0}deg", windDirection[key]);
            }
            else
            {
                textWindSpeed.text = string.Format("");
                textWindDirection.text = string.Format("");
            }
            bUpdateWindInfo = false;
        }
        if (bUpdateGraph)
        {
            UpdateGraph();
            bUpdateGraph = false;
        }
        if (bUpdateInfo)
        {
            UpdateInfo();
            bUpdateInfo = false;
        }
        if(bUpdateCooperation)
        {
            if(cooperationTexts1.ContainsKey(key) && cooperationTexts2.ContainsKey(key))
            {
                cooperationText1.text = cooperationTexts1[key];
                cooperationText2.text = cooperationTexts2[key];
            }
            else
            {
                cooperationText1.text = "";
                cooperationText2.text = "";
            }
            bUpdateCooperation = false;
        }
        if(bUpdatePlcInfo)
        {
            if (key == craneInfo.GetCraneKeycode(3, 5))
            {
                textTtcSlewing.text = "선회 정방향";
                textTtcSlewingInv.text = "선회 역방향";
            }
            else
            {
                textTtcSlewing.text = "호이스트3 상향";
                textTtcSlewingInv.text = "호이스트3 하향";
            }

            if (plcUpdateMessage.ContainsKey(key))
            {
                StructDefines.StPlcInfoMessage plcInfo = plcUpdateMessage[key];
                
                Hoist1DecelControl.text = ConvertDecel(plcInfo.Hoist1DecelControl);
                Hoist2DecelControl.text = ConvertDecel(plcInfo.Hoist2DecelControl);
                Hoist3DecelControl.text = ConvertDecel(plcInfo.Hoist3DecelControl);
                Trolley1DecelControl.text = ConvertDecel(plcInfo.Trolley1DecelControl);
                Trolley2DecelControl.text = ConvertDecel(plcInfo.Trolley2DecelControl);
                GantryDecelControl.text = ConvertDecel(plcInfo.GantryDecelControl);
                AuxHoistDecelControl.text = ConvertDecel(plcInfo.AuxHoistDecelControl);
                SlewingDecelControl.text = ConvertDecel(plcInfo.SlewingDecelControl);

                SetTextPlcEnable(ref bUserEnableSwich, plcInfo.bUserEnableSwich);
                SetTextPlcEnable(ref bGantryCommandFoward, plcInfo.bGantryCommandFoward);
                SetTextPlcEnable(ref bGantryCommandReverse, plcInfo.bGantryCommandReverse);
                SetTextPlcEnable(ref bTrolley1CommandRight, plcInfo.bTrolley1CommandRight);
                SetTextPlcEnable(ref bTrolley1CommandLeft, plcInfo.bTrolley1CommandLeft);
                SetTextPlcEnable(ref bTrolley2CommandRight, plcInfo.bTrolley2CommandRight);
                SetTextPlcEnable(ref bTrolley2CommandLeft, plcInfo.bTrolley2CommandLeft);
                SetTextPlcEnable(ref bHoist1CommandUp, plcInfo.bHoist1CommandUp);
                SetTextPlcEnable(ref bHoist1CommandDown, plcInfo.bHoist1CommandDown);
                SetTextPlcEnable(ref bHoist2CommandUp, plcInfo.bHoist2CommandUp);
                SetTextPlcEnable(ref bHoist2CommandDown, plcInfo.bHoist2CommandDown);
                SetTextPlcEnable(ref bHoist3CommandUp, plcInfo.bHoist3CommandUp);
                SetTextPlcEnable(ref bHoist3CommandDown, plcInfo.bHoist3CommandDown);
                SetTextPlcEnable(ref bAuxHoistCommandUp, plcInfo.bAuxHoistCommandUp);
                SetTextPlcEnable(ref bAuxHoistCommandDown, plcInfo.bAuxHoistCommandDown);

                Trolley1Position.text = string.Format("{0:0.00}m", plcInfo.Trolley1Position * 0.001);
                Trolley2Position.text = string.Format("{0:0.00}m", plcInfo.Trolley2Position * 0.001);
                GoliathPosition1.text = string.Format("{0:0.00}m", plcInfo.GoliathPosition1 * 0.001);
                GoliathPosition2.text = string.Format("{0:0.00}m", plcInfo.GoliathPosition2 * 0.001);
                Hoist1Position.text = string.Format("{0:0.00}m", plcInfo.Hoist1Position * 0.001);
                Hoist2Position.text = string.Format("{0:0.00}m", plcInfo.Hoist2Position * 0.001);
                Hoist3Position.text = string.Format("{0:0.00}m", plcInfo.Hoist3Position * 0.001);
                Hoist1Weight.text = string.Format("{0:0.0}t", plcInfo.Hoist1Weight * 0.1);
                Hoist2Weight.text = string.Format("{0:0.0}t", plcInfo.Hoist2Weight * 0.1);
                Hoist3Weight.text = string.Format("{0:0.0}t", plcInfo.Hoist3Weight * 0.1);
                AuxHoistWeight.text = string.Format("{0:0.0}t", plcInfo.AuxHoistWeight * 0.1);
            }
            else
            {
                Hoist1DecelControl.text = "";
                Hoist2DecelControl.text = "";
                Hoist3DecelControl.text = "";
                Trolley1DecelControl.text = "";
                Trolley2DecelControl.text =  "";
                GantryDecelControl.text =  ""; 
                AuxHoistDecelControl.text =  "";
                SlewingDecelControl.text =  ""; 

                bUserEnableSwich.text = ""; 
                bGantryCommandFoward.text = "";
                bGantryCommandReverse.text = "";
                bTrolley1CommandRight.text = "";
                bTrolley1CommandLeft.text =  "";
                bTrolley2CommandRight.text = "";
                bTrolley2CommandLeft.text =  "";
                bHoist1CommandUp.text = ""; 
                bHoist1CommandDown.text = ""; 
                bHoist2CommandUp.text = ""; 
                bHoist2CommandDown.text = ""; 
                bHoist3CommandUp.text = ""; 
                bHoist3CommandDown.text = ""; 
                bAuxHoistCommandUp.text = ""; 
                bAuxHoistCommandDown.text = "";

                Trolley1Position.text = ""; 
                Trolley2Position.text = ""; 
                GoliathPosition1.text = ""; 
                GoliathPosition2.text = ""; 
                Hoist1Position.text = ""; 
                Hoist2Position.text = ""; 
                Hoist3Position.text = ""; 
                Hoist1Weight.text = ""; 
                Hoist2Weight.text = ""; 
                Hoist3Weight.text = ""; 
                AuxHoistWeight.text = "";                      
            }
            bUpdatePlcInfo = false;
        }
    }
    public void UpdateOperator(int pier, int crane, string name, string info)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if(opertorName.ContainsKey(key) &&
            opertorInfo.ContainsKey(key))
        {
            opertorName[key] = name;
            opertorInfo[key] = info;

            if (pier == currentPier && crane == currentCrane)
            {
                bUpdateInfo = true;
            }
        }
    }

    public void UpdateRotorStatus(int pier, int crane, int[] rotorFps, int[] rotorRpm, int[] lidarFps, int[] errorCode)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if(this.rotorFps.ContainsKey(key) &&
            this.rotorRpm.ContainsKey(key) &&
            this.lidarFps.ContainsKey(key) &&
            this.errorCode.ContainsKey(key))
        {
            this.rotorFps[key] = rotorFps;
            this.rotorRpm[key] = rotorRpm;
            this.lidarFps[key] = lidarFps;
            this.errorCode[key] = errorCode;

            if (pier == currentPier && crane == currentCrane)
            {
                bUpdateInfo = true;
            }
        }
    }

    public void UpdateWindInfo(int pier, int crane, float windSpeed, float windDirection)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if (this.windSpeed.ContainsKey(key) &&
            this.windDirection.ContainsKey(key))
        {
            this.windSpeed[key] = windSpeed;
            this.windDirection[key] = windDirection;

            if (pier == currentPier && crane == currentCrane)
            {
                bUpdateWindInfo = true;
            }
        }
    }
    
    public void UpdateHistory(CollisionHistory history)
    {
        int key = craneInfo.GetCraneKeycode(history.pier, history.crane);

        if (collisionHistory.ContainsKey(key))
        {
            if(collisionHistory[key].ContainsKey(history.year))
            {
                collisionHistory[key][history.year] = history;

                if (history.pier == currentPier && history.crane == currentCrane)
                {
                    bUpdateGraph = true;
                }
            }
        }
    }
    public void UpdateOprtationHistory(OperationHistory history)
    {
        int key = craneInfo.GetCraneKeycode(history.pier, history.crane);

        if (operationHistory.ContainsKey(key))
        {
            if(operationHistory[key].ContainsKey(history.year))
            {
                operationHistory[key][history.year] = history;
                if (history.pier == currentPier && history.crane == currentCrane)
                {
                    bUpdateGraph = true;
                }
            }
        }
    }
    public void UpdateCurrentCrane(int pier, int crane)
    {
        currentPier = pier;
        currentCrane = crane;
        bUpdateGraph = true;
        bUpdateInfo = true;
        bUpdatePlcInfo = true;
        bUpdateCooperation = true;
        bUpdateWindInfo = true;

        if(pier == 3 && (crane == 0 || crane == 1 || crane == 5))
        {
            plcPanel.SetActive(true);
            plcTitlePanel.SetActive(true);
        }
        else
        {
            plcPanel.SetActive(false);
            plcTitlePanel.SetActive(false);
        }
    }

    public void UpdateInfo()
    {
        TextLabelDate1.text = today.ToString("yyyy년 MM월 dd일");
        TextLabelDate2.text = today.ToString("yyyy년 MM월");
        TextLabelMonth1.text = today.ToString("yyyy년 MM월");
        
        // update pier txt
        textPier.text = string.Empty;
        foreach (int key in craneInfo.keys)
        {
            if(craneInfo.dictPier[key] == currentPier)
            {
                textPier.text = craneInfo.dictPierName[key];
                break;
            }
        }

        // update crane txt
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            int key = craneInfo.GetCraneKeycode(currentPier, currentCrane);
            textCrane.text = craneInfo.dictCraneName[key];
        }
        else
        {
            textCrane.text = string.Empty;
        }

        // update button selection
        for (int i = 0; i < buttonSelectCrane.Count; i++)
        {
            if( craneInfo.Contains(currentPier, i) )
            {
                int key = craneInfo.GetCraneKeycode(currentPier, i);
                buttonSelectCrane[i].GetComponentInChildren<Text>().text = craneInfo.dictCraneName[key];
                buttonSelectCrane[i].gameObject.SetActive(true);

                if (i == currentCrane)
                {
                    ColorBlock colorBlock = buttonSelectCrane[i].colors;
                    colorBlock.normalColor = Color.gray;
                    colorBlock.highlightedColor = Color.gray;
                    buttonSelectCrane[i].colors = colorBlock;
                }
                else
                {
                    ColorBlock colorBlock = buttonSelectCrane[i].colors;
                    colorBlock.normalColor = Color.white;
                    colorBlock.highlightedColor = Color.white;
                    buttonSelectCrane[i].colors = colorBlock;
                }
            }
            else
            {
                buttonSelectCrane[i].gameObject.SetActive(false);
            }
        }

        if (craneInfo.Contains(currentPier, currentCrane))
        {
            int key = craneInfo.GetCraneKeycode(currentPier, currentCrane);

            int iDayOfYear = today.DayOfYear - 1;
            int iMonth = today.Month - 1;

            // Update Operator
            textDriver.text = opertorName[key];
            textDriverInfo.text = opertorInfo[key];

            // Update fps, rpm, error
            for (int i = 0; i < textRotorFps.Count; i++)
            {
                string text = "- FPS";
                if (i < rotorFps[key].Length)
                {
                    if (rotorFps[key][i] >= 0)
                    {
                        text = rotorFps[key][i].ToString() + " FPS";
                    }
                }
                textRotorFps[i].text = text;
            }

            for (int i = 0; i < textRotorRpm.Count; i++)
            {
                string text = "- RPM";
                if (i < rotorRpm[key].Length)
                {
                    if (rotorRpm[key][i] >= 0)
                    {
                        text = rotorRpm[key][i].ToString() + " RPM";
                    }
                }
                textRotorRpm[i].text = text;
            }

            for(int i = 0; i < textLidarFps.Count; i++)
            {
                string text = "- FPS";
                if(i < lidarFps[key].Length)
                {
                    if(lidarFps[key][i] >= 0)
                    {
                        text = lidarFps[key][i].ToString() + " FPS";
                    }
                }
                textLidarFps[i].text = text;
            }

            for (int i = 0; i < textErrorCode.Count; i++)
            {
                string text = "상태: -";
                if (i < errorCode[key].Length)
                {
                    if (errorCode[key][i] >= 0)
                    {
                        int err = errorCode[key][i];
                        text = "상태: " + ErrorText.text[err];
                    }
                }
                textErrorCode[i].text = text;
            }

            int numSensor = craneInfo.dictNumSensors[key];
            for(int i=0; i< sensorStatusPanel.Count; i++)
            {
                if(i < numSensor)
                {
                    sensorStatusPanel[i].SetActive(true);
                }
                else
                {
                    sensorStatusPanel[i].SetActive(false);
                }
            }
        }
        else
        {
            textDriver.text = "-";
            textDriverInfo.text = "-";
            for (int i = 0; i < textRotorFps.Count; i++)
            {
                textRotorFps[i].text = "- FPS";
            }

            for (int i = 0; i < textRotorRpm.Count; i++)
            {
                textRotorRpm[i].text = "- RPM";
            }

            for (int i = 0; i < textLidarFps.Count; i++)
            {
                textLidarFps[i].text = "- RPM";
            }

            for (int i = 0; i < textErrorCode.Count; i++)
            {
                textErrorCode[i].text = "상태: -";
            }

            foreach (GameObject panel in sensorStatusPanel)
            {
                panel.SetActive(false);
            }
        }
    }

    public void UpdateGraph()
    {
        if (craneInfo.Contains(currentPier, currentCrane))
        {
            int key = craneInfo.GetCraneKeycode(currentPier, currentCrane);
            int iDayOfYear = today.DayOfYear - 1;
            int iMonth = today.Month - 1;
            int year = today.Year;


            // Update current operation time text
            if (operationHistory[key][year].daily[iDayOfYear].lastHour > 0)
            {
                OperationHistoryDaily operation = operationHistory[key][year].daily[iDayOfYear];

                OperationHistoryDaily val = operationHistory[key][year].daily[iDayOfYear];
                DateTime t1 = new DateTime(today.Year, today.Month, today.Day, val.startHour, val.startMin, 0);
                DateTime t2 = new DateTime(today.Year, today.Month, today.Day, val.lastHour, val.lastMin, 0);
                TimeSpan diff = t2 - t1;

                textStartTime.text = string.Format("{0}:{1}", t1.Hour.ToString("00"), t1.Minute.ToString("00"));
                textLastTime.text = string.Format("{0}:{1}", t2.Hour.ToString("00"), t2.Minute.ToString("00"));
                textLastTime.text += string.Format(" ({0}시간 {1}분)", diff.Hours, diff.Minutes);
            }
            else
            {
                textStartTime.text = string.Format("-");
                textLastTime.text = string.Format("-");
            }

            // Update operation history
            List<OperationHistoryDaily> listOperation = new List<OperationHistoryDaily>();
            OperationHistoryDaily[] daily = operationHistory[key][year].daily;
            TimeSpan span = new TimeSpan();
            DateTime firstDay = new DateTime(today.Year, today.Month, 1);
            int idx = firstDay.DayOfYear - 1;
            for (int i = 0; i < DateTime.DaysInMonth(today.Year, today.Month); i++)
            {
                OperationHistoryDaily d = daily[idx + i];
                listOperation.Add(d);
                int startMin = d.startHour * 60 + d.startMin;
                int lastMin = d.lastHour * 60 + d.lastMin;
                int spanMin = lastMin - startMin;
                if (spanMin > 0)
                {
                    span += new TimeSpan(0, (lastMin - startMin), 0);
                }
            }
            graphOperationTime.UpdateGraph(listOperation);

            // Update total operation time
            textTotalOperationTime.text =
                today.Month.ToString("00") +
                "월 총 운용 시간: " +
                span.TotalHours.ToString("00.0") +
                "시간";

            // Update daily collision history
            List<int[]> listCollisionHistory = new List<int[]>();
            char[] level1 = collisionHistory[key][year].dayly[iDayOfYear].level1;
            char[] level2 = collisionHistory[key][year].dayly[iDayOfYear].level2;
            char[] level3 = collisionHistory[key][year].dayly[iDayOfYear].level3;
            for (int i = 0; i < level1.Length; i++)
            {
                int[] data = new int[3];
                data[0] = Convert.ToInt32(level1[i]);
                data[1] = Convert.ToInt32(level2[i]);
                data[2] = Convert.ToInt32(level3[i]);

                listCollisionHistory.Add(data);
            }
            graphCollisionHistoryDaily.UpdateGraph(listCollisionHistory);

            // Update monthly collision history
            List<int[]> listCollisionHistoryMonthly = new List<int[]>();
            char[] level1Monthly = collisionHistory[key][year].monthly[iMonth].level1;
            char[] level2Monthly = collisionHistory[key][year].monthly[iMonth].level2;
            char[] level3Monthly = collisionHistory[key][year].monthly[iMonth].level3;
            for (int i = 0; i < level1Monthly.Length; i++)
            {
                int[] data = new int[3];
                data[0] = Convert.ToInt32(level1Monthly[i]);
                data[1] = Convert.ToInt32(level2Monthly[i]);
                data[2] = Convert.ToInt32(level3Monthly[i]);

                listCollisionHistoryMonthly.Add(data);
            }

            graphCollisionHistoryMonthly.UpdateGraph(listCollisionHistoryMonthly);

        }
        else
        {
            textStartTime.text = string.Format("-");
            textLastTime.text = string.Format("-");
        }

    }

    public void UpdatePlcInfo(StructDefines.StPlcInfoMessage plcMessage)
    {
        int pier = (int)plcMessage.pier;
        int crane = (int)plcMessage.crane;

        int key = craneInfo.GetCraneKeycode(pier, crane);
        if(plcUpdateMessage.ContainsKey(key))
        {
            plcUpdateMessage[key] = plcMessage;

            if (currentPier == pier && currentCrane == crane)
            {
                Debug.Log("UpdatePlcInfo");
                bUpdatePlcInfo = true;
            }
        }
    }

    public void UpdateCooperationText(int pier, int crane, string text1, string text2)
    {
        int key = craneInfo.GetCraneKeycode(pier, crane);

        if (cooperationTexts1.ContainsKey(key))
        {
            cooperationTexts1[key] = text1;
        }

        if (cooperationTexts2.ContainsKey(key))
        {
            cooperationTexts2[key] = text2;
        }

        if(currentPier == pier && currentCrane == crane)
        {
            bUpdateCooperation = true;
        }
    }

    private Text SetTextPlcEnable(ref Text textItem, bool bEnableSwitch)
    {
        if (bEnableSwitch)
        {
            textItem.color = controlColor;
            textItem.text = "On";
        }
        else
        {
            textItem.color = Color.black;
            textItem.text = "Off";
        }
        return textItem;
    }

    private string ConvertDecel(int level)
    {
        string ret = "";
        switch(level)
        {
            case 3:
                ret = "정상";
                break;
            case 2:
                ret = "1단감속";
                break;
            case 1:
                ret = "2단감속";
                break;
            case 0:
                ret = "정지";
                break;
            default:
                ret = "-";
                break;
        }
        return ret;
    }
}
