using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CsCore;
using System.IO;
using CsCore.IO;

public class ProcessCraneMonitoring : MonoBehaviour
{
    public LogControlMenuBar logControlMenuBar;

    [Header("GUI object")] [SerializeField]
    private TopMenuBar topMenuBar;

    [SerializeField] private PopupDashBoard popupDashboard;
    [SerializeField] private TabStatus tabStatus;
    [SerializeField] private TabMaintenance tabMaintenance;
    [SerializeField] private PanelFunctionSet panelFunctionSet;

    [Header("Process object")] [SerializeField]
    private ProcessCamera processCamera;

    [SerializeField] private ProcessAlarm processAlarm;
    [SerializeField] private ProcessDistance processDistance;
    [SerializeField] private ProcessPointCloud processPointCloud;
    //pjh
    //[SerializeField] private List<CraneObject> craneObjects;
    CraneManager craneManager;
    //~pjh

    public CraneObject currentCrane { get; private set; }

    public bool bOperatorMode = false;
    public bool bStart = true;
    public float logPlayDuration = 1.0f;
    private float nextTime = 0;

    private void Awake() 
    {
        EventLog.Write("Program Start");//pjh
        
    }

    private void Start()
    {
        //pjh
        craneManager = GameObject.Find("CraneManager").GetComponent<CraneManager>();
        //bStart = true;
        SetStartBit(true);
        //~pjh
    }
 
     //pjh
    private void OnDestroy() {
        EventLog.Write("Program Quit");
        Debug.Log("Program Quit");

    }
    //~pjh

    public bool GetOperatorMode()
    {
        return bOperatorMode;
    }

    public bool SetStartBit(bool b)
    {
        //pjh
        if (bStart != b)
        {
            bStart = b;
            if (bStart)
            {
                StartCoroutine(ProcessMonitoring());
            }
        }
        //bStart = b;
        //~pjh
        return b;
    }

    //pjh
    IEnumerator ProcessMonitoring()
    {
        while (Time.time >= nextTime)
        {
            nextTime = Time.time + logPlayDuration;
            yield return null;
        }

        string textPier = Configuration.ReadConfigIni("CraneType", "Pier", CraneUtility.GetPierString((int)CraneUtility.PierType.PIER_Z));
        string textType = Configuration.ReadConfigIni("CraneType", "Type", "0");
        string textAdmin = Configuration.ReadConfigIni("Authority", "Admin", "1");
        string textOperator = Configuration.ReadConfigIni("Authority", "bOperator", "0");
        string textAlarmContinued = Configuration.ReadConfigIni("AlarmSound", "AlarmContinued", "0");
        string textLogRemainSpace = Configuration.ReadConfigIni("Log", "DiskMinSpaceGB", "5");
        string textDrawLengthCount = Configuration.ReadConfigIni("View", "DrawLengthCount", "3");
        string textDrawPointSize = Configuration.ReadConfigIni("View", "DrawPointSize", "1");
        string textAccumFrame = Configuration.ReadConfigIni("Viewer", "AccumFrame", "2");

        // CES
        // 시작시 Pier 초기화 상태 False로 통일 
        craneManager.PierReset();
        for (int i = 0; i < craneManager.GetCountCrans(); i++)
        {
            CraneObject craneObject = craneManager.GetCrane(i);
            {
                if (string.Compare(textPier, craneObject.pier, StringComparison.Ordinal) == 0 &&
                    string.Compare(textType, craneObject.type, StringComparison.Ordinal) == 0)
                {
                    currentCrane = craneObject;
                    craneObject.gameObject.SetActive(true);
                    craneManager.ActivePier(CraneUtility.GetPierType(textPier));

                    topMenuBar.SetCraneName(craneObject.craneName);
                    processCamera.SetCrane(craneObject);
                }
                else
                {
                    craneObject.gameObject.SetActive(false);
                }
            }

            //pjh
            if (!currentCrane)
            {
                currentCrane = craneManager.GetCranes()[craneManager.GetCountCrans() - 1];
            }
            //~pjh

            if (string.Compare(textAdmin, "1", StringComparison.Ordinal) == 0)
            {
                topMenuBar.SetLoginInfo(new PanelAccount.AccountData("admin", "admin", "", "", 2));
            }

            if (string.Compare(textOperator, "1", StringComparison.Ordinal) == 0)
            {
                bOperatorMode = true;
            }

            if (string.Compare(textAlarmContinued, "1", StringComparison.Ordinal) == 0)
            {
                processAlarm.SetSoundContState(AlarmSoundContState.Continous);
                popupDashboard.UpdateAlarmSoundContState(AlarmSoundContState.Continous);
            }
            else
            {
                processAlarm.SetSoundContState(AlarmSoundContState.Uncontinous);
                popupDashboard.UpdateAlarmSoundContState(AlarmSoundContState.Uncontinous);
            }

            if (int.TryParse(textLogRemainSpace, out var logRemainSpace))
            {
                CsCore.IO.DistanceLog.reserveDiskGB = logRemainSpace;
            }

            if (int.TryParse(textDrawLengthCount, out var drawLengthCount))
            {
                if (0 < drawLengthCount)
                {
                    processDistance.numDrawDistance = drawLengthCount;
                }
            }

            if (int.TryParse(textDrawPointSize, out var pointSize))
            {
                switch (pointSize)
                {
                    case 0:
                        processPointCloud.pointSize = 0.1f;
                        break;
                    case 1:
                        processPointCloud.pointSize = 0.2f;
                        break;
                    case 2:
                        processPointCloud.pointSize = 0.3f;
                        break;
                    case 3:
                        processPointCloud.pointSize = 0.5f;
                        break;
                    case 4:
                        processPointCloud.pointSize = 0.7f;
                        break;
                    default:
                        processPointCloud.pointSize = 0.3f;
                        break;
                }
            }

            if (int.TryParse(textAccumFrame, out var accumFrame))
            {
                processPointCloud.queueSize = accumFrame;
            }

            popupDashboard.SetNumRotor(currentCrane.numRotor);
            tabStatus.SetNumRotor(currentCrane.numRotor);
            tabMaintenance.SetNumRotor(currentCrane.numRotor);
            processAlarm.UpdateTargetCraneMosel(currentCrane.craneModel);

            bStart = false;

            yield return null;
        }
    }//~pjh
}
