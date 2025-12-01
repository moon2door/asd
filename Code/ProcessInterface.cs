using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using CsCore.IO;
using CsCore;
using System.Runtime.InteropServices;
using System.Text;
using CsCore.Data.CollisionProcessor;
using TMPro;
using System.Net.NetworkInformation;

public class ProcessInterface : MonoBehaviour, CsCore.ICollisionProcessor, CsCore.IIntegratedServer
{
    //[DllImport("Wrapper")]
    //public Wrapper.Compressor compressor;
    private CsCore.InterfaceCollisionProcessor interfaceCollisionProcessor;
    private CsCore.InterfaceIntegratedServer interfaceIntegratedServer;
    public ProcessPointCloud processPointCloud;
    public ProcessDistance processDistance;
    public ProcessAlarm processAlarm;
    public ProcessCraneMonitoring processCraneMonitoring;
    public ProcessLabel processLabel;
    public ProcessCamera processCamera;
    //public ProcessInput processInput;

    [Header("GUI Object")]
    public PopupDashBoard viewDashboardPopup;
    [SerializeField]
    private TabOperation tabOperation;
    [SerializeField]
    private TabStatus tabStatus;
    [SerializeField]
    private TopMenuBar topMenuBar;
    [SerializeField]        
    private TabManage tabManage;
    [SerializeField]
    private TabMaintenance tabMaintenance;
    [SerializeField]
    private LogControlMenuBar logControlMenuBar;
    [SerializeField]
    private PanelFunctionSet panelFunctionSet;

    //pjh
    // [SerializeField]
    // private List<CraneObject> craneObjects;
    //~pjh


    public bool bIgnoreCommunication = false;
    //private float syncTime = 0; pjh
    private CraneObject currentCrane;
    private CraneObject previouseCrane;
    private DateTime startTime;//pjh

    private DateTime updateTime;
    private DateTime currentTime;
    private bool bManagerClose = false;

    private Queue<AlarmState> states = new Queue<AlarmState>();
    private int maxState = 10;
    private int pierId = 0;
    private int craneId = 0;
    private int gpsQuality = -1;
    //pjh private bool bReset = false;
    //pjh  private bool bLogUpdate = false;
    //pjhprivate bool bLogClose = true;
    private bool bNetworkGps = true;
    private bool bNetworkDB = true;
    private int initialCount = 0;
    bool hasEverGPSConnected;//pjh
    private float connectCheckTime = 10f;//pjh
    string pierName;
    string craneName;

    //CES
    private bool bGPSQuality = true;
    private CsCore.Data.CollisionProcessor.StSystemStatus statusLog;
    readonly WaitForSecondsRealtime wait = new WaitForSecondsRealtime(1);//pjh
    // Start is called before the first frame update

    bool reConnect = false;
    private void Awake()
    {
        //pjh
        Application.targetFrameRate = 20;
        SocketConnect();
        statusLog = new CsCore.Data.CollisionProcessor.StSystemStatus();
        pierName = CsCore.Configuration.ReadConfigIni("CraneType", "Pier");
        craneName = CsCore.Configuration.ReadConfigIni("CraneType", "Type");
    }
    void SocketConnect()
    {
        //pjh
        string textCollisionIP = "";
        textCollisionIP = Configuration.ReadConfigIni("CraneMonitoringInterface", "IP", "127.0.0.1");
        string textCollisionPort = Configuration.ReadConfigIni("CraneMonitoringInterface", "PORT", "9098");
        int collisionPort = 9098;
        int.TryParse(textCollisionPort, out collisionPort);
        interfaceCollisionProcessor = new CsCore.InterfaceCollisionProcessor(this);
        interfaceCollisionProcessor.Create(textCollisionIP, collisionPort + 30000);

        string serverip = Configuration.ReadConfigIni("IntegratedMonitoringRouter", "IP", "127.0.0.1");
        string serverPort = Configuration.ReadConfigIni("IntegratedMonitoringRouter", "PORT", "27017");
        int port = 15000;
        int.TryParse(serverPort, out port);
        interfaceIntegratedServer = new CsCore.InterfaceIntegratedServer(this);
        interfaceIntegratedServer.Create(serverip, port);
        //tabManage.Init();
        //~pjh
    }
    //pjh
    private void Start()
    {
        startTime = DateTime.Now;
        StartCoroutine(UpdateCoroutine());
    }

    IEnumerator UpdateCoroutine()
    {
        while(true)
        {
            if (processCraneMonitoring.GetOperatorMode())
            {
                CsCore.Data.CollisionProcessor.MonitoringUserInfo data =
                    new CsCore.Data.CollisionProcessor.MonitoringUserInfo();
                data.id = topMenuBar.longinInfo.id;
                data.authority = Convert.ToInt32(processCraneMonitoring.bOperatorMode);
                data.name = topMenuBar.longinInfo.name;
                data.info = topMenuBar.longinInfo.info;
                SendReportMonitoringUserInfo(data);
            }

            currentTime = DateTime.Now;
            int currentTimeInt = currentTime.Minute * 60 + currentTime.Second;
            if ((currentTimeInt - (updateTime.Minute * 60 + updateTime.Second) > connectCheckTime && bManagerClose) 
            ||(currentTimeInt - (startTime.Minute * 60 + startTime.Second) > connectCheckTime && !bManagerClose))//3 //pjh
            {
                bManagerClose=false;
                //
                if(topMenuBar.bConnected == true)
                {
                    EventLog.Write("충돌방지시스템 연결 끊김");
                    Debug.Log("충돌방지시스템 연결 끊김\n");
                }

                tabOperation.UpdateTextGpsNetwork(true);
                tabOperation.UpdateTextGPSStandAole(false);
                tabOperation.UpdateTextRouterNetwork(true);
                topMenuBar.SetConnected(false);
                viewDashboardPopup.SetConnected(false);

                CsCore.Data.CollisionProcessor.StSystemStatus status = new StSystemStatus();
                status.networkSensorCom = false;
                status.networkGPS = false;
                status.networkRouter = false;
                for (int i = 0; i < 2; i++)
                {
                    status.rotorStatus[i].lidarFps = 0;
                    status.rotorStatus[i].rotorFps = 0;
                    status.rotorStatus[i].rotorRpm = 0;
                    status.rotorStatus[i].packetError = false;
                    status.rotorStatus[i].rotationError = true;
                    status.rotorStatus[i].NetworkError = false;
                    status.rotorStatus[i].encoderError = false;
                    status.rotorStatus[i].proxyMeterError = false;
                    status.rotorStatus[i].zeroSetError = false;
                    status.rotorStatus[i].errorCodeActurator = 0x0B;
                }

                GPSRouterCheck(status.networkGPS, gpsQuality);
                DBRouterCheck(status.networkRouter);
                tabStatus.UpdateNetworkStatus(status.networkSensorCom, status.networkGPS, status.networkRouter, 0);
                tabStatus.UpdateRotorInfo(status.rotorStatus);
                viewDashboardPopup.UpdateRotorStatus(status.rotorStatus);
            }


            yield return wait;
        }
    }

    //private void Update()
    //{
    //     if (Time.time > syncTime)
    //     {
    //         if (processCraneMonitoring.GetOperatorMode())
    //         {
    //             CsCore.Data.CollisionProcessor.MonitoringUserInfo data =
    //                 new CsCore.Data.CollisionProcessor.MonitoringUserInfo();
    //             data.id = topMenuBar.longinInfo.id;
    //             data.authority = Convert.ToInt32(processCraneMonitoring.bOperatorMode);
    //             data.name = topMenuBar.longinInfo.name;
    //             data.info = topMenuBar.longinInfo.info;
    //             SendReportMonitoringUserInfo(data);
    //         }

    //         if(statusLog.rotorStatus.Length > 0)
    //         {
    //             string statusRotor1 = ",0x" + statusLog.rotorStatus[0].errorCodeActurator.ToString("00") + " ," + statusLog.rotorStatus[0].rotorFps.ToString() + " ," + statusLog.rotorStatus[0].rotorRpm.ToString() + " ," + statusLog.rotorStatus[0].lidarFps.ToString();
    //             string statusRotor2 = ",0x" + statusLog.rotorStatus[1].errorCodeActurator.ToString("00") + " ," + statusLog.rotorStatus[1].rotorFps.ToString() + " ," + statusLog.rotorStatus[1].rotorRpm.ToString() + " ," + statusLog.rotorStatus[1].lidarFps.ToString();
    //             StatusLog.Write(statusRotor1 + statusRotor2, CraneUtility.GetCraneString(CraneUtility.GetPierString(pierName), Int32.Parse(craneName)));
    //         }
    //         syncTime = Time.time + 1;
    //     }

    //     currentTime = DateTime.Now;
    //     int currentTimeInt = currentTime.Minute * 60 + currentTime.Second;
    //     if ((currentTimeInt - (updateTime.Minute * 60 + updateTime.Second) > connectCheckTime && bManagerClose) 
    //     ||(currentTimeInt - (startTime.Minute * 60 + startTime.Second) > connectCheckTime && !bManagerClose))//3 //pjh
    //     {
    //         bManagerClose=false;
    //         //
    //         if(topMenuBar.bConnected == true)
    //         {
    //             EventLog.Write("충돌방지시스템 연결 끊김");
    //             Debug.Log("충돌방지시스템 연결 끊김\n");
    //         }

    //         tabOperation.UpdateTextGpsNetwork(true);
    //         tabOperation.UpdateTextGPSStandAole(false);
    //         tabOperation.UpdateTextRouterNetwork(true);
    //         topMenuBar.SetConnected(false);
    //         viewDashboardPopup.SetConnected(false);

    //         CsCore.Data.CollisionProcessor.StSystemStatus status = new StSystemStatus();
    //         status.networkSensorCom = false;
    //         status.networkGPS = false;
    //         status.networkRouter = false;
    //         for (int i = 0; i < 2; i++)
    //         {
    //             status.rotorStatus[i].lidarFps = 0;
    //             status.rotorStatus[i].rotorFps = 0;
    //             status.rotorStatus[i].rotorRpm = 0;
    //             status.rotorStatus[i].packetError = false;
    //             status.rotorStatus[i].rotationError = true;
    //             status.rotorStatus[i].NetworkError = false;
    //             status.rotorStatus[i].encoderError = false;
    //             status.rotorStatus[i].proxyMeterError = false;
    //             status.rotorStatus[i].zeroSetError = false;
    //             status.rotorStatus[i].errorCodeActurator = 0x0B;
    //         }
    //         tabStatus.UpdateNetworkStatus(status.networkSensorCom, status.networkGPS, status.networkRouter, 0);
    //         tabStatus.UpdateRotorInfo(status.rotorStatus);
    //     }
    // }
    //~pjh

    private void OnApplicationQuit()
    {
        interfaceCollisionProcessor?.Destroy();//pjh
        interfaceIntegratedServer?.Destroy();//pjh
    }

    public void SendDistanceBuffer(byte[] data, int length)
    {
        try
        {
            interfaceIntegratedServer?.SendDistanceBuffer(data, length);//pjh
        }
        catch(Exception)
        {
        }
    }

    public void SendAlarmUse(CsCore.Data.CollisionProcessor.StAlarmUse data)
    {
        interfaceCollisionProcessor?.SendAlarmUse(data);//pjh
    }

    public void SendCollisionZoneLength(CsCore.Data.CollisionProcessor.CollisionZoneLength data)
    {
        interfaceCollisionProcessor?.SendCollisionZoneLength(data);//pjh
    }

    public void SendRotorControl(CsCore.Data.CollisionProcessor.RotorControl data)
    {
        interfaceCollisionProcessor?.SendRotorControl(data);//pjh
    }

    public void SendRequestRotorParameter()
    {
        interfaceCollisionProcessor?.SendRequestRotorParameter();//pjh
    }

    public void SendRequestNewSetParameter(CsCore.Data.CollisionProcessor.RotorParameter data)
    {
        interfaceCollisionProcessor?.SendRequestSetNewParameter(data);//pjh
    }


    public void SendRequestCollisionZoneLength(CsCore.Data.CollisionProcessor.CollisionRequestZone data)
    {
        interfaceCollisionProcessor?.SendRequestCollisionZoneLength(data);//pjh
    }

    public void SendReportMonitoringUserInfo(CsCore.Data.CollisionProcessor.MonitoringUserInfo data)
    {
        interfaceCollisionProcessor?.SendReportMonitoringUserInfo(data);//pjh
    }
    public void SendMonitoringLabelInfo(CsCore.Data.CollisionProcessor.StMonitoringLabelInfo data)
{
        interfaceCollisionProcessor?.SendMonitoringLabelInfo(data);//pjh
    }
    public void SendMaxDistance(CsCore.Data.CollisionProcessor.StMaxDistance data)
    {
        interfaceCollisionProcessor?.SendMaxDistance(data);//pjh
    }

    void CsCore.ICollisionProcessor.OnConnected(string ip, int port)
    {
        reConnect = false;
        Debug.Log("CsCore.ICollisionProcessor.OnConnected" + ip + "(" + port + ")");
    }
    void CsCore.ICollisionProcessor.OnDisconnected(string ip, int port)
    {
        Debug.Log("CsCore.ICollisionProcessor.OnDisconnected" + ip + "(" + port + ")");
    }

    void CsCore.IIntegratedServer.OnConnected(string ip, int port)
    {
        Debug.Log("CsCore.IIntegratedServer.OnConnected" + ip + "(" + port + ")");
    }

    void CsCore.IIntegratedServer.OnDisconnected(string ip, int port)
    {
        Debug.Log("CsCore.IIntegratedServer.OnDisconnected" + ip + "(" + port + ")");
    }

    void CsCore.ICollisionProcessor.OnCollisionProcessorDistanceCompressed(CsCore.Data.CollisionProcessor.StDistanceSocketCompressed compressed)
    {
        if (bIgnoreCommunication == false)
        {
        }
    }
    void CsCore.ICollisionProcessor.OnCraneMiniInfo(CsCore.Data.CollisionProcessor.StCraneMiniInfo info)
    {
        if (bIgnoreCommunication == false)
        {
        }
    }

    void CsCore.ICollisionProcessor.OnMonitoringLabelInfo(CsCore.Data.CollisionProcessor.StMonitoringLabelInfo info)
    {
        //Debug.Log("aaa ---" + info.xyz.Length);
        List<Vector3> pinpoints = new List<Vector3>();
        for(int i = 0; i< info.xyz.Length; i+=3)
        {
            pinpoints.Add(new Vector3(info.xyz[i+0], info.xyz[i+1], info.xyz[i+2]));
        }
        processLabel.AddPinpoints(pinpoints);
    }


    public void ProcessDistanceSocket(ref CsCore.Data.CollisionProcessor.StDistanceSocket data)
    {
        int pier = data.attitude.pierId;
        int crane = data.attitude.craneId;
        craneId = data.attitude.craneId;
        pierId = data.attitude.pierId;
        gpsQuality = data.attitude.quality;

        List<int> indices = new List<int>();
        int posBeginBody = 0;
        int posEndbody = 0;
        int posBeginException = 0;
        int posEndException = 0;
        if (data.indicesBody.Count > 0)
        {
            posBeginBody = data.indicesBody[0];
            posEndbody = data.indicesBody[data.indicesBody.Count - 1];
        }
        if (data.indicesException.Count > 0)
        {
            posBeginException = data.indicesException[0];
            posEndException = data.indicesException[data.indicesException.Count - 1];
        }

        for (int i = 0; i < posBeginBody; i++)
        {
            indices.Add(i);
        }
        for (int i = posEndbody; i < posBeginException; i++)
        {
            indices.Add(i);
        }
        for (int i = posEndException; i < data.xyz.Length/3; i++)
        {
            indices.Add(i);
        }

        Vector3[] points = new Vector3[indices.Count];
        for (int i = 0; i < indices.Count; i++)
        {
            int idx = indices[i];
            points[i].x = data.xyz[idx * 3 + 0];
            points[i].y = data.xyz[idx * 3 + 1];
            points[i].z = data.xyz[idx * 3 + 2];
        }
        processPointCloud.UpdatePoints(pier, crane, points);

        float llcHighAngle = data.attitude.pose[1];

        //float llcAzimuth = -data.attitude.azimuth;
        float llcAzimuth = data.attitude.azimuth;


        double latitude = data.attitude.gpsLatitude2;
        double longitude = data.attitude.gpsLongitude2;
        // 고도 데ㅐ이터.
        float llcHeight = data.attitude.height;
        if (logControlMenuBar.IsLogPlaying())
        {
            //logControlMenuBar.SetLogStatus(false);
            currentCrane = logControlMenuBar.currentCrane;
            //bLogClose = true;//pjh
        }
        else
        {
            currentCrane = processCraneMonitoring.currentCrane;
        }

        if (currentCrane != null)
        {
            //cms : pier9, num0 is TC1, and this can't have any angle for jib part(highangle part)
            if (currentCrane.pierNum == 9 && currentCrane.craneNum == 0)
            {
                // 원래 값이 0이 아니라면 즉시 콘솔에 경고를 남긴다.
                if (Mathf.Abs(llcHighAngle) > Mathf.Epsilon)
                {
                    Debug.LogWarning(
                        $"[ProcessInterface] Pier 9 / Crane 0(TC1)에서 llcHighAngle 값이 {llcHighAngle}°로 들어왔습니다. " +
                        "안전규칙에 따라 0°로 강제 리셋합니다.");
                }
                llcHighAngle = 0f;
            }
            //cms
            if (gpsQuality > 0)
            {
                currentCrane.SetGPS(gpsQuality);
                currentCrane.SetCraneAttitude(llcHighAngle, llcAzimuth );
                currentCrane.SetPosition(latitude, longitude, llcHeight);
            }
            currentCrane.SetROI(data.attitude.hookRoi,data.attitude.craneRoi);
        }
        //CES 크레인에서 특정 부위에 대하여 거리 위험을 제외하는데 사용.
        //임시 코드
        if(currentCrane != null)
        {
            bool isCollisionCheck = false;
            for (int i=0;i< data.distanceInfo.Length; i++)
            {
                bool distanceException = false;
                distanceException = currentCrane.ExceptionDistance(data.distanceInfo[i]);
                if (distanceException == true)
                {
                    int expInt = data.distanceNormal.FindIndex(e => e == i);
                    if(expInt != -1)
                    {
                        data.distanceNormal[expInt] = -1;
                        data.collisionInfo[expInt] = 0;
                        isCollisionCheck = true;
                    }
                }
            }
            if(isCollisionCheck == true)
            {
                int maxCollision = 0;
                for (int i = 0; i < data.collisionInfo.Length; i++)
                {
                    if(maxCollision< data.collisionInfo[i])
                    {
                        maxCollision = data.collisionInfo[i];
                    }
                }
                data.collisionTotal = maxCollision;
            }
        }
        //pjh
        if(data.distanceNormal.Count > 0)
        {//~pjh
            int countDistance = 0;
            foreach (int idx in data.distanceNormal)
            {
                if (idx < 0)
                    continue;
                if (data.distanceInfo[idx].distance <= panelFunctionSet.maxDistance)
                {
                    countDistance++;
                }
            }

            DistanceObject.StDistance[] distance = new DistanceObject.StDistance[countDistance];

            int count = 0;
            foreach (int idx in data.distanceNormal)
            {
                if (idx < 0)
                    continue;
                if (data.distanceInfo[idx].distance <= panelFunctionSet.maxDistance)
                {
                    distance[count].position1.x = data.distanceInfo[idx].xCluster;
                    distance[count].position1.y = data.distanceInfo[idx].yCluster;
                    distance[count].position1.z = data.distanceInfo[idx].zCluster;

                    distance[count].position2.x = data.distanceInfo[idx].xCrane;
                    distance[count].position2.y = data.distanceInfo[idx].yCrane;
                    distance[count].position2.z = data.distanceInfo[idx].zCrane;

                    distance[count].distance = data.distanceInfo[idx].distance;
                    distance[count].craneIndex = data.distanceInfo[idx].CraneIndex;
                    distance[count].level = (int)data.collisionInfo[idx];
                    count++;
                }
            }
            processDistance.UpdateDistance(pier, crane, distance);
        }//~pjh

        if (logControlMenuBar.IsLogPlaying())
        {
            panelFunctionSet.UpdateMaxDisatance();
        }
        
        bool bAlarmArea = false;

        // �˶� ���� �︱�� �Ǵ�
        foreach (var i in data.distanceNormal)
        {
            if (i < 0)
                continue;
            var d = data.distanceInfo[i];
            var reason = data.reasonInfo[i];
            bool bException = (reason == 1) || (reason == 3) || (reason == 5);
            if (d.CraneIndex == 0)
            {
                if (bException)
                {
                    if (d.distance < tabManage.collisionZoneLengths[3].zone1 && d.distance > tabManage.collisionZoneLengths[3].zone2)
                    {
                        bAlarmArea = true;
                       if(!logControlMenuBar.IsLogPlaying())//pjh
                       EventLog.Write(string.Format(" 알림 {0}m, 영역{1}", d.distance.ToString("0.0"), reason));
                        break;
                    }
                }
                else
                {
                    if (d.distance < tabManage.collisionZoneLengths[2].zone1 && d.distance > tabManage.collisionZoneLengths[2].zone2)
                    {
                        bAlarmArea = true;
                       if(!logControlMenuBar.IsLogPlaying())//pjh
                        EventLog.Write(string.Format(" 알림 {0}m, 영역{1}", d.distance.ToString("0.0"), reason));
                        break;
                    }
                }
            }
            else if (d.CraneIndex == 1)
            {
                if (bException)
                {
                    if (d.distance < tabManage.collisionZoneLengths[5].zone1 && d.distance > tabManage.collisionZoneLengths[5].zone2)
                    {
                        bAlarmArea = true;
                       if(!logControlMenuBar.IsLogPlaying())//pjh
                        EventLog.Write(string.Format(" 알림 {0}m, 영역{1}", d.distance.ToString("0.0"), reason));
                        break;
                    }
                }
                else
                {
                    if (d.distance < tabManage.collisionZoneLengths[4].zone1 && d.distance > tabManage.collisionZoneLengths[4].zone2)
                    {
                        bAlarmArea = true;
                       if(!logControlMenuBar.IsLogPlaying())//pjh
                        EventLog.Write(string.Format(" 알림 {0}m, 영역{1}", d.distance.ToString("0.0"), reason));
                        break;
                    }
                }
            }
            else if (d.CraneIndex == 2)
            {
                if (bException)
                {
                    if (d.distance < tabManage.collisionZoneLengths[1].zone1 && d.distance > tabManage.collisionZoneLengths[1].zone2)
                    {
                        bAlarmArea = true;
                       if(!logControlMenuBar.IsLogPlaying())//pjh
                        EventLog.Write(string.Format(" 알림 {0}m, 영역{1}", d.distance.ToString("0.0"), reason));
                        break;
                    }
                }
                else
                {
                    if (d.distance < tabManage.collisionZoneLengths[0].zone1 && d.distance > tabManage.collisionZoneLengths[0].zone2)
                    {
                        bAlarmArea = true;
                       if(!logControlMenuBar.IsLogPlaying())//pjh
                        EventLog.Write(string.Format(" 알림 {0}m, 영역{1}", d.distance.ToString("0.0"), reason));
                        break;
                    }
                }
                    
            }
            else
            {
            }
        }

        // Update alarm state
        AlarmState state = AlarmState.None;
        if (data.collisionTotal == 1)
        {
            state = AlarmState.Warn;
        }
        else if (data.collisionTotal == 2)
        {
            state = AlarmState.Danger;
        }
        else if (data.collisionTotal == 3)
        {
            state = AlarmState.Stop;
        }
        else
        {
            if (bAlarmArea == true)
            {
                state = AlarmState.Alert;
            }
        }

        // �ֱ� ���� ���� ó��
        states.Enqueue(state);
        if (states.Count > maxState)
        {
            states.Dequeue();
        }
        
        if (state == AlarmState.None || state == AlarmState.Alert)
        {
            AlarmState maxCollision = AlarmState.None;
            foreach (var s in states)
            {
                if (s > maxCollision) maxCollision = s;
            }

            if (maxCollision == AlarmState.Alert)
            {
                state = AlarmState.Alert;
            }
        }

        if ( data.collisionInfo.Length == data.reasonInfo.Length && 
             data.collisionInfo.Length == data.distanceInfo.Length)
        {
            for (int i = 0; i < data.collisionInfo.Length; i++)
            {
                if (data.collisionInfo[i] == 1)
                {
                    if(!logControlMenuBar.IsLogPlaying())//pjh
                    EventLog.Write(string.Format(" 주의 {0}m, 영역{1}", data.distanceInfo[i].distance.ToString("0.0"), data.reasonInfo[i]));
                }
                else if(data.collisionInfo[i] == 2)
                {
                    if(!logControlMenuBar.IsLogPlaying())//pjh
                    EventLog.Write(string.Format(" 경고 {0}m, 영역{1}", data.distanceInfo[i].distance.ToString("0.0"), data.reasonInfo[i]));
                }
                else if (data.collisionInfo[i] == 3)
                {
                    if(!logControlMenuBar.IsLogPlaying())//pjh
                   EventLog.Write(string.Format(" 정지 {0}m, 영역{1}", data.distanceInfo[i].distance.ToString("0.0"), data.reasonInfo[i]));
                }
                else
                {
                }
            }
        }
        
        processAlarm.UpdateAlarmState(state);
    }
    void CsCore.ICollisionProcessor.OnCollisionProcessorDistance(CsCore.Data.CollisionProcessor.StDistanceSocket data)
    {
        if(bIgnoreCommunication == false)
        {
            ProcessDistanceSocket(ref data);
        }
    }

    void CsCore.ICollisionProcessor.OnSystemStatus(CsCore.Data.CollisionProcessor.StSystemStatus status)
    {
        var statusCopy = status.Clone();
        statusLog = statusCopy.Clone();
        if (topMenuBar.bConnected == false)
        {
            EventLog.Write("충돌방지시스템 연결됨");
            Debug.Log("충돌방지시스템 연결됨\n");
        }
        topMenuBar.SetConnected(true);
        viewDashboardPopup.SetConnected(true);

        if (statusCopy.bUseAlarm)
        {
            tabOperation.UpdateAlarmStatus(true);
            processAlarm.UpdateAlarmUseState(AlarmUseState.Use);
            viewDashboardPopup.UpdateAlarmUseStatus(AlarmUseState.Use);
        }
        else
        {
            tabOperation.UpdateAlarmStatus(false);
            processAlarm.UpdateAlarmUseState(AlarmUseState.UnUse);
            viewDashboardPopup.UpdateAlarmUseStatus(AlarmUseState.UnUse);
        }
        viewDashboardPopup.UpdateRotorStatus(statusCopy.rotorStatus);

        tabStatus.UpdateNetworkStatus(statusCopy.networkSensorCom, statusCopy.networkGPS, statusCopy.networkRouter, gpsQuality);

        GPSRouterCheck(statusCopy.networkGPS, gpsQuality);//pjh
        if (statusCopy.networkGPS)
        {
            tabOperation.UpdateTextGpsNetwork(false);
        }
        else
        {
            if (statusCopy.networkRouter)
            {
                tabOperation.UpdateTextGpsNetwork(true);
                tabOperation.UpdateTextRouterNetwork(false);
                tabOperation.UpdateTextGPSStandAole(false);
            }
        }

        if (statusCopy.networkRouter)
        {
            tabOperation.UpdateTextRouterNetwork(false);
        }
        else
        {
            tabOperation.UpdateTextRouterNetwork(true);
            tabOperation.UpdateTextGpsNetwork(false);
            tabOperation.UpdateTextGPSStandAole(false);
        }
        DBRouterCheck(statusCopy.networkRouter);

        if (statusCopy.networkGPS && statusCopy.networkRouter)
        {
            bool checkGpsQuality = false;
            if (gpsQuality >= 4)
            {
                checkGpsQuality = true;
                tabOperation.UpdateTextGPSStandAole(false);
            }
            else
            {
                checkGpsQuality = false;
                tabOperation.UpdateTextGPSStandAole(true);
            }

            GPSQualityCheck(checkGpsQuality);
        }
#if UNITY_EDITOR
        //Debug.Log("gpsQuality : " + gpsQuality + ", networkGPS : " + statusCopy.networkGPS + ", networkRouter : " + statusCopy.networkRouter);//pjh
#endif
        tabStatus.UpdateVersion(
            statusCopy.versionInterface,
            statusCopy.versionCluster,
            statusCopy.versionDistance,
            statusCopy.versionCollision,
            statusCopy.versionManagerSensor,
            statusCopy.versionManagerCollision);


        tabStatus.UpdateRotorInfo(statusCopy.rotorStatus);
        updateTime = DateTime.Now;
        bManagerClose = true;
        // statusLog 작성
        if (statusLog?.rotorStatus != null && statusLog.rotorStatus.Length > 0)
        {
            var sb = new System.Text.StringBuilder();

            for (int i = 0; i < statusLog.rotorStatus.Length; i++)
            {
                var rs = statusLog.rotorStatus[i];
                var code = rs.errorCodeActurator;
                if (rs.lidarFps <= 0) code |= 0x10;
                sb.Append($",0x{code:X2} ,{rs.rotorFps} ,{rs.rotorRpm} ,{rs.lidarFps}");
            }

            StatusLog.Write(
                sb.ToString(),
                CraneUtility.GetCraneString(CraneUtility.GetPierString(pierName), int.Parse(craneName))
            );
        }
     
        //StatusLog.Write(status, CraneUtility.GetCraneString(CraneUtility.GetPierString(pierInit), Int32.Parse(craneInit)));
        //string statusRotor1Actuator = "0x" + Convert.ToString(status.rotorStatus[0].errorCodeActurator);
        //var statusRotor1Fps = status.rotorStatus[0].rotorFps.ToString();
        //var statusRotor1Rpm = status.rotorStatus[0].rotorRpm.ToString();
        //var statusRotor1LidarFps = status.rotorStatus[0].lidarFps.ToString();
        //var statusRotor1 = "," + statusRotor1Actuator + "," + statusRotor1Fps + "," + statusRotor1Rpm + "," +
        //                   statusRotor1LidarFps;
        //var statusRotor2 = ",0x" + status.rotorStatus[1].errorCodeActurator.ToString("00") + " ," + status.rotorStatus[1].rotorFps.ToString() + " ," + status.rotorStatus[1].rotorRpm.ToString() + " ," + status.rotorStatus[1].lidarFps.ToString() + " ";

    }

    //pjh
    void GPSRouterCheck(bool gpsConnected, int quality)
    {
        gpsQuality = !gpsConnected ? -1 : quality;
        if(hasEverGPSConnected)
        {
            if (gpsConnected != bNetworkGps)
            {
                //pjh
                if(!gpsConnected)
                {
                    EventLog.Write("GPS 연결 오류");
                    Debug.Log("GPS 연결 오류\n");
                }
                else
                {
                    EventLog.Write("GPS 연결");
                    Debug.Log("GPS 연결\n");
                }
                //~pjh

                bNetworkGps = gpsConnected;
            }
        }
        else
        {
            if(gpsConnected && quality >=4) hasEverGPSConnected=true;
        }
    }
    //~pjh

    //CES
    void DBRouterCheck(bool dbConnected)
    {
        if (dbConnected != bNetworkDB)
        {
            //pjh
            if (!dbConnected)
            {
                EventLog.Write("DB 연결 오류");
                Debug.Log("DB 연결 오류\n");
            }
            else
            {
                EventLog.Write("DB 연결");
                Debug.Log("DB 연결\n");
            }
        }
        bNetworkDB = dbConnected;
    }
    void GPSQualityCheck(bool checkQuality)
    {
        if (bNetworkGps == true)
        {
            if (checkQuality != bGPSQuality)
            {
                //pjh
                if (!checkQuality)
                {
                    EventLog.Write("GPS 정확도 불량");
                    Debug.Log("GPS 정확도 불량\n");
                }
                else
                {
                    EventLog.Write("GPS 정확도 정상");
                    Debug.Log("GPS 정확도 정상\n");
                }

                bGPSQuality = checkQuality;
            }
        }
    }
    void CsCore.ICollisionProcessor.OnRotorParameter(CsCore.Data.CollisionProcessor.RotorParameter param)
    {
        tabMaintenance.UpdateRotorParameter(param);
    }

    void CsCore.ICollisionProcessor.OnMaintenanceInfo(CsCore.Data.CollisionProcessor.MaintenanceInfo info)
    {
        tabMaintenance.UpdateMaintenanceInfo(info);
    }

    void CsCore.ICollisionProcessor.OnCollisionZoneLength(CsCore.Data.CollisionProcessor.CollisionZoneLength length)
    {
        tabManage.UpdateCollisionDistance(length);
    }

    void CsCore.ICollisionProcessor.OnMaxDistance(CsCore.Data.CollisionProcessor.StMaxDistance maxDistance)
    {
        panelFunctionSet.maxDistance = maxDistance.maxDistance;
    }

    void CsCore.ICollisionProcessor.OnRotorControlSocket(RotorControlSocket info)
    {
        string state = "";
       if (info.bStart)
        {
            state = "시작";
        }
        else
        {
            state = "정지";
        }

        EventLog.Write((info.numSensor+ 1)+" 번 회전 구동기 제어 상태 : " + state);
    }
}