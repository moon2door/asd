using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System;

using CraneKey = System.Int32;
using System.IO;
using System.Collections;

public class MySocket : MonoBehaviour
{
    CraneInfo craneInfo;
    public MenuController menu;
    public CooperationMentControl coopCtrl;
    private TcpClient clientSocket = new TcpClient();
    private pointCloudTest pointManagerComponent;
    private DistanceManager distanceManager;
    private ASCIIEncoding ascii = new ASCIIEncoding();

    private Thread threadConnection;
    private Thread threadMessage;

    private Dictionary<CraneKey, bool> listUpdated = new Dictionary<CraneKey, bool>();
    private Dictionary<CraneKey, Vector3> position = new Dictionary<CraneKey, Vector3>();
    private Dictionary<CraneKey, Vector3> rotation = new Dictionary<CraneKey, Vector3>();
    private Dictionary<CraneKey, Vector3> translation = new Dictionary<CraneKey, Vector3>();
    private Dictionary<CraneKey, Vector3> hookPos1 = new Dictionary<CraneKey, Vector3>();
    private Dictionary<CraneKey, Vector3> hookPos2 = new Dictionary<CraneKey, Vector3>();
    private Dictionary<CraneKey, Vector3> hookPos3 = new Dictionary<CraneKey, Vector3>();

    public bool bAutoStartInterface = false;

    // graph objects
    public DetailedMenuControl graphManager;

    // craneColorControl
    public CraneColorControl craneColorControl;

    //
    private byte[] bufferHead = new byte[20];
    //private byte[] buffer = new byte[12000268];
    private byte[] buffer = new byte[6553600];
    private byte[] bufferSend = new byte[10240];

    private List<CraneParts> cranes = new List<CraneParts>();//pjh

    private Dictionary<int, float> windSpeeds = new Dictionary<int, float>();

    private uint numCooperationRequest = 0;

    private object _lock = new object(); // 자물쇠 추가

    // Start is called before the first frame update
    void Start()
    {
        craneInfo = GetComponent<CraneInfo>();

        foreach (int key in craneInfo.keys)
        {
            listUpdated.Add(key, false);
            position.Add(key, Vector3.zero);
            rotation.Add(key, Vector3.zero);
            translation.Add(key, Vector3.zero);
            hookPos1.Add(key, Vector3.zero);
            hookPos2.Add(key, Vector3.zero);
            hookPos3.Add(key, Vector3.zero);
        }

        foreach(var obj in craneInfo.craneGameObject)
        {
            cranes.Add(obj.GetComponent<CraneParts>());
        }

        //pjh
        try
        {
            if (bAutoStartInterface)
            {
                string processName = "IntegratedMonitoringInterface";

                var processes = System.Diagnostics.Process.GetProcessesByName(processName);

                if (processes.Length == 0)
                {
                    var startInfo = new System.Diagnostics.ProcessStartInfo
                        {
                            FileName = ".\\" + processName + ".exe",
                            #if UNITY_EDITOR
                            WindowStyle = System.Diagnostics.ProcessWindowStyle.Minimized
                            #else
                            WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden // 최소화 상태로 시작
                            #endif
                        };
                    System.Diagnostics.Process.Start(startInfo);
                }
                else
                {
                    Debug.Log("The application is already running.");
                }
            }

            threadConnection = new Thread(ThreadSocketConnection);
            threadConnection.IsBackground = true;
            threadConnection.Start();

            threadMessage = new Thread(ThreadGetMessage);
            threadMessage.IsBackground = true;
            threadMessage.Start();
        }
        catch (SocketException e)
        {
            Debug.Log("Connection failed : " + e.ToString());
        }
        //~pjh

        pointManagerComponent = GetComponent<pointCloudTest>();
        distanceManager = GetComponent<DistanceManager>();

        // Initialize version text
        //Text versionText = GameObject.Find("TextVersion").GetComponent<Text>();
        //versionText.text = "Version " + Application.version;
    }

    void OnApplicationQuit()
    {
        // Kill interface process   
        if (bAutoStartInterface)
        {
            System.Diagnostics.Process[] processList = System.Diagnostics.Process.GetProcessesByName("IntegratedMonitoringInterface");
            foreach (System.Diagnostics.Process processitem in processList)
            {
                processitem.Kill();
            }
        }

        // Kill threads
        threadConnection.Abort();
        threadMessage.Abort();

        // Disconnect socket
        if (clientSocket.Connected)
        {
            clientSocket.Close();
        }
    }

    private void ThreadSocketConnection()
    {
        while (threadConnection.ThreadState == ThreadState.Background)
        {
            if (!clientSocket.Connected)
            {
                try
                {
                    TcpClient client = new TcpClient();
                    var result = client.BeginConnect("127.0.0.1", 15000, null, null);
                    bool success = result.AsyncWaitHandle.WaitOne(1000, true);
                    if (success)
                    {
                        Debug.Log("Connected");
                        client.EndConnect(result);
                        clientSocket = client;
                    }
                    else
                    {
                        Debug.Log("Connect Failed, ");
                        client.Close();
                        throw new SocketException(10060); // Connection timeout. 
                    }
                }
                catch (SocketException e)
                {
                    Debug.Log(string.Format("SocketException: {0}", e));
                }

            }
        }
        Debug.Log("End ThreadSocketConnection");
    }

    void Update()
    {
        UpdateModels();
    }

    //pjh
    private void UpdateModels()
    {
        int key = 0;
        foreach (var parts in cranes)
        {
            key = craneInfo.GetCraneKeycode(parts.pierCode, parts.craneCode);
            parts.gameObject.SetActive(listUpdated[key]);

            if (parts.gameObject.activeSelf == true)
            {
                // 1. 사용할 변수들을 미리 선언
                Vector3 currentPos;
                Vector3 currentRot;
                // GC 크레인 등을 위해 필요한 다른 변수들도 여기서 복사 준비
                Vector3 currentTrans;
                Vector3 curHook1, curHook2, curHook3;

                // 2. 자물쇠를 걸고 '한 번에' 복사 (스냅샷 뜨기)
                lock (_lock)
                {
                    if (position[key] == Vector3.zero) continue;

                    currentPos = position[key];
                    currentRot = rotation[key];

                    // (GC 크레인 로직이 뒤에 있다면 이것들도 같이 복사해야 안전합니다)
                    currentTrans = translation[key];
                    curHook1 = hookPos1[key];
                    curHook2 = hookPos2[key];
                    curHook3 = hookPos3[key];
                }

                // PHJ
                if (parts.pierCode == 8)
                {
                    // [수정] position[key] -> currentPos 로 변경
                    // [수정] rotation[key] -> currentRot 로 변경
                    parts.SetPosition(Math.Abs(currentPos.x), Math.Abs(currentPos.y), -currentPos.z, -currentRot.x);
                }
                //

                else
                {
                    // 여기 아래도 전부 currentPos, currentRot, currentTrans 등을 사용해야 합니다.
                    parts.transform.localPosition = currentPos + parts.cranePositionOffset;

                    if (parts.craneType == CraneParts.CraneType.LLC)
                    {
                        parts.SetJibAngle(-currentRot.x);
                        parts.SetCraneRotate(-currentRot.z + 180);
                    }
                    else if (parts.craneType == CraneParts.CraneType.TC)
                    {
                        parts.SetCraneRotate(-currentRot.z + 180);
                    }
                    else if (parts.craneType == CraneParts.CraneType.GC)
                    {
                        // translation[key] 대신 currentTrans 사용
                        // hookPos1[key] 대신 curHook1 사용
                        parts.SetGCTowerTransform(new Vector3(0, currentTrans.y, 0), new Vector3(0, curHook1.y, 0),
                            new Vector3(curHook1.x, curHook1.y, curHook1.z),
                            new Vector3(curHook2.x, curHook2.y, curHook2.z),
                            new Vector3(curHook3.x, curHook3.y, curHook3.z));
                    }
                    else
                    {
                        if (parts.pierCode == 2 && parts.craneCode == 3)
                        {
                            GameObject.Find("k_ttc23_tower").transform.localRotation = Quaternion.AngleAxis(currentRot.z + 180 + 90 + -8.2f, new Vector3(0, 0, 1));
                        }
                        else
                        {
                            parts.SetCraneRotate(-currentRot.z + 180);
                        }
                    }
                }
            }
        }
    }
    //~pjh

    private void ThreadGetMessage()
    {
        while (threadMessage.ThreadState == ThreadState.Background)
        {
            //pjh
            if (!clientSocket.Connected) continue;
            //~pjh
            try
            {
                NetworkStream stream = clientSocket.GetStream();
                var asyncResult = stream.BeginRead(bufferHead, 0, 12, null, null);
                //pjh
                if (!asyncResult.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(5)))
                {
                    Debug.LogError("데이터를 읽는 동안 타임아웃이 발생했습니다.");
                    continue;
                }
                int bytes = stream.EndRead(asyncResult);
                if (bytes == 0)
                {
                    Debug.LogError("서버와의 연결이 끊어졌습니다.");
                    continue;
                }
                //int bytes = stream.Read(bufferHead, 0, 12);
                //~pjh

                int messageId = BitConverter.ToInt32(bufferHead, 0);
                int pierId = BitConverter.ToInt32(bufferHead, 4);
                int craneId = BitConverter.ToInt32(bufferHead, 8);
                int key = craneInfo.GetCraneKeycode(pierId, craneId);
                CraneParts crane = cranes.Find(e => e.pierCode == pierId && e.craneCode == craneId);
                
                bool bSocketPierType = false;
                for(int i = 0; i < (int)PierUtility.PierType.MAX; i++)
                {
                    if(pierId == i + 1)
                    {
                        bSocketPierType = true;
                        break;
                    }
                }
                if (bSocketPierType == true)
                {
                    //Debug.Log("OnMessage[" + pierId + "," + craneId + "] " + messageId);

                    if (messageId == 0) // Read Point Cloud
                    {
                        bytes = stream.Read(bufferHead, 12, 4 + 4);
                        uint numPoints = BitConverter.ToUInt32(bufferHead, 12);
                        float timestamp = BitConverter.ToSingle(bufferHead, 12 + 4);

                        int pos = 0;
                        int remained = 4 * 6 * (int)numPoints;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        Vector3[] myPoints = new Vector3[numPoints];
                        Color[] myColors = new Color[numPoints];
                        int[] indecies = new int[numPoints];
                        for (int i = 0; i < numPoints; ++i)
                        {
                            myPoints[i] = new Vector3(BitConverter.ToSingle(buffer, i * 24 + 0), BitConverter.ToSingle(buffer, i * 24 + 8), BitConverter.ToSingle(buffer, i * 24 + 4));
                            myColors[i] = new Color(BitConverter.ToSingle(buffer, i * 24 + 12), BitConverter.ToSingle(buffer, i * 24 + 16), BitConverter.ToSingle(buffer, i * 24 + 20));
                            indecies[i] = i;
                        }
                        pointManagerComponent.UpdatePoints(pierId, craneId, numPoints, myPoints, myColors, indecies);
                    }
                    else if (messageId == 1) // Read crane attitude
                    {
                        int pos = 0;
                        int remained = 6 * 4 * 5;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        lock (_lock) // 자물쇠 잠금
                        {
                            position[key] = new Vector3(BitConverter.ToSingle(buffer, 0), -BitConverter.ToSingle(buffer, 4), BitConverter.ToSingle(buffer, 8));
                            rotation[key] = new Vector3(-BitConverter.ToSingle(buffer, 12), -BitConverter.ToSingle(buffer, 16), -BitConverter.ToSingle(buffer, 20));
                            translation[key] = new Vector3(BitConverter.ToSingle(buffer, 24), BitConverter.ToSingle(buffer, 28), BitConverter.ToSingle(buffer, 32));

                            hookPos1[key] = new Vector3(BitConverter.ToSingle(buffer, 36), BitConverter.ToSingle(buffer, 40), BitConverter.ToSingle(buffer, 44));
                            hookPos2[key] = new Vector3(BitConverter.ToSingle(buffer, 48), BitConverter.ToSingle(buffer, 52), BitConverter.ToSingle(buffer, 56));
                            hookPos3[key] = new Vector3(BitConverter.ToSingle(buffer, 60), BitConverter.ToSingle(buffer, 64), BitConverter.ToSingle(buffer, 68));
                            listUpdated[key] = true;
                        }

                    }
                    else if (messageId == 2) // Read crane status
                    {
                        int pos = 0;
                        int remained = 4 * 16 * 4;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        int[] craneAlive = new int[16];
                        float[] distance = new float[16];
                        int[] distanceStatus = new int[16];
                        int[] dataStatus = new int[16];

                        for (int i = 0; i < 16; ++i)
                        {
                            craneAlive[i] = BitConverter.ToInt32(buffer, 4 * 0 + i * 4);
                            distance[i] = BitConverter.ToSingle(buffer, 4 * 16 + i * 4);
                            distanceStatus[i] = BitConverter.ToInt32(buffer, 4 * 32 + i * 4);
                            dataStatus[i] = BitConverter.ToInt32(buffer, 4 * 48 + i * 4);
                        }

                        // 현재 안벽의 활성화된 크레인 개수
                        int numCranes = 0;
                        for (int i = 0; i < craneInfo.cranePierCode.Count; i++)
                        {
                            if (craneInfo.cranePierCode[i] == pierId)
                            {
                                cranes[i].isWork = craneAlive[numCranes] == 1;//pjh
                                numCranes++;
                            }
                        }
                        numCranes = Math.Min(numCranes, 16);

                        // 각 크레인 상태 갱신
                        for (int i = 0; i < craneAlive.Length; i++)
                        {
                            if (!craneInfo.Contains(pierId, i)) continue;

                            string txtStatus = "";
                            string txtDistance = "-";
                            string txtComment = "-";
                            Color color = Color.black;

                            switch (craneAlive[i])
                            {
                                case 0:
                                    txtStatus = "미접속";
                                    craneColorControl.SetActiveStatus(pierId, i, false);
                                    break;
                                case 1:
                                    if (dataStatus[i] == 0)
                                    {
                                        txtStatus = "점검";
                                        craneColorControl.SetActiveStatus(pierId, i, false);
                                    }
                                    else if (dataStatus[i] == 2)
                                    {
                                        txtStatus = "GPS 이상";
                                        craneColorControl.SetActiveStatus(pierId, i, false);
                                    }
                                    else
                                    {
                                        txtStatus = "정상";
                                        craneColorControl.SetActiveStatus(pierId, i, true);
                                    }
                                    break;
                                default:
                                    txtStatus = "-";
                                    craneColorControl.SetActiveStatus(pierId, i, false);
                                    break;
                            }

                            if (dataStatus[i] != 0)
                            {
                                switch (distanceStatus[i])
                                {
                                    case 0:
                                        txtDistance = "정상";
                                        craneColorControl.SetFlickering(pierId, i, false);
                                        color = new Color(50.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 1);
                                        break;
                                    case 1:
                                        txtDistance = "주의";
                                        craneColorControl.SetFlickeringColor(pierId, i, 0);
                                        craneColorControl.SetFlickering(pierId, i, true);
                                        color = new Color(252.0f / 255.0f, 204.0f / 255.0f, 0, 1);
                                        break;
                                    case 2:
                                        txtDistance = "경고";
                                        craneColorControl.SetFlickeringColor(pierId, craneId, 1);
                                        craneColorControl.SetFlickering(pierId, i, true);
                                        color = new Color(255.0f / 255.0f, 134.0f / 255.0f, 0, 1);
                                        break;
                                    case 3:
                                        txtDistance = "정지";
                                        craneColorControl.SetFlickeringColor(pierId, i, 2);
                                        craneColorControl.SetFlickering(pierId, i, true);
                                        color = new Color(192.0f / 255.0f, 0, 0);
                                        break;
                                    default:
                                        break;
                                }
                            }

                            menu.UpdateStatusInfo(pierId, i, txtStatus, txtDistance, txtComment, color);
                        }
                    }
                    else if (messageId == 3) // Read distance data
                    {
                        bytes = stream.Read(bufferHead, 12, 4);
                        uint numDistance = BitConverter.ToUInt32(bufferHead, 12);

                        int pos = 0;
                        int remained = 4 * 8 * (int)numDistance;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }
                        // 제외 영역 수정 위치 수정할대 알람 및 로그 확인.
                        if (numDistance > 1) continue;
                        DistanceData[] distanceData = new DistanceData[numDistance];
                        for (int i = 0; i < numDistance; ++i)
                        {
                            float x1 = BitConverter.ToSingle(buffer, i * 32 + 0);
                            float y1 = BitConverter.ToSingle(buffer, i * 32 + 4);
                            float z1 = BitConverter.ToSingle(buffer, i * 32 + 8);
                            float x2 = BitConverter.ToSingle(buffer, i * 32 + 12);
                            float y2 = BitConverter.ToSingle(buffer, i * 32 + 16);
                            float z2 = BitConverter.ToSingle(buffer, i * 32 + 20);
                            float distance = BitConverter.ToSingle(buffer, i * 32 + 24);
                            int level = BitConverter.ToInt32(buffer, i * 32 + 28);

                            DistanceData serverData = new DistanceData(new Vector3(-x1, y1, z1), new Vector3(-x2, y2, z2), distance, level);
                            //if (crane != null)
                            //{
                            //    if(crane.isException == true)
                            //    {
                            //        bool check = crane.ExceptionDistance(serverData);
                            //        if (check == true)
                            //            continue;
                            //    }
                            //}
                            distanceData[i] = serverData;
                        }
                        distanceManager.UpdateDistance(pierId, craneId, distanceData);
                    }
                    else if (messageId == 4) // Read collision log
                    {
                        bytes = stream.Read(bufferHead, 12, 4);
                        int year = BitConverter.ToInt32(bufferHead, 12);

                        int pos = 0;
                        int remained1 = ((4 * 24) + (3 * 24)) * 366;
                        while (remained1 > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained1);
                            pos += nRead;
                            remained1 -= nRead;
                        }

                        CollisionHistoryDaily[] daily = new CollisionHistoryDaily[366];
                        for (int i = 0; i < 366; ++i)
                        {
                            daily[i] = new CollisionHistoryDaily();
                            for (int j = 0; j < 24; j++)
                            {
                                float distance = BitConverter.ToSingle(buffer, i * (7 * 24) + j * 4);
                                byte level1 = buffer[i * (7 * 24) + (4 * 24) + j];
                                byte level2 = buffer[i * (7 * 24) + (5 * 24) + j];
                                byte level3 = buffer[i * (7 * 24) + (6 * 24) + j];

                                daily[i].minDistance[j] = distance;
                                daily[i].level1[j] = (char)level1;
                                daily[i].level2[j] = (char)level2;
                                daily[i].level3[j] = (char)level3;
                            }
                        }

                        pos = 0;
                        int remained2 = ((4 * 31) + (3 * 31)) * 12;
                        while (remained2 > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained2);
                            pos += nRead;
                            remained2 -= nRead;
                        }

                        CollisionHistoryMonthly[] monthly = new CollisionHistoryMonthly[12];
                        for (int i = 0; i < 12; i++)
                        {
                            monthly[i] = new CollisionHistoryMonthly();
                            for (int j = 0; j < 31; j++)
                            {
                                float distance = BitConverter.ToSingle(buffer, i * (7 * 31) + j * 4);
                                byte level1 = buffer[i * (7 * 31) + (4 * 31) + j];
                                byte level2 = buffer[i * (7 * 31) + (5 * 31) + j];
                                byte level3 = buffer[i * (7 * 31) + (6 * 31) + j];

                                monthly[i].minDistance[j] = distance;
                                monthly[i].level1[j] = (char)level1;
                                monthly[i].level2[j] = (char)level2;
                                monthly[i].level3[j] = (char)level3;
                            }
                        }
                        graphManager.UpdateHistory(new CollisionHistory(pierId, craneId, year, daily, monthly));
                    }
                    else if (messageId == 6) // Read operator info
                    {
                        byte[] bufferId = new byte[32];
                        byte[] bufferAuthority = new byte[key];
                        byte[] bufferName = new byte[32];
                        byte[] bufferPassword = new byte[32];
                        byte[] bufferReserved = new byte[32];

                        stream.Read(bufferId, 0, 32);
                        stream.Read(bufferAuthority, 0, 4);
                        stream.Read(bufferName, 0, 32);
                        stream.Read(bufferPassword, 0, 32);
                        stream.Read(bufferReserved, 0, 32);

                        string id = Encoding.GetEncoding("euc-kr").GetString(bufferId);
                        uint authority = BitConverter.ToUInt32(bufferAuthority, 0);
                        string name = Encoding.GetEncoding("euc-kr").GetString(bufferName);
                        string password = Encoding.GetEncoding("euc-kr").GetString(bufferPassword);
                        string reserved = Encoding.GetEncoding("euc-kr").GetString(bufferReserved);

                        graphManager.UpdateOperator(pierId, craneId, name, reserved);
                    }
                    else if (messageId == 7) // Read system status
                    {
                        bytes = stream.Read(buffer, 0, 4 * 33);

                        double timestamp = BitConverter.ToDouble(buffer, 0);
                        uint InterfaceVersion = BitConverter.ToUInt32(buffer, 4 * 2);
                        uint ClusterVersion = BitConverter.ToUInt32(buffer, 4 * 3);
                        uint DistanceVersion = BitConverter.ToUInt32(buffer, 4 * 4);
                        uint CollisionVersion = BitConverter.ToUInt32(buffer, 4 * 5);
                        uint ManagerSensorVersion = BitConverter.ToUInt32(buffer, 4 * 6);
                        uint ManagerCollisionVersion = BitConverter.ToUInt32(buffer, 4 * 7);
                        uint cooperationMode = BitConverter.ToUInt32(buffer, 4 * 8);
                        uint numCooperationRequest = BitConverter.ToUInt32(buffer, 4 * 9);

                        uint numSensor = BitConverter.ToUInt32(buffer, 4 * 10);
                        int[] rotorFps = new int[numSensor];
                        int[] rotorRpm = new int[numSensor];
                        int[] lidarFps = new int[numSensor];
                        int[] errorCode = new int[numSensor];

                        for (int i = 0; i < numSensor; i++)
                        {
                            rotorFps[i] = BitConverter.ToInt32(buffer, 4 * 11 + i * 4);
                            rotorRpm[i] = BitConverter.ToInt32(buffer, 4 * 11 + 5 * 4 + i * 4);
                            errorCode[i] = BitConverter.ToInt32(buffer, 4 * 11 + 10 * 4 + i * 4);
                        }

                        float windspeed = BitConverter.ToSingle(buffer, 4 * 11 + 15 * 4);
                        float windDirection = BitConverter.ToSingle(buffer, 4 * 11 + 15 * 4 + 4);

                        for (int i = 0; i < numSensor; i++)
                        {
                            lidarFps[i] = BitConverter.ToInt32(buffer, 4 * 11 + 15 * 4 + 8 + i * 4);
                        }

                        if (pierId == 3)
                        {
                            if (craneId == 0 || craneId == 1 || craneId == 5)
                            {
                                if (numCooperationRequest > 0)
                                {
                                    //menu.UpdateCooperationText(string.Format("협업 요청이 {0}건 있습니다.", numCooperationRequest));

                                    if (this.numCooperationRequest != numCooperationRequest)
                                    {
                                        StructDefines.StRequestCooperationList reqMessage =
                                            new StructDefines.StRequestCooperationList(3, 1,
                                                (uint)StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST,
                                                DateTime.Today.Year, DateTime.Today.Month, DateTime.Today.Day);
                                        SendRequestCooperationList(reqMessage);
                                    }
                                }
                                else
                                {
                                    menu.UpdateCooperationText("");
                                }
                            }

                            this.numCooperationRequest = numCooperationRequest;
                        }

                        float speed = 0;
                        windSpeeds[key] = windspeed;
                        foreach (float s in windSpeeds.Values)
                        {
                            if (speed < s) speed = s;
                        }

                        menu.UpdateWindText(string.Format("{0:0.0}m/s", speed));

                        graphManager.UpdateRotorStatus(pierId, craneId, rotorFps, rotorRpm, lidarFps, errorCode);
                        graphManager.UpdateWindInfo(pierId, craneId, windspeed, windDirection);
                        coopCtrl.UpdateCooperationState(pierId, craneId, cooperationMode);

                        //COOP_INDICATING_NONE = 0x10,
                        //COOP_INDICATING_REQUEST = 0x11,
                        //COOP_INDICATING_REQUEST_ACCEPTED = 0x12,
                        //COOP_INDICATING_ON = 0x13,
                        string text1 = "";
                        string text2 = "";
                        if (cooperationMode == 0x13)
                        {
                            text1 = "협업중";
                            menu.UpdateCooperationState(pierId, craneId, true);
                        }
                        else if (cooperationMode == 0x12)
                        {
                            text1 = "협업 예정";
                            menu.UpdateCooperationState(pierId, craneId, false);
                        }
                        else if (cooperationMode == 0x11)
                        {
                            text1 = "협업 요청";
                            menu.UpdateCooperationState(pierId, craneId, false);
                        }
                        else
                        {
                            text1 = "협업 해제";
                            menu.UpdateCooperationState(pierId, craneId, false);
                        }
                        graphManager.UpdateCooperationText(pierId, craneId, text1, text2);
                    }
                    else if (messageId == 8) // Read operation history
                    {
                        bytes = stream.Read(bufferHead, 12, 4);
                        int year = BitConverter.ToInt32(bufferHead, 12);

                        int pos = 0;
                        int remained = (4 * 4) * 366;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        OperationHistoryDaily[] daily = new OperationHistoryDaily[366];
                        for (int i = 0; i < 366; ++i)
                        {
                            daily[i] = new OperationHistoryDaily();

                            char startHour = (char)buffer[i * (16) + 0];
                            char startMin = (char)buffer[i * (16) + 1];
                            char lastHour = (char)buffer[i * (16) + 2];
                            char lastMin = (char)buffer[i * (16) + 3];

                            if (0 <= startHour && startHour < 24) 
                            daily[i].startHour = startHour;
                            if (0 <= startMin && startMin < 60) 
                            daily[i].startMin = startMin;
                            if (0 <= lastHour && lastHour < 24) 
                            daily[i].lastHour = lastHour;
                            if (0 <= lastMin && lastMin < 60) 
                            daily[i].lastMin = lastMin;
                        }
                        graphManager.UpdateOprtationHistory(new OperationHistory(pierId, craneId, year, daily));

                        if (DateTime.Now.Year == year)
                        {
                            OperationHistoryDaily val = daily[DateTime.Today.DayOfYear - 1];
                            DateTime t1 = new DateTime(DateTime.Now.Year, DateTime.Now.Month, DateTime.Now.Day, val.startHour, val.startMin, 0);
                            DateTime t2 = new DateTime(DateTime.Now.Year, DateTime.Now.Month, DateTime.Now.Day, val.lastHour, val.lastMin, 0);
                            TimeSpan diff = t2 - t1;
                            menu.UpdateTimestamp(pierId, craneId, diff);
                        }
                    }
                    else if (messageId == 12) // Read cooperation
                    {
                        bytes = stream.Read(bufferHead, 12, 4);
                        int numList = BitConverter.ToInt32(bufferHead, 12);

                        int pos = 0;
                        int remained = 164 * 10;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        StructDefines.StCooperationMessage[] coops = new StructDefines.StCooperationMessage[numList];
                        for (int i = 0; i < numList; i++)
                        {
                            byte[] array = new byte[StructDefines.StCooperationMessage.byteSize];
                            Array.Copy(buffer, StructDefines.StCooperationMessage.byteSize * i, array, 0, StructDefines.StCooperationMessage.byteSize);
                            coops[i].FromBytes(array);
                        }
                        coopCtrl.UpdateCooperationList(coops);

                        menu.UpdateCooperationList(coops);
                    }
                    else if (messageId == 13)
                    {
                        StructDefines.StPlcInfoMessage plcMessage = new StructDefines.StPlcInfoMessage((uint)pierId, (uint)craneId);

                        int pos = 0;
                        int remained = StructDefines.StPlcInfoMessage.byteSize;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        if (plcMessage.FromBytes(buffer))
                        {
                            graphManager.UpdatePlcInfo(plcMessage);
                        }
                    }
                    else if (messageId == 15)
                    {
                        StructDefines.StReplyLogin repMessage = new StructDefines.StReplyLogin();

                        int pos = 0;
                        int remained = StructDefines.StReplyLogin.byteSize;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        if (repMessage.FromBytes(buffer))
                        {
                            menu.OnReplyLogin(repMessage, repMessage.id);
                        }
                    }
                    else if (messageId == 18)
                    {
                        Debug.Log("Receive Log Data");
                        StructDefines.StIndicateLogPlay logMessage = new StructDefines.StIndicateLogPlay();

                        int pos = 0;
                        int remained = StructDefines.StIndicateLogPlay.byteSize;
                        while (remained > 0)
                        {
                            int nRead = stream.Read(buffer, pos, remained);
                            pos += nRead;
                            remained -= nRead;
                        }

                        if (logMessage.FromBytes(buffer))
                        {
                            receiveLogData = true; //pjh
                            menu.UpdateLogPlayInfo(logMessage.startTime, logMessage.endTime, logMessage.curTime);
                        }
                    }
                    else
                    {
                    }
                }
            }
            catch (IOException e)
            {
                Debug.Log("Recieve Network exception: " + e.ToString());
                Thread.Sleep(1000);
            }
            catch (SocketException e)
            {
                Debug.Log("Recieve Socket exception: " + e.ToString());
                Thread.Sleep(1000);
            }
            catch (InvalidOperationException e)
            {
                Debug.Log("Recieve exception: " + e.ToString());
                Thread.Sleep(1000);
            }
        }
        Debug.Log("End ThreadSocketMessage");
    }

    public void SendCooperation(StructDefines.StCooperationMessage message)
    {
        int size = message.ToBytes(ref bufferSend);

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }

    public void SendCooperationAccept(StructDefines.StCooperationAccept message)
    {
        int size = message.ToBytes(ref bufferSend);

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }
    public void SendCooperationRemove(StructDefines.StCooperationRemove message)
    {
        int size = message.ToBytes(ref bufferSend);

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }

    public void SendRequestCooperationList(StructDefines.StRequestCooperationList message)
    {
        int size = message.ToBytes(ref bufferSend);

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }
    public void SendRequestLogin(StructDefines.StRequestLogin message)
    {
        int size = message.ToBytes(ref bufferSend);

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }

    public void SendRequestAddUser(StructDefines.StRequestAddUser message)
    {
        int size = message.ToBytes(ref bufferSend);

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }

    public void SendRequestLogPlay(StructDefines.StRequestLogPlay message)
    {
        int size = message.ToBytes(ref bufferSend);
        StopCoroutine(CheckLogDataReceive());
        receiveLogData = false ; //pjh
        StartCoroutine(CheckLogDataReceive());//pjh

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }

    //pjh
    readonly WaitForSecondsRealtime wait = new WaitForSecondsRealtime(1);
    const int WaitForLogData = 10;
    bool receiveLogData;
    IEnumerator CheckLogDataReceive()
    {
        int waitTime = 0 ;
        while(waitTime++ < WaitForLogData && !receiveLogData)
        {
            yield return wait;
        }

        if(waitTime > WaitForLogData)
        {
            menu.SetLoadingPanel(false);
            Debug.Log("Log Data Time out");
        }
    }
    //~pjh
}
