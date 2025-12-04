using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Runtime.InteropServices;
using System;
using Microsoft.Win32.SafeHandles;
using UnityEngine.UI;

using CraneKey = System.Int32;

public class MySocket : MonoBehaviour
{
    public CraneInfo craneInfo;
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

    //
    private GameObject kjib1;
    private GameObject kjib2;
    private GameObject kjib3;
    private GameObject kttc23;
    private GameObject kllc18;
    private GameObject kllc19;
    private GameObject hangc1;
    private GameObject hangc2;
    private GameObject hantc1;
    private GameObject hantc2;
    private GameObject hantc3;
    private GameObject hantc4;
    private GameObject hantc5;
    private GameObject hantc6;
    private GameObject p6llc7;
    private GameObject p6llc23;
    private GameObject jllc24;
    private GameObject jllc11;
    private GameObject jllc8;
    private GameObject jllc9;
    private GameObject g2llc12;
    private GameObject g2llc13;
    private GameObject g3llc19;
    private GameObject g3llc20;
    private GameObject g4llc25;
    private GameObject g4llc26;

    private Dictionary<int, float> windSpeeds = new Dictionary<int, float>();

    private uint numCooperationRequest=0;

    // Start is called before the first frame update
    void Start()
    {
        foreach(int key in craneInfo.keys)
        {
            listUpdated.Add(key, false);
            position.Add(key, Vector3.zero);
            rotation.Add(key, Vector3.zero);
            translation.Add(key, Vector3.zero);
            hookPos1.Add(key, Vector3.zero);
            hookPos2.Add(key, Vector3.zero);
            hookPos3.Add(key, Vector3.zero);
        }
        
        kjib1 = craneInfo.craneGameObject[0];
        kjib2 = craneInfo.craneGameObject[1];
        kjib3 = craneInfo.craneGameObject[2];
        kttc23 = craneInfo.craneGameObject[3];
        kllc18 = craneInfo.craneGameObject[4];
        kllc19 = craneInfo.craneGameObject[5];
        hangc1 = craneInfo.craneGameObject[6];
        hangc2 = craneInfo.craneGameObject[7];
        hantc1 = craneInfo.craneGameObject[8];
        hantc2 = craneInfo.craneGameObject[9];
        hantc3 = craneInfo.craneGameObject[10];
        hantc4 = craneInfo.craneGameObject[11];
        hantc5 = craneInfo.craneGameObject[12];
        hantc6 = craneInfo.craneGameObject[13];
        p6llc7 = craneInfo.craneGameObject[14];
        p6llc23 = craneInfo.craneGameObject[15];
        jllc24 = craneInfo.craneGameObject[16];
        jllc11 = craneInfo.craneGameObject[17];
        jllc8 = craneInfo.craneGameObject[18];
        jllc9 = craneInfo.craneGameObject[19];
        g2llc12 = craneInfo.craneGameObject[20];
        g2llc13 = craneInfo.craneGameObject[21];
        g3llc19 = craneInfo.craneGameObject[22];
        g3llc20 = craneInfo.craneGameObject[23];
        g4llc25 = craneInfo.craneGameObject[24];
        g4llc26 = craneInfo.craneGameObject[25];

        try
        {
            if (bAutoStartInterface)
            {
                string appname = ".\\IntegratedMonitoringInterface.exe";
                System.Diagnostics.Process.Start(appname);
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
                        Debug.Log("Failed");
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

    private void UpdateModels()
    {
        float pierOffsetK = -8.2f;

        // Jib1
        int key = craneInfo.GetCraneKeycode(2, 0);
        kjib1.SetActive(listUpdated[key]);
        if (kjib1.activeSelf == true)
        {
            Transform tfjoint = GameObject.Find("k_jib1_tower_joint").transform;
            kjib1.transform.localPosition = position[key];
            kjib1.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("k_jib1_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("k_jib1_tower").transform.localPosition = new Vector3();
            GameObject.Find("k_jib1_tower").transform.localRotation = new Quaternion();
            GameObject.Find("k_jib1_tower").transform.RotateAround(tfjoint.position, Vector3.up, rotation[key].z + 180 + pierOffsetK);
        }

        // Jib2
        key = craneInfo.GetCraneKeycode(2, 1);
        kjib2.SetActive(listUpdated[key]);
        if (kjib2.activeSelf == true)
        {
            Transform tfjoint = GameObject.Find("k_jib2_tower_joint").transform;
            kjib2.transform.localPosition = position[key];
            kjib2.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("k_jib2_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("k_jib2_tower").transform.localPosition = new Vector3();
            GameObject.Find("k_jib2_tower").transform.localRotation = new Quaternion();
            GameObject.Find("k_jib2_tower").transform.RotateAround(tfjoint.position, Vector3.up, rotation[key].z + 180 + pierOffsetK);
        }

        // Jib3
        key = craneInfo.GetCraneKeycode(2, 2);
        kjib3.SetActive(listUpdated[key]);
        if (kjib3.activeSelf == true)
        {
            Transform tfjoint = GameObject.Find("k_jib3_tower_joint").transform;
            kjib3.transform.localPosition = position[key];
            kjib3.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("k_jib3_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("k_jib3_tower").transform.localPosition = new Vector3();
            GameObject.Find("k_jib3_tower").transform.localRotation = new Quaternion();
            GameObject.Find("k_jib3_tower").transform.RotateAround(tfjoint.position, Vector3.up, rotation[key].z + 90 + pierOffsetK);
        }

        // K TTC23
        key = craneInfo.GetCraneKeycode(2, 3);
        kttc23.SetActive(listUpdated[key]);
        if (kttc23.activeSelf == true)
        {
            kttc23.transform.localPosition = position[key];
            kttc23.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("k_ttc23_tower").transform.localRotation = Quaternion.AngleAxis(rotation[key].z + 180 + 90 + pierOffsetK, new Vector3(0, 0, 1));
        }

        // K LLC18
        key = craneInfo.GetCraneKeycode(2, 4);
        kllc18.SetActive(listUpdated[key]);
        if (kllc18.activeSelf == true)
        {
            Transform tf18joint = GameObject.Find("k_llc18_tower_joint").transform;
            kllc18.transform.localPosition = position[key];
            kllc18.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("k_llc18_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("k_llc18_tower").transform.localPosition = new Vector3();
            GameObject.Find("k_llc18_tower").transform.localRotation = new Quaternion();
            GameObject.Find("k_llc18_tower").transform.RotateAround(tf18joint.position, Vector3.up, rotation[key].z + 180 + pierOffsetK);
        }

        // K LLC19
        key = craneInfo.GetCraneKeycode(2, 5);
        kllc19.SetActive(listUpdated[key]);
        if (kllc19.activeSelf == true)
        {
            Transform tf19joint = GameObject.Find("k_llc19_tower_joint").transform;
            kllc19.transform.localPosition = position[key];
            kllc19.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("k_llc19_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("k_llc19_tower").transform.localPosition = new Vector3();
            GameObject.Find("k_llc19_tower").transform.localRotation = new Quaternion();
            GameObject.Find("k_llc19_tower").transform.RotateAround(tf19joint.position, Vector3.up, rotation[key].z + 180 + pierOffsetK);
        }

        // HAN GC1
        key = craneInfo.GetCraneKeycode(3, 0);
        hangc1.SetActive(listUpdated[key]);
        if (hangc1.activeSelf == true)
        {
            hangc1.transform.localPosition = position[key];
            hangc1.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_gc900_operationroom").transform.localPosition = new Vector3(0, translation[key].y, 0);
            GameObject.Find("han_gc1_trolly").transform.localPosition = new Vector3(0, hookPos1[key].y, 0);

            GameObject.Find("han_gc1_hook1high").transform.localPosition = new Vector3(hookPos1[key].x, hookPos1[key].y, 0);
            GameObject.Find("han_gc1_hook1low").transform.localPosition = new Vector3(hookPos1[key].x, hookPos1[key].y, hookPos1[key].z);
            GameObject.Find("han_gc1_hook2high").transform.localPosition = new Vector3(hookPos2[key].x, hookPos2[key].y, 17.5f);
            GameObject.Find("han_gc1_hook2low").transform.localPosition = new Vector3(hookPos2[key].x, hookPos2[key].y, hookPos2[key].z);
            GameObject.Find("han_gc1_hook3high").transform.localPosition = new Vector3(hookPos3[key].x, hookPos3[key].y, 17.5f);
            GameObject.Find("han_gc1_hook3low").transform.localPosition = new Vector3(hookPos3[key].x, hookPos3[key].y, hookPos3[key].z);

        }

        float pierOffsetHan = -39.0f;

        // HAN GC2
        key = craneInfo.GetCraneKeycode(3, 1);
        hangc2.SetActive(listUpdated[key]);
        if (hangc2.activeSelf == true)
        {
            hangc2.transform.localPosition = position[key];
            hangc2.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_gc450_operationroom").transform.localPosition = new Vector3(0, translation[key].y, 0);
            GameObject.Find("han_gc2_trolly").transform.localPosition = new Vector3(0, hookPos1[key].y, 0);

            GameObject.Find("han_gc2_hook1high").transform.localPosition = new Vector3(hookPos1[key].x, hookPos1[key].y, 0);
            GameObject.Find("han_gc2_hook1low").transform.localPosition = new Vector3(hookPos1[key].x, hookPos1[key].y, hookPos1[key].z);
            GameObject.Find("han_gc2_hook2high").transform.localPosition = new Vector3(hookPos2[key].x, hookPos2[key].y, 15.5f);
            GameObject.Find("han_gc2_hook2low").transform.localPosition = new Vector3(hookPos2[key].x, hookPos2[key].y, hookPos2[key].z);
            GameObject.Find("han_gc2_hook3high").transform.localPosition = new Vector3(hookPos3[key].x, hookPos3[key].y, 15.5f);
            GameObject.Find("han_gc2_hook3low").transform.localPosition = new Vector3(hookPos3[key].x, hookPos3[key].y, hookPos3[key].z);
        }

        // HAN TC1
        key = craneInfo.GetCraneKeycode(3, 2);
        hantc1.SetActive(listUpdated[key]);
        if (hantc1.activeSelf == true)
        {
            Transform tfTC1joint = GameObject.Find("han_tc1_tower_joint").transform;
            hantc1.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_tc1_tower").transform.localPosition = new Vector3();
            GameObject.Find("han_tc1_tower").transform.localRotation = new Quaternion();
            GameObject.Find("han_tc1_tower").transform.RotateAround(tfTC1joint.position, Vector3.up, rotation[key].z + pierOffsetHan);
        }

        // HAN TC2
        key = craneInfo.GetCraneKeycode(3, 3);
        hantc2.SetActive(listUpdated[key]);
        if (hantc2.activeSelf == true)
        {
            Transform tfTC2joint = GameObject.Find("han_tc2_tower_joint").transform;
            hantc2.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_tc2_tower").transform.localPosition = new Vector3();
            GameObject.Find("han_tc2_tower").transform.localRotation = new Quaternion();
            GameObject.Find("han_tc2_tower").transform.RotateAround(tfTC2joint.position, Vector3.up, rotation[key].z + pierOffsetHan);
        }

        // HAN TC3
        key = craneInfo.GetCraneKeycode(3, 4);
        hantc3.SetActive(listUpdated[key]);
        if (hantc3.activeSelf == true)
        {
            Transform tfTC3joint = GameObject.Find("han_tc3_tower_joint").transform;
            hantc3.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_tc3_tower").transform.localPosition = new Vector3();
            GameObject.Find("han_tc3_tower").transform.localRotation = new Quaternion();
            GameObject.Find("han_tc3_tower").transform.RotateAround(tfTC3joint.position, Vector3.up, rotation[key].z + pierOffsetHan);
        }

        // HAN TC4
        key = craneInfo.GetCraneKeycode(3, 5);
        hantc4.SetActive(listUpdated[key]);
        if (hantc4.activeSelf == true)
        {
            Transform tfTTC4joint = GameObject.Find("han_ttc4_tower_joint").transform;
            hantc4.transform.localPosition = position[key];
            hantc4.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_ttc4_tower").transform.localPosition = new Vector3();
            GameObject.Find("han_ttc4_tower").transform.localRotation = new Quaternion();
            GameObject.Find("han_ttc4_tower").transform.RotateAround(tfTTC4joint.position, Vector3.up, rotation[key].z + pierOffsetHan);

            GameObject.Find("han_ttc4_hook1high").transform.localPosition = new Vector3(hookPos1[key].x, hookPos1[key].y, 2.0f);
            GameObject.Find("han_ttc4_hook1low").transform.localPosition = new Vector3(hookPos1[key].x, hookPos1[key].y, hookPos1[key].z);
            GameObject.Find("han_ttc4_hook2high").transform.localPosition = new Vector3(hookPos2[key].x, hookPos2[key].y, 2.0f);
            GameObject.Find("han_ttc4_hook2low").transform.localPosition = new Vector3(hookPos2[key].x, hookPos2[key].y, hookPos2[key].z);
        }

        // HAN TC5
        key = craneInfo.GetCraneKeycode(3, 6);
        hantc5.SetActive(listUpdated[key]);
        if (hantc5.activeSelf == true)
        {
            Transform tfTC5joint = GameObject.Find("han_tc5_tower_joint").transform;
            hantc5.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_tc5_tower").transform.localPosition = new Vector3();
            GameObject.Find("han_tc5_tower").transform.localRotation = new Quaternion();
            GameObject.Find("han_tc5_tower").transform.RotateAround(tfTC5joint.position, Vector3.up, rotation[key].z + pierOffsetHan);
        }

        // HAN TC6
        key = craneInfo.GetCraneKeycode(3, 7);
        hantc6.SetActive(listUpdated[key]);
        if (hantc6.activeSelf == true)
        {
            Transform tfTC6joint = GameObject.Find("han_tc6_tower_joint").transform;
            hantc6.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("han_tc6_tower").transform.localPosition = new Vector3();
            GameObject.Find("han_tc6_tower").transform.localRotation = new Quaternion();
            GameObject.Find("han_tc6_tower").transform.RotateAround(tfTC6joint.position, Vector3.up, rotation[key].z + pierOffsetHan);
        }

        float pierOffset6 = 91.0f;
        // 6 LLC7
        key = craneInfo.GetCraneKeycode(4, 0);
        p6llc7.SetActive(listUpdated[key]);
        if (p6llc7.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("6llc7_tower_joint").transform;
            p6llc7.transform.localPosition = position[key];
            p6llc7.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("6_llc7_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("6_llc7_tower").transform.localPosition = new Vector3();
            GameObject.Find("6_llc7_tower").transform.localRotation = new Quaternion();
            GameObject.Find("6_llc7_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffset6);
        }

        // 6 LLC23
        key = craneInfo.GetCraneKeycode(4, 1);
        p6llc23.SetActive(listUpdated[key]);
        if (p6llc23.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("6llc23_tower_joint").transform;
            p6llc23.transform.localPosition = position[key];
            p6llc23.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("6_llc23_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("6_llc23_tower").transform.localPosition = new Vector3();
            GameObject.Find("6_llc23_tower").transform.localRotation = new Quaternion();
            GameObject.Find("6_llc23_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffset6);
        }

        // J LLC24
        float pierOffsetJ = -8.2f;
        key = craneInfo.GetCraneKeycode(1, 0);
        jllc24.SetActive(listUpdated[key]);
        if (jllc24.activeSelf == true)
        {
            Transform tf24joint = GameObject.Find("j_llc24_tower_joint").transform;
            jllc24.transform.localPosition = position[key];
            jllc24.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("J_llc24_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("J_llc24_tower").transform.localPosition = new Vector3();
            GameObject.Find("J_llc24_tower").transform.localRotation = new Quaternion();
            GameObject.Find("J_llc24_tower").transform.RotateAround(tf24joint.position, Vector3.up, rotation[key].z + 180 + pierOffsetJ);
        }

        // J LLC11
        key = craneInfo.GetCraneKeycode(1, 1);
        jllc11.SetActive(listUpdated[key]);
        if (jllc11.activeSelf == true)
        {
            Transform tf11joint = GameObject.Find("j_llc11_tower_joint").transform;
            jllc11.transform.localPosition = position[key];
            jllc11.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("J_llc11_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("J_llc11_tower").transform.localPosition = new Vector3();
            GameObject.Find("J_llc11_tower").transform.localRotation = new Quaternion();
            GameObject.Find("J_llc11_tower").transform.RotateAround(tf11joint.position, Vector3.up, rotation[key].z + 180 + pierOffsetJ);
        }

        // J LLC8
        key = craneInfo.GetCraneKeycode(1, 2);
        jllc8.SetActive(listUpdated[key]);
        if (jllc8.activeSelf == true)
        {
            Transform tf8joint = GameObject.Find("j_llc8_tower_joint").transform;
            jllc8.transform.localPosition = position[key];
            jllc8.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("J_llc8_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("J_llc8_tower").transform.localPosition = new Vector3();
            GameObject.Find("J_llc8_tower").transform.localRotation = new Quaternion();
            GameObject.Find("J_llc8_tower").transform.RotateAround(tf8joint.position, Vector3.up, rotation[key].z + 180 + pierOffsetJ);
        }

        // J LLC9
        key = craneInfo.GetCraneKeycode(1, 3);
        jllc9.SetActive(listUpdated[key]);
        if (jllc9.activeSelf == true)
        {
            Transform tf9joint = GameObject.Find("j_llc9_tower_joint").transform;
            jllc9.transform.localPosition = position[key];
            jllc9.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("J_llc9_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("J_llc9_tower").transform.localPosition = new Vector3();
            GameObject.Find("J_llc9_tower").transform.localRotation = new Quaternion();
            GameObject.Find("J_llc9_tower").transform.RotateAround(tf9joint.position, Vector3.up, rotation[key].z + 180 + pierOffsetJ);
        }

        float pierOffsetG2 = -10.0f;
        // G2 LLC12
        key = craneInfo.GetCraneKeycode(5, 0);
        g2llc12.SetActive(listUpdated[key]);
        if (g2llc12.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("g2_llc12_tower_joint").transform;
            g2llc12.transform.localPosition = position[key];
            g2llc12.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("g2_llc12_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("g2_llc12_tower").transform.localPosition = new Vector3();
            GameObject.Find("g2_llc12_tower").transform.localRotation = new Quaternion();
            GameObject.Find("g2_llc12_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffsetG2);
        }

        // G2 LLC13
        key = craneInfo.GetCraneKeycode(5, 1);
        g2llc13.SetActive(listUpdated[key]);
        if (g2llc13.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("g2_llc13_tower_joint").transform;
            g2llc13.transform.localPosition = position[key];
            g2llc13.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("g2_llc13_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("g2_llc13_tower").transform.localPosition = new Vector3();
            GameObject.Find("g2_llc13_tower").transform.localRotation = new Quaternion();
            GameObject.Find("g2_llc13_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffsetG2);
        }

        float pierOffsetG3 = -10.0f;
        // G3 LLC19
        key = craneInfo.GetCraneKeycode(6, 0);
        g3llc19.SetActive(listUpdated[key]);
        if (g3llc19.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("g3_llc19_tower_joint").transform;
            g3llc19.transform.localPosition = position[key];
            g3llc19.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("g3_llc19_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("g3_llc19_tower").transform.localPosition = new Vector3();
            GameObject.Find("g3_llc19_tower").transform.localRotation = new Quaternion();
            GameObject.Find("g3_llc19_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffsetG3);
        }

        // G3 LLC20
        key = craneInfo.GetCraneKeycode(6, 1);
        g3llc20.SetActive(listUpdated[key]);
        if (g3llc20.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("g3_llc20_tower_joint").transform;
            g3llc20.transform.localPosition = position[key];
            g3llc20.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("g3_llc20_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("g3_llc20_tower").transform.localPosition = new Vector3();
            GameObject.Find("g3_llc20_tower").transform.localRotation = new Quaternion();
            GameObject.Find("g3_llc20_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffsetG3);
        }

        float pierOffsetG4 = -10.0f;
        // G4 LLC25
        key = craneInfo.GetCraneKeycode(7, 0);
        g4llc25.SetActive(listUpdated[key]);
        if (g4llc25.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("g4_llc25_tower_joint").transform;
            g4llc25.transform.localPosition = position[key];
            g4llc25.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("g4_llc25_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("g4_llc25_tower").transform.localPosition = new Vector3();
            GameObject.Find("g4_llc25_tower").transform.localRotation = new Quaternion();
            GameObject.Find("g4_llc25_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffsetG4);
        }

        // G4 LLC26
        key = craneInfo.GetCraneKeycode(7, 1);
        g4llc26.SetActive(listUpdated[key]);
        if (g4llc26.activeSelf == true)
        {
            Transform tfJoint = GameObject.Find("g4_llc26_tower_joint").transform;
            g4llc26.transform.localPosition = position[key];
            g4llc26.transform.localRotation = Quaternion.AngleAxis(-rotation[key].z + 180, new Vector3(0, 0, 1));
            GameObject.Find("g4_llc26_jib").transform.localRotation = Quaternion.AngleAxis(-rotation[key].x, new Vector3(1, 0, 0));
            GameObject.Find("g4_llc26_tower").transform.localPosition = new Vector3();
            GameObject.Find("g4_llc26_tower").transform.localRotation = new Quaternion();
            GameObject.Find("g4_llc26_tower").transform.RotateAround(tfJoint.position, Vector3.up, rotation[key].z + pierOffsetG4);
        }
    }

    private void ThreadGetMessage()
    {
        while (threadMessage.ThreadState == ThreadState.Background)
        {
            try
            {
                NetworkStream stream = clientSocket.GetStream();
                int bytes = stream.Read(bufferHead, 0, 12);
                
                int messageId = BitConverter.ToInt32(bufferHead, 0);
                int pierId = BitConverter.ToInt32(bufferHead, 4);
                int craneId = BitConverter.ToInt32(bufferHead, 8);
                int key = craneInfo.GetCraneKeycode(pierId, craneId);

                //int bytes = stream.Read(bufferHead, 0, 6);
                //byte type = bufferHead[0];
                //byte code = bufferHead[1];
                //int payloadLength = BitConverter.ToInt32(bufferHead, 2);
                //int byteData = stream.Read(buffer, 0, payloadLength);
                //
                //CsCore.Data.CollisionProcessor.StDistanceSocket distanceData = new CsCore.Data.CollisionProcessor.StDistanceSocket(buffer);
                //int messageId = BitConverter.ToInt32(bufferHead, 0);
                //int pierId = distanceData.attitude.pierId;
                //int craneId = distanceData.attitude.craneId;
                //int key = craneInfo.GetCraneKeycode(pierId, craneId);
                // pointManagerComponent.UpdatePoints(pierId, craneId, numPoints, myPoints, myColors, indecies);

                if (pierId == 1 ||
                    pierId == 2 ||
                    pierId == 3 ||
                    pierId == 4 ||
                    pierId == 5 ||
                    pierId == 6 ||
                    pierId == 7)
                {
                  Debug.Log("OnMessage[" + pierId + "," + craneId + "] " + messageId);
                
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
                
                        position[key] = new Vector3(BitConverter.ToSingle(buffer, 0), -BitConverter.ToSingle(buffer, 4), BitConverter.ToSingle(buffer, 8));
                        rotation[key] = new Vector3(-BitConverter.ToSingle(buffer, 12), -BitConverter.ToSingle(buffer, 16), -BitConverter.ToSingle(buffer, 20));
                        translation[key] = new Vector3(BitConverter.ToSingle(buffer, 24), BitConverter.ToSingle(buffer, 28), BitConverter.ToSingle(buffer, 32));
                
                        hookPos1[key] = new Vector3(BitConverter.ToSingle(buffer, 36), BitConverter.ToSingle(buffer, 40), BitConverter.ToSingle(buffer, 44));
                        hookPos2[key] = new Vector3(BitConverter.ToSingle(buffer, 48), BitConverter.ToSingle(buffer, 52), BitConverter.ToSingle(buffer, 56));
                        hookPos3[key] = new Vector3(BitConverter.ToSingle(buffer, 60), BitConverter.ToSingle(buffer, 64), BitConverter.ToSingle(buffer, 68));
                
                        listUpdated[key] = true;
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
                
                        // 현재 안벽의 크레인 개수
                        int numCranes = 0;
                        for (int i=0; i< craneInfo.cranePierCode.Count; i++)
                        {
                            if(craneInfo.cranePierCode[i] == pierId)
                            {
                                numCranes++;
                            }
                        }
                        numCranes = Math.Min(numCranes, 16);
                
                        // 각 크레인 상태 갱신
                        for (int i = 0; i< numCranes; i++)
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
                            
                            distanceData[i] = new DistanceData(new Vector3(-x1, y1, z1), new Vector3(-x2, y2, z2), distance, level);
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
                                                (uint) StructDefines.StRequestCooperationList.ListCode.COOPLIST_REQUEST,
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
                
                            if(0<= startHour && startHour < 24) daily[i].startHour = startHour;
                            if(0<= startMin && startMin < 60) daily[i].startMin = startMin;
                            if(0<= lastHour && lastHour < 24) daily[i].lastHour = lastHour;
                            if (0 <= lastMin && lastMin < 60) daily[i].lastMin = lastMin;
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
                        for(int i = 0; i < numList; i++)
                        {
                            byte[] array = new byte[StructDefines.StCooperationMessage.byteSize];
                            Array.Copy(buffer, StructDefines.StCooperationMessage.byteSize * i, array, 0, StructDefines.StCooperationMessage.byteSize);
                            coops[i].FromBytes(array);
                        }
                        coopCtrl.UpdateCooperationList(coops);
                
                        menu.UpdateCooperationList(coops);
                    }
                    else if(messageId == 13)
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
                    else if(messageId == 15)
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
                            menu.UpdateLogPlayInfo(logMessage.startTime, logMessage.endTime, logMessage.curTime);
                        }
                    }
                    else
                    {
                    }
                }         
            }
            catch (SocketException e)
            {
                Debug.Log("Recieve exceprion: " + e.ToString());
                Thread.Sleep(1000);
            }
            catch (InvalidOperationException e)
            {
                Debug.Log("Recieve exceprion: " + e.ToString());
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

        NetworkStream stream = clientSocket.GetStream();
        stream.Write(bufferSend, 0, size);
    }
}
