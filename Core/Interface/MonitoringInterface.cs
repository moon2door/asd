using System;
using UnityEngine;

//pjh
public class MonitoringInterface : MonoBehaviour
{
    System.Diagnostics.Process craneMonitoringInterface;

    private void Awake() 
    {
        try
        {
            string processName = CsCore.Configuration.ReadConfigIni("Interface", "ProcessName", "CraneMonitoringInterface");

            System.Diagnostics.Process[] processes = System.Diagnostics.Process.GetProcessesByName(processName);
            if (processes.Length == 0)
            {
                var startInfo = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = ".\\" + processName + ".exe",
                        #if UNITY_EDITOR
                        WindowStyle = System.Diagnostics.ProcessWindowStyle.Minimized
                        #else
                        WindowStyle = System.Diagnostics.ProcessWindowStyle.Hidden
                        #endif
                    };
                craneMonitoringInterface = System.Diagnostics.Process.Start(startInfo);
                Debug.Log(craneMonitoringInterface.ProcessName + ".exe Start");
            }
            else
            {
                Debug.Log("Already started" + processName + ".exe");
            }
        }
        catch (Exception)
        {
        }
    }

    private void OnDestroy() 
    {
        if(craneMonitoringInterface != null && !craneMonitoringInterface.HasExited)
        {
            craneMonitoringInterface.Kill();
            craneMonitoringInterface=null;
        }
    }
}
//~pjh