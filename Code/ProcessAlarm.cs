using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;
using CsCore;
using System.IO;
public enum AlarmState
{
    None, Alert, Warn, Danger, Stop
}

public enum AlarmUseState
{
    Use, UnUse
}

public enum AlarmSoundContState
{
    Continous, Uncontinous
}

public class ProcessAlarm : MonoBehaviour
{
    public Color screenColor1;
    public Color screenColor2;
    public Color screenColor3;
    public Color screenColorAlert;
    public Color craneColorOriginal;
    public Color craneColor1;
    public Color craneColor2;
    public Color craneColor3;
    public Color craneColorAlert;

    private Color screenColor;
    private Color craneColor;

    private List<MeshRenderer> craneList;

    public AlarmState alarmState { get; private set; }
    public AlarmUseState alarmUseState { get; private set; }
    private AlarmSoundContState alarmContState;
      //pjh
    bool alarmLoop{
        get { return alarmContState == AlarmSoundContState.Continous;}
    }
    //~pjh

    private AudioSource alarmSound;

    public AudioClip clip1;
    public AudioClip clip2;
    public AudioClip clip3;
    public AudioClip clipAlert;
    private GameObject screenObject;
    
    private string fileConfig1;
    private string fileConfig2;
    private string fileConfig3;
    private string fileConfigAlert;

    private bool bUpdateAlarmState = false;
    private bool bAlarmChanged = false;
    private bool bUpdateAlarmUseState = false;
    private float playTimeUntill = float.MaxValue;

    private void Awake()
    {
        screenObject = GameObject.Find("ScreenAlarm");
        alarmSound = GetComponent<AudioSource>();

        alarmState = AlarmState.None;
        alarmUseState = AlarmUseState.UnUse;
        alarmContState = AlarmSoundContState.Uncontinous;

        screenColor = screenColor1;
        craneColor = craneColor1;

        fileConfig1 = Configuration.ReadConfigIni("AlarmSound", "sound1");
        fileConfig2 = Configuration.ReadConfigIni("AlarmSound", "sound2");
        fileConfig3 = Configuration.ReadConfigIni("AlarmSound", "sound3");
        fileConfigAlert = Configuration.ReadConfigIni("AlarmSound", "soundAlarm");

         //pjh
        string folderPath = Application.dataPath + "/AlarmSound";
        if (!Directory.Exists(folderPath))
        {
            //���丮�� �������� �ʴٸ� ������ �ʿ䰡 ������ �ٷ� �Լ� ����
            Directory.CreateDirectory(folderPath);
        }
        else
        {
            string[] files = System.IO.Directory.GetFiles(folderPath, "*.wav");
            //string[] files = System.IO.Directory.GetFiles(Application.dataPath + "/AlarmSound", "*.wav");


            string[] names = new string[files.Length];
            for (int i = 0; i < files.Length; i++)
            {
                char[] separators = new char[] { '\\', '/' };
                string[] segment = files[i].Split(separators);

                if (segment.Length > 0)
                {
                    names[i] = segment[segment.Length - 1];
                }
                StartCoroutine(LoadAudioClip(files[i], names[i]));
            }
        }
        //~pjh
    }

    IEnumerator LoadAudioClip(string path, string name)
    {
        using (UnityWebRequest www = UnityWebRequestMultimedia.GetAudioClip(path, AudioType.WAV))
        {
            yield return www.SendWebRequest();

            if (www.result == UnityWebRequest.Result.ConnectionError)
            {
                Debug.Log(www.error);
            }
            else
            {
                AudioClip clip = DownloadHandlerAudioClip.GetContent(www);

                if (string.Compare(fileConfig1, name) == 0)
                {
                    clip1 = clip;
                }
                else if (string.Compare(fileConfig2, name) == 0)
                {
                    clip2 = clip;
                }
                else if (string.Compare(fileConfig3, name) == 0)
                {
                    clip3 = clip;
                }
                else if (string.Compare(fileConfigAlert, name) == 0)
                {
                    clipAlert = clip;
                }
                else
                {
                }
            }
        }
    }

    private void UpdateScreen(Color color)
    {
        if (((int)UnityEngine.Time.fixedTime) % 2 == 0)
        {
            screenObject.GetComponent<Image>().color = color;
            screenObject.SetActive(true);
        }
        else
        {
            screenObject.SetActive(false);
        }
    }

    private void UpdateCranecolor(Color color)
    {
        //pjh
        if(craneList==null) 
        {
            Debug.LogWarning("craneList is null");
            return;
        }
        //~pjh
        if (((int)UnityEngine.Time.fixedTime) % 2 == 0)
        {
            for (int i = 0; i < craneList.Count; i++)
            {
                if(craneList[i] != null)
                    craneList[i].materials[0].color = color;
            }
        }
        else
        {
            for (int i = 0; i < craneList.Count; i++)
            {
                if (craneList[i] != null)
                    craneList[i].materials[0].color = craneColorOriginal;
            }
        }
    }

    void Update()
    {
        if(bUpdateAlarmUseState)
        {
            if (alarmUseState == AlarmUseState.UnUse)
            {
                alarmSound.Stop();
            }
            else
            {
                alarmSound.Play();
                ResetPlayTime(5);
            }
            bUpdateAlarmUseState = false;
        }

        if(bUpdateAlarmState)
        {
            if (bAlarmChanged)
            {
                ResetPlayTime(5);
                bAlarmChanged = false;
            }

            switch (alarmState)
            {
                case AlarmState.None:
                    alarmSound.clip = clip1;
                    alarmSound.Stop();
                    break;
                case AlarmState.Warn:
                    alarmSound.clip = clip1;
                    screenColor = screenColor1;
                    craneColor = craneColor1;
                    if (alarmSound.isPlaying == false && alarmUseState == AlarmUseState.Use && playTimeUntill > Time.time)
                    {
                        alarmSound.Play();
                    }
                    break;
                case AlarmState.Danger:
                    alarmSound.clip = clip2;
                    screenColor = screenColor2;
                    craneColor = craneColor2;
                    if (alarmSound.isPlaying == false && alarmUseState == AlarmUseState.Use && playTimeUntill > Time.time)
                    {
                        alarmSound.Play();
                    }
                    break;
                case AlarmState.Stop:
                    alarmSound.clip = clip3;
                    screenColor = screenColor3;
                    craneColor = craneColor3;
                    if (alarmSound.isPlaying == false && alarmUseState == AlarmUseState.Use && playTimeUntill > Time.time)
                    {
                        alarmSound.Play();
                    }
                    break;
                case AlarmState.Alert:
                    alarmSound.clip = clipAlert;
                    screenColor = screenColorAlert;
                    craneColor = craneColorAlert;
                    if (alarmSound.isPlaying == false && alarmUseState == AlarmUseState.Use && playTimeUntill > Time.time)
                    {
                        alarmSound.Play();
                    }
                    break;
                default:
                    alarmSound.Stop();
                    break;
            }

            bUpdateAlarmState = false;
        }

        if (alarmUseState == AlarmUseState.Use)
        {
            if (alarmState != AlarmState.None)
            {
                UpdateScreen(screenColor);

                UpdateCranecolor(craneColor);

                if (Time.time > playTimeUntill)
                {
                    alarmSound.Stop();
                }
            }
            else
            {
                //pjh
                if(craneList==null) 
                {
                    Debug.LogWarning("craneList is null");
                }
                else
                {
                    for (int i = 0; i < craneList.Count; i++)
                    {
                        if (craneList[i] != null)
                            craneList[i].materials[0].color = craneColorOriginal;
                    }
                }
                //~pjh

                screenObject.SetActive(false);
                alarmSound.Stop();
            }
        }
        else
        {
              //pjh
                if(craneList==null) 
                {
                    Debug.LogWarning("craneList is null");
                }
                else
                {
            for (int i = 0; i < craneList.Count; i++)
                {
                    if (craneList[i] != null)
                        craneList[i].materials[0].color = craneColorOriginal;
            }
                }
                //
            screenObject.SetActive(false);
            alarmSound.Stop();
        }
    }

    public void UpdateTargetCraneMosel(List<MeshRenderer> craneList)
    {
        this.craneList = craneList;
    }

    public void UpdateAlarmState(AlarmState state)
    {
        switch (alarmState)
        {
            case AlarmState.None:
                if (state != AlarmState.None)
                {
                    bAlarmChanged = true;
                }
                break;
            case AlarmState.Alert:
                if (state == AlarmState.Warn || state == AlarmState.Danger || state == AlarmState.Stop)
                {
                    bAlarmChanged = true;
                }
                break;
            case AlarmState.Warn:
                if (state == AlarmState.Danger || state == AlarmState.Stop)
                {
                    bAlarmChanged = true;
                }
                break;
            case AlarmState.Danger:
                if (state == AlarmState.Stop)
                {
                    bAlarmChanged = true;
                }
                break;
            case AlarmState.Stop:
                break;
        }
        alarmState = state;
        bUpdateAlarmState = true;
    }

    public void UpdateAlarmUseState(AlarmUseState alarmUseState)
    {
        if(this.alarmUseState != alarmUseState)
        {
            this.alarmUseState = alarmUseState;
            bUpdateAlarmUseState = true;
        }
    }

    public void SetSoundContState(AlarmSoundContState SoundState)
    {
        alarmContState = SoundState;
        ResetPlayTime(5);
    }

    public void ResetPlayTime(int inTime)
    {
        if(alarmContState == AlarmSoundContState.Continous)
        {
            playTimeUntill = float.MaxValue;
        }
        else
        {
            playTimeUntill = Time.time + 5;
        }
    }


}
