using UnityEngine;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using Unity.Collections;

public class ProcessCamera : MonoBehaviour
{
    public Camera mainCamera;
    public Camera subCamera1;
    public Camera subCamera2;
    public Camera subCamera3;

    [ReadOnly, SerializeField]
    private CraneObject TargetCrane;

    [ReadOnly, SerializeField]
    private Transform FaceCamera;
    [ReadOnly, SerializeField]
    private Transform LeftSideCamera;
    [ReadOnly, SerializeField]
    private Transform RightSideCamera;
    [ReadOnly, SerializeField]
    private Transform TopSideCamera;
    [ReadOnly, SerializeField]
    private Transform BackSideCamera;

    private Transform[] userCamera = new Transform[5];
    //pjh
    // private bool bUpdate = false;
    // private int angle = 0;
    //~pjh
    bool craneFollow = false;
    public void SaveUserCameraInfo(int index)
    {
        CameraPosition cameraPosition = new CameraPosition(mainCamera.transform.localPosition, mainCamera.transform.localRotation);

        BinaryFormatter formatter = new BinaryFormatter();
        string path = Application.dataPath + "/cameraInfoUser" + index + ".bin";

        FileStream stream = new FileStream(path, FileMode.Create);
        formatter.Serialize(stream, cameraPosition);
        stream.Close();

        TargetCrane.UserCamera[index].transform.position = mainCamera.transform.position;
        TargetCrane.UserCamera[index].transform.rotation = mainCamera.transform.rotation;
    }
    private void LoadUserCameraInfo()
    {
        TargetCrane.UserCamera[0].transform.position = FaceCamera.transform.position;
        TargetCrane.UserCamera[0].transform.rotation = FaceCamera.transform.rotation;
        TargetCrane.UserCamera[1].transform.position = LeftSideCamera.transform.position;
        TargetCrane.UserCamera[1].transform.rotation = LeftSideCamera.transform.rotation;
        TargetCrane.UserCamera[2].transform.position = RightSideCamera.transform.position;
        TargetCrane.UserCamera[2].transform.rotation = RightSideCamera.transform.rotation;
        TargetCrane.UserCamera[3].transform.position = TopSideCamera.transform.position;
        TargetCrane.UserCamera[3].transform.rotation = TopSideCamera.transform.rotation;
        TargetCrane.UserCamera[4].transform.position = BackSideCamera.transform.position;
        TargetCrane.UserCamera[4].transform.rotation = BackSideCamera.transform.rotation;

        for (int i=0; i< TargetCrane.UserCamera.Length; i++)
        {
            string path = Application.dataPath + "/cameraInfoUser" + i + ".bin";
            if (File.Exists(path))
            {
                BinaryFormatter formatter = new BinaryFormatter();
                FileStream stream = new FileStream(path, FileMode.Open);

                CameraPosition cameraPosition = formatter.Deserialize(stream) as CameraPosition;
                stream.Close();

                TargetCrane.UserCamera[i].transform.localPosition = cameraPosition.GetPosition();
                TargetCrane.UserCamera[i].transform.localRotation = cameraPosition.GetRotation();
                //Debug.Log(TargetCrane.UserCamera[i].transform.localPosition + " Load: " + cameraPosition.ToString());
            }
        }
    }

    public void OnChangeCameraFollow()
    {
        craneFollow = !craneFollow;
        Vector3 preCameraPosition = mainCamera.transform.position;
        Quaternion preCameraRotation = mainCamera.transform.rotation;
        SetCrane(TargetCrane);

        mainCamera.transform.rotation = preCameraRotation;
        mainCamera.transform.position = preCameraPosition;
    }
    public void SetCrane(CraneObject crane)
    {
        FaceCamera = crane.FaceCamera;
        LeftSideCamera = crane.LeftSideCamera;
        RightSideCamera = crane.RightSideCamera;
        TopSideCamera = crane.TopSideCamera;
        BackSideCamera = crane.BackSideCamera;

        TargetCrane = crane;

        mainCamera.transform.position = FaceCamera.transform.position;
        mainCamera.transform.rotation = FaceCamera.transform.rotation;
        if (craneFollow == false)
            mainCamera.transform.SetParent(crane.llcTower.transform);
        else
            mainCamera.transform.SetParent(crane.llcBody.transform);
        //LoadCameraInfo();

        LoadUserCameraInfo();
        userCamera[0] = TargetCrane.UserCamera[0];
        userCamera[1] = TargetCrane.UserCamera[1];
        userCamera[2] = TargetCrane.UserCamera[2];
        userCamera[3] = TargetCrane.UserCamera[3];
        userCamera[4] = TargetCrane.UserCamera[4];

        string textSelection = CsCore.Configuration.ReadConfigIni("Camera", "Selection");
        int cameraSelection = 0;
        int.TryParse(textSelection, out cameraSelection);
        switch(cameraSelection)
        {
            case 0:
                SetCameraFace();
                break;
            case 1:
                SetCameraLeft();
                break;
            case 2:
                SetCameraRight();
                break;
            case 3:
                SetCameraTop();
                break;
            case 4:
                SetCameraBack();
                break;

            case 5:
                SetCameraUser(0);
                break;
            case 6:
                SetCameraUser(1);
                break;
            case 7:
                SetCameraUser(2);
                break;
            case 8:
                SetCameraUser(3);
                break;
            case 9:
                SetCameraUser(4);
                break;
        }
    }

    public void RotateAround(float MouseX, float MouseY, int screenNum = 0)
    {
        // CYS 예외 처리.
        if (TargetCrane == null)
        {
            CsCore.IO.EventLog.Write("None Target Crane");
            return;
        }
        float mouseX = MouseX * 2.0f;
        float mouseY = MouseY * 2.0f;

        switch (screenNum)
        {
            case 0:
                float angleX = mainCamera.transform.eulerAngles.x;
                angleX = angleX > 180f ? angleX - 360f : angleX;
                float targetAngleX = Mathf.Clamp(angleX - mouseY, -80f, 80f);
                float deltaAngleX = targetAngleX - angleX;
                mainCamera.transform.RotateAround(TargetCrane.transform.position, mainCamera.transform.right, deltaAngleX);
                mainCamera.transform.RotateAround(TargetCrane.transform.position, mainCamera.transform.up, mouseX);
                break;
            case 1:
                float angleX1 = subCamera1.transform.eulerAngles.x;
                angleX1 = angleX1 > 180f ? angleX1 - 360f : angleX1;
                float targetAngleX1 = Mathf.Clamp(angleX1 - mouseY, -80f, 80f);
                float deltaAngleX1 = targetAngleX1 - angleX1;
                subCamera1.transform.RotateAround(TargetCrane.transform.position, subCamera1.transform.right, deltaAngleX1);
                subCamera1.transform.RotateAround(TargetCrane.transform.position, subCamera1.transform.up, mouseX);
                break;
            case 2:
                float angleX2 = subCamera2.transform.eulerAngles.x;
                angleX2 = angleX2 > 180f ? angleX2 - 360f : angleX2;
                float targetAngleX2 = Mathf.Clamp(angleX2 - mouseY, -80f, 80f);
                float deltaAngleX2 = targetAngleX2 - angleX2;
                subCamera2.transform.RotateAround(TargetCrane.transform.position, subCamera2.transform.right, deltaAngleX2);
                subCamera2.transform.RotateAround(TargetCrane.transform.position, subCamera2.transform.up, mouseX);
                break;
            case 3:
                float angleX3 = subCamera3.transform.eulerAngles.x;
                angleX3 = angleX3 > 180f ? angleX3 - 360f : angleX3;
                float targetAngleX3 = Mathf.Clamp(angleX3 - mouseY, -80f, 80f);
                float deltaAngleX3 = targetAngleX3 - angleX3;
                subCamera3.transform.RotateAround(TargetCrane.transform.position, subCamera3.transform.right, deltaAngleX3);
                subCamera3.transform.RotateAround(TargetCrane.transform.position, subCamera3.transform.up, mouseX);
                break;
            default:
                break;
        }
    }

    public void Rotate(float x, float y, int screenNum = 0)
    {
       switch (screenNum)
       {
           case 0:
               mainCamera.transform.Rotate(-y, x, 0);
               break;
           case 1:
               subCamera1.transform.Rotate(-y, x, 0);
               break;
           case 2:
               subCamera2.transform.Rotate(-y, x, 0);
               break;
           case 3:
               subCamera3.transform.Rotate(-y, x, 0);
               break;
           default:
               break;
       }
    }

    public void Move(float x, float y)
    {
        mainCamera.transform.position += -mainCamera.gameObject.transform.right * x * 3;
        mainCamera.transform.position += -mainCamera.gameObject.transform.up * y * 3;
    }

    public void Zoom(float zoom)
    {
        mainCamera.transform.Translate(Vector3.forward * zoom * 13);
    }

    public int GetScreenNum(Vector3 screenPosition)
    {
        int ret = -1;
        Vector3 viewPositionMain = mainCamera.ScreenToViewportPoint(screenPosition);
        Vector3 viewPositionSub1 = subCamera1.ScreenToViewportPoint(screenPosition);
        Vector3 viewPositionSub2 = subCamera2.ScreenToViewportPoint(screenPosition);
        Vector3 viewPositionSub3 = subCamera3.ScreenToViewportPoint(screenPosition);

        Rect rc = new Rect(0,0,1,1);
        bool containsMain = rc.Contains(viewPositionMain);
        bool containsSub1 = rc.Contains(viewPositionSub1);
        bool containsSub2 = rc.Contains(viewPositionSub2);
        bool containsSub3 = rc.Contains(viewPositionSub3);

        if(containsMain)
        {
            ret = 0;
        }
        else if (containsSub1)
        {
            ret = 1;
        }
        else if (containsSub2)
        {
            ret = 2;
        }
        else if (containsSub3)
        {
            ret = 3;
        }
        else
        {
        }
        return ret;
    }

    public void SetCameraDivision()
    {
        mainCamera.transform.position = FaceCamera.transform.position;
        mainCamera.transform.rotation = FaceCamera.transform.rotation;

        subCamera1.transform.position = LeftSideCamera.transform.position;
        subCamera1.transform.rotation = LeftSideCamera.transform.rotation;

        subCamera2.transform.position = RightSideCamera.transform.position;
        subCamera2.transform.rotation = RightSideCamera.transform.rotation;

        subCamera3.transform.position = TopSideCamera.transform.position;
        subCamera3.transform.rotation = TopSideCamera.transform.rotation;
        SetFourScreen(true);
    }

    public void SetCameraFace()
    {
        SetFourScreen(false);
        mainCamera.transform.position = FaceCamera.transform.position;
        mainCamera.transform.rotation = FaceCamera.transform.rotation;

        CsCore.Configuration.WriteConfigIni("Camera", "Selection", "0");
    }

    public void SetCameraLeft()
    {
        SetFourScreen(false);
        mainCamera.transform.position = LeftSideCamera.transform.position;
        mainCamera.transform.rotation = LeftSideCamera.transform.rotation;

        CsCore.Configuration.WriteConfigIni("Camera", "Selection", "1");
    }

    public void SetCameraRight()
    {
        SetFourScreen(false);
        mainCamera.transform.position = RightSideCamera.transform.position;
        mainCamera.transform.rotation = RightSideCamera.transform.rotation;

        CsCore.Configuration.WriteConfigIni("Camera", "Selection", "2");
    }

    public void SetCameraTop()
    {
        SetFourScreen(false);
        mainCamera.transform.position = TopSideCamera.transform.position;
        mainCamera.transform.rotation = TopSideCamera.transform.rotation;

        CsCore.Configuration.WriteConfigIni("Camera", "Selection", "3");
    }

    public void SetCameraBack()
    {
        SetFourScreen(false);
        mainCamera.transform.position = BackSideCamera.transform.position;
        mainCamera.transform.rotation = BackSideCamera.transform.rotation;

        CsCore.Configuration.WriteConfigIni("Camera", "Selection", "4");
    }

    public void SetCameraUser(int idx)
    {
        SetFourScreen(false);
        mainCamera.transform.position = userCamera[idx].transform.position;
        mainCamera.transform.rotation = userCamera[idx].transform.rotation;
        int val = idx + 5;
        CsCore.Configuration.WriteConfigIni("Camera", "Selection", val.ToString());
    }

    private void SetFourScreen(bool bFourScreen)
    {
        if(bFourScreen)
        {
            mainCamera.rect = new Rect(0.0f, 0.5f, 0.5f, 0.5f);
            subCamera1.rect = new Rect(0.5f, 0.5f, 0.5f, 0.5f);
            subCamera2.rect = new Rect(0.0f, 0.0f, 0.5f, 0.5f);
            subCamera3.rect = new Rect(0.5f, 0.0f, 0.5f, 0.5f);

            subCamera1.gameObject.SetActive(true);
            subCamera2.gameObject.SetActive(true);
            subCamera3.gameObject.SetActive(true);
        }
        else
        {
            mainCamera.rect = new Rect(0.0f, 0.0f, 1.0f, 1.0f);
            subCamera1.rect = new Rect(0.0f, 0.0f, 1.0f, 1.0f);
            subCamera2.rect = new Rect(0.0f, 0.0f, 1.0f, 1.0f);
            subCamera3.rect = new Rect(0.0f, 0.0f, 1.0f, 1.0f);
            
            subCamera1.gameObject.SetActive(false);
            subCamera2.gameObject.SetActive(false);
            subCamera3.gameObject.SetActive(false);
        }
    }

    [System.Serializable]
    public class CameraPosition
    {
        public float x;
        public float y;
        public float z;
        public float qx;
        public float qy;
        public float qz;
        public float qw;

        public CameraPosition(Vector3 position, Quaternion rotation)
        {
            x = position.x;
            y = position.y;
            z = position.z;
            qx = rotation.x;
            qy = rotation.y;
            qz = rotation.z;
            qw = rotation.w;
        }

        public Vector3 GetPosition()
        {
            return new Vector3(x, y, z);
        }

        public Quaternion GetRotation()
        {
            return new Quaternion(qx, qy, qz, qw);
        }

        public override string ToString()
        {
            return string.Format("position = ({0}, {1}, {2}) rotation = ({3}, {4}, {5}, {6})", x, y, z, qx, qy, qz, qw);
        }
    }
}