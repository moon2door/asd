using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System.ComponentModel;
using Unity.Collections;

public class CraneObject : MonoBehaviour
{
    #region Variable
    [Header("original")]
    // 피어 이름
    public string pier;
    // 피어 에서 크레인 고유 번호
    public string type;
    // 피어 이름(설치 위치)
    public int pierNum;
    // 피어에서 크레인 고유 번호
    public int craneNum;
    // 크레인 이름
    public string craneName;
    // 센서 개수
    public int numRotor;
    [SerializeField]
    public Transform FaceCamera;
    [SerializeField]
    public Transform LeftSideCamera;
    [SerializeField]
    public Transform RightSideCamera;
    [SerializeField]
    public Transform TopSideCamera;
    [SerializeField]
    public Transform BackSideCamera;
    [SerializeField]
    public Transform[] UserCamera = new Transform[5];
    [SerializeField]
    public List<MeshRenderer> craneModel;
    [SerializeField]// 포인트 설치되는 중심점.
    public GameObject pointOrigin;
    [SerializeField] Vector3 pointCloudPositionOffset;
    [SerializeField] float pointCloudRotationOffset;
    [SerializeField]
    public GameObject llcJib;
    [SerializeField]
    public GameObject llcTower;
    [SerializeField]
    public GameObject llcBody;
    [SerializeField]
    public ReferancePosition pierReferancePosition;
    [SerializeField]
    public float gpsAngleOffset;
    public float jibAngleOffset;
    [SerializeField] private float highAngle = 45f;
    float _lastHighAngle = 0;
    float _lastHighAngleTime = 0;
    private float azimuth;
    private float height;
    float ratioHeight = 0.89f;

    private bool bUpdateAttitude;
    private bool bUpdatePosition;
    
    //pjh
    [Space]
    [Header("AngleOffset_PJH")]
    [SerializeField]Vector3 bodyRotateDirection = Vector3.forward;

    [Space]
    [Header("GPS Adjust_PJH")]
    [SerializeField]Vector2D_Double currentGPS;
    [SerializeField] float clampMin = .1f, clampMax =.9f , clampValue;
    [SerializeField] Vector3 cranePositionOffset;

    Vector3 startUnity;
    Vector3 endUnity;
    // 시작과 끝 GPS 좌표의 거리 계산
    float totalDistance ;

    [HideInInspector] public Vector3 gpsMap;

    //~pjh
    // CSY
    // Pier(안벽)에 따른 구분 중에 선상(배 위) 크레인의 경우 움직이지 않고 고정되어 처리되고 있어. 
    // GPS값에 의한 위치 이동을 제한해야한다.
    // 이를 위해 GPS 이동 제한을 위해서 구분 값 추가.
    public bool bAbleMove = true;

    // 거리 측정 제거 영역
    public Transform[] exceptionTower;
    public Transform[] exceptionJIB;

    //GPS 컬리티
    public int gpsQuilty = -1;
    #endregion
    private void Awake()
    {
        string name = CsCore.Configuration.ReadConfigIni("CraneType", "Name", "");
        if (string.IsNullOrEmpty(name) == false)
            craneName = name;

        exceptionPosList.Clear();
        if(exceptionTower.Length == 2)
        {
            (Vector3, Vector3,float) pos = (exceptionTower[0].localPosition, exceptionTower[1].localPosition,5);
            exceptionPosList.Add(pos);
        }
        if(exceptionJIB.Length == 2)
        {
            (Vector3, Vector3, float) pos = (exceptionJIB[0].localPosition, exceptionJIB[1].localPosition, 2.7f);
            exceptionPosList.Add(pos);
        }

    }
    private void Start()
    {
        bUpdateAttitude = false;
        bUpdatePosition = false;

        if(string.IsNullOrEmpty(pier)== false)
            bAbleMove = CraneUtility.isAbleMovePier(pier);

        pierReferancePosition.gameObject.SetActive(true);
        //pjh
        gpsAngleOffset = float.Parse(CsCore.Configuration.ReadConfigIni("GPSOffset", "BodyRotate", gpsAngleOffset.ToString()));
        jibAngleOffset = float.Parse(CsCore.Configuration.ReadConfigIni("CraneOffset", "JibAngle", jibAngleOffset.ToString()));
        cranePositionOffset = CsCore.Configuration.StringToVector3(CsCore.Configuration.ReadConfigIni("CraneOffset", "CranePosition", cranePositionOffset.x.ToString()+ ","+cranePositionOffset.y.ToString()+","+cranePositionOffset.z.ToString()));

        pointCloudPositionOffset = CsCore.Configuration.StringToVector3(CsCore.Configuration.ReadConfigIni("pointCloudOffset", "Position", pointCloudPositionOffset.x.ToString()+ ","+pointCloudPositionOffset.y.ToString()+","+pointCloudPositionOffset.z.ToString()));
        pointCloudRotationOffset = float.Parse(CsCore.Configuration.ReadConfigIni("pointCloudOffset", "Rotate", pointCloudRotationOffset.ToString()));

        if(pointOrigin)
        {
            var localOffset = pierReferancePosition.transform.TransformDirection(pointCloudPositionOffset);
            pointOrigin.transform.localPosition += localOffset;

            pointOrigin.transform.localRotation = Quaternion.Euler(pointOrigin.transform.localRotation.eulerAngles.x, pointOrigin.transform.localRotation.eulerAngles.y, pointCloudRotationOffset);
        }

        //InitSettingUnityPosition();
        //startUnity = ReferenceGPSToUnity(pierReferancePosition.startPoint);
        //endUnity = ReferenceGPSToUnity(pierReferancePosition.endPoint);
        //// 시작과 끝 GPS 좌표의 거리 계산
        //totalDistance = Vector3.Distance(startUnity, endUnity);
    }
    // 피어에 따른 Ini 설정 적용.
    public void InitSettingUnityPosition()
    {
        if (string.IsNullOrEmpty(pier) == true)
            return;
        // 해당 메소드에서 config의 pier 같을 때에만 point들이 조정됨.
        pierReferancePosition.BeginStartPosition(pier);
        startUnity = ReferenceGPSToUnity(pierReferancePosition.startPoint);
        endUnity = ReferenceGPSToUnity(pierReferancePosition.endPoint);
        // 시작과 끝 GPS 좌표의 거리 계산
        totalDistance = Vector3.Distance(startUnity, endUnity);
    }

    private void OnDisable() {
        pierReferancePosition.gameObject.SetActive(false);
    }

    #if UNITY_EDITOR
      [Space]
    [Header("ForTest_PJH")]
    [SerializeField] Vector2D_Double testGPS1;
    [SerializeField] Vector2D_Double testGPS2;
    [SerializeField] float t_azimuth;
    [SerializeField] float t_highAngle;
    [SerializeField] bool t_autoUpdate;

#endif
    void Update()
    {
    #if UNITY_EDITOR
        if(Input.GetKeyDown(KeyCode.Alpha1))
        {
            SetPosition(testGPS1.x, testGPS1.y, 0);
        }
        if(Input.GetKeyDown(KeyCode.Alpha2))
        {
            SetPosition(testGPS2.x, testGPS2.y,0);
        }
        if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            if(hockROI!=null && hockROI.Length >= 5)
                CreateROI(hockROI[0],hockROI[1],hockROI[2],hockROI[3],hockROI[4],hockROI[5]);
            if (craneROI != null && craneROI.Length >= 5)
                CreateROI(craneROI[0], craneROI[1], craneROI[2], craneROI[3], craneROI[4], craneROI[5]);
        }
        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (Input.GetKeyDown(KeyCode.Alpha3))
            {
                ProcessManager.instance.drawLiner.ResetDraw();
            }
        }
        if (t_autoUpdate)SetCraneAttitude(t_highAngle, t_azimuth);
        // 크레인 궤도 시각화 도구
        //if ((bUpdateAttitude || bUpdatePosition) && testObj != null) Instantiate(testObj, transform.position, Quaternion.identity);
#endif
        if ((bUpdateAttitude || bUpdatePosition) && gpsQuilty == 4 && ProcessManager.instance != null) ProcessManager.instance.gpsOffsetSetting.AddSample(transform.position, llcBody.transform.localRotation.eulerAngles.z);
        if (bUpdateAttitude)
        {
            //pjh
            // 기존 localRotation에서 X, Y 축을 유지하고 Z 축 회전만 변경
            Quaternion currentRotation = llcBody.transform.localRotation;

            // 기존 회전에서 Z 축을 분리하고 새로운 Z 축 회전 값 적용
            float newZRotation = azimuth + gpsAngleOffset; // Z 축 회전 값
            //float newZRotation = azimuth; // Z 축 회전 값
            Quaternion newRotation = Quaternion.Euler(currentRotation.eulerAngles.x, currentRotation.eulerAngles.y, newZRotation);

            // 새로 계산한 Z 축 회전을 적용
            llcBody.transform.localRotation = newRotation;
            //~pjh
            // 
            bool validAngle = isValidJibAngle();
            llcJib.transform.localRotation = Quaternion.Euler(new Vector3(highAngle + jibAngleOffset, 0, 0));

            bUpdateAttitude = false;
        }

        if (bUpdatePosition)
        {
            var localOffset = pierReferancePosition.transform.TransformDirection(cranePositionOffset); 
            // xz 평면에서의크레인 움직임 적용.
            Vector2 startXZ = new Vector2(pierReferancePosition.transform.position.x,pierReferancePosition.transform.position.z);
            Vector2 endXZ = new Vector2(pierReferancePosition.endPointUnityPosition.position.x,pierReferancePosition.endPointUnityPosition.position.z);
            Vector2 interpolatedXZ = Vector2.Lerp(startXZ, endXZ, clampValue);

            // GPS 고도에 따른 높이 값.
            float highCrane = height * ratioHeight;

            // 보간된 XZ 값과 고도의 조합
            Vector3 interpolatedUnity
                = new Vector3(interpolatedXZ.x, highCrane, interpolatedXZ.y);
                //= new Vector3(gpsMap.x, highCrane, gpsMap.z);

            if (bAbleMove == true)
            {
                //transform.position = interpolatedUnity + localOffset - CraneGpsOffset(pierNum,craneNum);
                transform.position = interpolatedUnity + localOffset;
            }
            bUpdatePosition = false;
        }
    }
    public void CreateROI(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
    {
        ProcessManager.instance.drawLiner.DrawBoundingBox( minX,  maxX,  minY,  maxY,  minZ,  maxZ, pointOrigin.transform,Color.red);
    }

    public void SetCraneAttitude(float highAngle, float azimuth)
    {
        this.highAngle = highAngle;
        //this.azimuth = azimuth + gpsAngleOffset;
        this.azimuth = azimuth;
        bUpdateAttitude = true;
    }
    public void SetGPS(int gps)
    {
        this.gpsQuilty = gps;
    }


    List<(Vector3, Vector3,float)> exceptionPosList = new List<(Vector3, Vector3, float)>();
    // ENI TC 중 모델에 누락된 부분이 있어 해당 적용 전까지 사용할 예정
    // 특정 오브젝트가 있을때 해당 오브젝트 경로에서 round의 반경을 가지고 있는 distance 알람 을 누락하는 기능.
    public bool ExceptionDistance(CsCore.Data.CollisionProcessor.StDistance data)
    {
        for(int i = 0; i < exceptionPosList.Count; i++)
        {
            (Vector3, Vector3, float) exceptionPos = exceptionPosList[i];

            float round = exceptionPos.Item3;

            Vector3 startPoint = new Vector3(data.xCrane, data.yCrane, data.zCrane);
            bool startCheck = false;
            Vector3 AB = exceptionPos.Item2 - exceptionPos.Item1;
            float abLen = AB.magnitude;
            Vector3 AP = startPoint - exceptionPos.Item1;
            float area2 = Vector3.Cross(AB, AP).magnitude;
            float startDis=  area2 / abLen;
            if (startDis  < round)
                startCheck = true;
            
            Vector3 endPoint = new Vector3(data.xCluster, data.yCluster, data.zCluster);
            AB = exceptionPos.Item2 - exceptionPos.Item1;
            abLen = AB.magnitude;
            AP = endPoint - exceptionPos.Item1;
            area2 = Vector3.Cross(AB, AP).magnitude;
            float endDis = area2 / abLen;
            bool endCheck = false;
            if (endDis < round)
                endCheck = true;

            if (startCheck && endCheck == true)
                return true;
        }
        return false;
    }

    //pjh
    Vector3 ReferenceGPSToUnity(Vector2D_Double gps)
    {
        var earthRadius = 6371000f;

        // 위도와 경도를 라디안으로 변환
        var latRad = gps.x * Mathf.Deg2Rad;
        var lonRad = gps.y * Mathf.Deg2Rad;
        var lat0Rad = pierReferancePosition.startPoint.x * Mathf.Deg2Rad;
        var lon0Rad = pierReferancePosition.startPoint.y * Mathf.Deg2Rad;
        
        // X, Y 좌표 계산 (적절한 축척 적용)
        var x = earthRadius * (lonRad - lon0Rad) * Mathf.Cos((float)lat0Rad);
        var y = earthRadius * (latRad - lat0Rad);

        return new Vector3((float)x, 0, (float)y);  // Unity는 y축이 높이임
    }

    public float InterpolateGPS(Vector2D_Double currentGPS)
    {
        Vector3 currentUnity = ReferenceGPSToUnity(currentGPS);

        // 현재 GPS와 시작 GPS 사이의 거리 계산
        float currentDistance = Vector3.Distance(startUnity, currentUnity);

        // 비율 계산
        double ratio = currentDistance / totalDistance;
        //Debug.Log(currentDistance + " / " + totalDistance);
        // 비율이 0~1 범위에 있도록 클램- .5f프
        float clampValue = Mathf.Clamp((float)ratio, clampMin, clampMax);
        //Debug.Log(totalDistance + " / "+ currentDistance + " / "+ ratio);

        return clampValue;
    }
    //~pjh
    float[] hockROI;
    float[] craneROI;
    public void SetROI(float[] hockROI, float[] craneROI)
    {
        this.hockROI = hockROI;
        this.craneROI = craneROI;
    }

    public void SetPosition(double latitude, double longitude, float height)
    {
        this.height = height;
        //pjh
        var _currentGPS  = new Vector2D_Double(CoordinateConverter.ConvertToDecimal(latitude.ToString()), CoordinateConverter.ConvertToDecimal(longitude.ToString(), false));//ReferenceGPSToUnity(CoordinateConverter.ConvertToDecimal(latitude.ToString()), CoordinateConverter.ConvertToDecimal(longitude.ToString()));
        Vector3 map= GPS2Map.GpsToUnity(new Vector3((float)_currentGPS.x, (float)_currentGPS.y));
        gpsMap = map;
        if (currentGPS==_currentGPS)return;
        //Debug.Log(latitude + " / "+ longitude + "\n"+ _currentGPS.x + " / " + _currentGPS.y);
        currentGPS= _currentGPS;
        clampValue = InterpolateGPS(_currentGPS);

        this.height = height;

        bUpdatePosition = true;
        return;
    }

    // 크레인에 설치된 GPS의 위치에 따른 offset 적용.
    // 적용되는 GPS와 유니티 맵과의 스케일 차이가 있어 Pier에 대한 Offset 추가.
    Vector3 CraneGpsOffset(int pierId, int craneId)
    {
        Vector3 gpsOffset = Vector3.zero;
        gpsOffset = CraneUtility.CraneGPSOffset(pierId, craneId);
        Vector3 pierOffset = Vector3.zero;
        pierOffset = CraneUtility.CranePierOffset(pierId, craneId);
        // 통합관제의 값을 들고 오지만, 통합관제의 월드는 75배율이며 모니터링은 100배율을 가지고 있어 해당 값으로 수정한다.
        Vector3 angle = llcBody.transform.localRotation.eulerAngles * Mathf.Deg2Rad;
        float x = gpsOffset.x * Mathf.Cos(angle.z) - gpsOffset.z * Mathf.Sin(angle.z);
        float z = gpsOffset.x * Mathf.Sin(angle.z) + gpsOffset.z * Mathf.Cos(angle.z);
        gpsOffset = new Vector3(x, 0, z);
        return gpsOffset + pierOffset;
    }
    bool isValidJibAngle()
    {
        bool isValid = false;
        // 마지막 각도와 30도 차이나면 오류 값이라고 판정하여 마지막 입력값으로 수정.
        // 5초 동안 해당 현상 발생시 해당 현상이 정규 값이라고 판정하여 해당 현상으로 진행.
        if ((_lastHighAngle != 0 && Math.Abs(_lastHighAngle - highAngle) > 30) && _lastHighAngleTime < 5)
        {
            highAngle = _lastHighAngle;
            _lastHighAngleTime += Time.deltaTime;
        }
        else
        {
            isValid = true;
            _lastHighAngle = highAngle;
            _lastHighAngleTime = 0;
        }
        return isValid;
    }
}
