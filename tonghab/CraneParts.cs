using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;

//pjh
public class CraneParts : MonoBehaviour
{
    // [추가] 타워의 초기 회전값을 저장할 변수 선언
    Vector3 initCraneTowerEulerAngles;

    #region Variable
    public enum CraneType
    {
        LLC,
        GC,
        TC,
        TTC
    }
    public int craneIndex;
    public CraneType craneType = CraneType.LLC;
    [Header("Crane Data")]
    public bool isWork;

    public int pierCode;
    public int craneCode;
    public string pierName;
    public string craneName;
    public int sensorCount;
    public GameObject pointCloudTransform;
    public bool useReverseRotator;
    [SerializeField] bool useReverseRotateCrane;

    [SerializeField] ReferencePosition refPosition;
    [Header("Offset")]
    public float pierDir;
    public Vector3 cranePositionOffset;
    public float bodyAngleOffset;
    public float jibAngleOffset;
    [SerializeField] float clampMin = .1f, clampMax =.9f , clampValue;
    [SerializeField] Vector3 pointCloudPositionOffset;
    [SerializeField] float pointCloudRotationOffset;
   [Header("Public Crane Parts")]
    [SerializeField] Transform craneJib;
    [SerializeField] Transform craneTower;
    [SerializeField] Transform cranwTowerJoint;

    [Header("LLC Crane Parts")]
    [SerializeField] Transform craneBody;
    public Transform CraneBody{
        get 
        {
            return craneBody;
        }
    }
    Vector3 initCraneBodyEulerAngles;

    [Header("GC Crane Parts")]
    [SerializeField] Transform operationRoom;
    [SerializeField] Transform trolly;
    [SerializeField] Transform hook1High;
    [SerializeField] Transform hook1Low;
    [SerializeField] Transform hook2High;
    [SerializeField] Transform hook2Low;
    [SerializeField] Transform hook3High;
    [SerializeField] Transform hook3Low;


    [Header("Debug")]
    [SerializeField] bool debugMode;
    [SerializeField] Vector2D_Double testGPS1, testGPS2;
    [SerializeField] float t_azimuth, t_highAngle;

    [SerializeField] Vector2D_Double currentGPS;

    [Header("Exception")]
    [SerializeField] public List<Transform> exceptionTower = new List<Transform>();
    [SerializeField] public List<Transform> exceptionJIB = new List<Transform>();

    public bool isException = true;
    List<(Vector3, Vector3, float)> exceptionPosList = new List<(Vector3, Vector3, float)>();

    Vector3 defaultPointCloudRotation = Vector3.right * 270f + Vector3.up * 180f;

    Vector3 startUnity;
    Vector3 endUnity;
    // 시작과 끝 GPS 좌표의 거리 계산
    float totalDistance ;
    #endregion
    private void Awake()
    {
        exceptionPosList.Clear();
        isException = false;
        if (exceptionTower.Count == 2)
        {
            (Vector3, Vector3, float) pos = (exceptionTower[0].localPosition, exceptionTower[1].localPosition, 10f);
            exceptionPosList.Add(pos);
            isException = true;
        }
        if (exceptionJIB.Count == 5)
        {
            (Vector3, Vector3, float) pos = (exceptionJIB[0].localPosition, exceptionJIB[1].localPosition, 5f);
            exceptionPosList.Add(pos);
            isException = true;
        }

    }
    private void Start()
    {
        if(useReverseRotateCrane) pointCloudTransform.transform.parent = craneTower.transform;
        initCraneBodyEulerAngles = craneBody != null ? craneBody.transform.localEulerAngles : Vector3.zero;

        // [추가] 타워 초기 회전값 저장
        initCraneTowerEulerAngles = craneTower.localEulerAngles;

        defaultPointCloudRotation = pointCloudTransform.transform.localEulerAngles;
        if(refPosition==null)return;
        startUnity = ReferenceGPSToUnity(refPosition.startPoint);
        endUnity = ReferenceGPSToUnity(refPosition.endPoint);
        // 시작과 끝 GPS 좌표의 거리 계산
        totalDistance = Vector3.Distance(startUnity, endUnity);

    }

    public void SetActivePointCloud()
    {
        pointCloudTransform.gameObject.SetActive(isWork);
    }

    public void ReadConfigOffset()
    {
        cranePositionOffset = CsCore.Configuration.StringToVector3(CsCore.Configuration.ReadConfigIni("CranePositionOffset", pierName+"_"+craneName, cranePositionOffset.x.ToString()+ ","+cranePositionOffset.y.ToString()+","+cranePositionOffset.z.ToString()));
        bodyAngleOffset = float.Parse(CsCore.Configuration.ReadConfigIni("CraneBodyAngleOffset", pierName+"_"+craneName, "0"));
        jibAngleOffset = float.Parse(CsCore.Configuration.ReadConfigIni("CraneJibAngleOffset", pierName+"_"+craneName, "0"));
        pointCloudPositionOffset = CsCore.Configuration.StringToVector3(CsCore.Configuration.ReadConfigIni("pointCloudPositionOffset", pierName+"_"+craneName, pointCloudPositionOffset.x.ToString()+ ","+pointCloudPositionOffset.y.ToString()+","+pointCloudPositionOffset.z.ToString()));
        pointCloudRotationOffset = float.Parse(CsCore.Configuration.ReadConfigIni("pointCloudRotationOffset", pierName+"_"+craneName, "0"));
    }
    public void SetJibAngle(float angle)
    {
        craneJib.localRotation = Quaternion.AngleAxis(angle + jibAngleOffset, new Vector3(1, 0, 0));
    }

    public void SetCraneRotate(float rotate)
    {
        Debug.Log(rotate);
        craneTower.localPosition = new Vector3();
        craneTower.localRotation = new Quaternion();

        if(craneBody)
        {
            craneBody.transform.localEulerAngles = initCraneBodyEulerAngles;
            craneBody.transform.Rotate(0f, 0f, ((useReverseRotateCrane ? -1 : 1) * rotate) + bodyAngleOffset, Space.Self);
            
        }
        else craneTower.RotateAround(cranwTowerJoint.position, Vector3.up, useReverseRotateCrane ? -1 : 1 * (rotate + pierDir));
        SetPointCloudTransformFromOffset();
    }

    public void SetGCTowerTransform(Vector3 operationPos, Vector3 trollyPos, Vector3 hookPos1, Vector3 hookPos2, Vector3 hookPos3)
    {
        operationRoom.localPosition = operationPos;
        trolly.localPosition = trollyPos;

        if(hookPos1 != Vector3.zero)
        {
            hook1High.localPosition = new Vector3(hookPos1.x, hookPos1.y, 0);
            hook1Low.localPosition = new Vector3(hookPos1.x, hookPos1.y, hookPos1.z);
        }
        if(hookPos2 != Vector3.zero)
        {
            hook2High.localPosition = new Vector3(hookPos2.x, hookPos2.y, 17.5f);
            hook2Low.localPosition = new Vector3(hookPos2.x, hookPos2.y, hookPos2.z);
        }
        if(hookPos3 != Vector3.zero)
        {
            hook3High.localPosition = new Vector3(hookPos3.x, hookPos3.y, 17.5f);
            hook3Low.localPosition = new Vector3(hookPos3.x, hookPos3.y, hookPos3.z);
        }
    }


    #if UNITY_EDITOR
    void Update()
    {
        if(debugMode)
        {
            if(Input.GetKeyDown(KeyCode.Alpha1))
            {
                SetPosition(testGPS1.x, testGPS1.y, t_azimuth, t_highAngle);
            }
            if(Input.GetKeyDown(KeyCode.Alpha2))
            {
                SetPosition(testGPS2.x, testGPS2.y, t_azimuth, t_highAngle);
            }
        }
        //SetPointCloudTransformFromOffset();
    }
#endif

    void SetPointCloudTransformFromOffset()
    {
        pointCloudTransform.transform.localPosition = pointCloudPositionOffset;

        pointCloudTransform.transform.localEulerAngles = defaultPointCloudRotation;

        // [수정] 회전 보정 로직 개선: CraneBody 차이 + CraneTower 차이 합산
        float bodyAngleDiff = (craneBody != null) ? (initCraneBodyEulerAngles.z - craneBody.localEulerAngles.z) : 0f;
        float towerAngleDiff = (initCraneTowerEulerAngles.y - craneTower.localEulerAngles.y); // 타워는 Y축 회전 기준

        float totalAngleDiff = bodyAngleDiff + towerAngleDiff;

        pointCloudTransform.transform.Rotate(0f, (useReverseRotator ? totalAngleDiff * float.Parse(CsCore.Configuration.ReadConfigIni("PointCloudRotateAmount", "Amount", "1")) : 0f) + pointCloudRotationOffset, 0f, Space.Self);
    }

    public void SetPosition(double latitude, double longitude, float azimuth, float highAngle)
    {
        SetCraneRotate(azimuth);
        SetJibAngle(highAngle);

        if (refPosition!=null)
        {
            var _currentGPS  = new Vector2D_Double(CoordinateConverter.ConvertToDecimal(latitude.ToString()), CoordinateConverter.ConvertToDecimal(longitude.ToString(),false));//ReferenceGPSToUnity(CoordinateConverter.ConvertToDecimal(latitude.ToString()), CoordinateConverter.ConvertToDecimal(longitude.ToString()));
            if(currentGPS ==_currentGPS)return;
            currentGPS = _currentGPS;
            clampValue = InterpolateGPS(_currentGPS);
            var localOffset = refPosition.transform.TransformDirection(cranePositionOffset);
            Vector3 interpolatedUnity = Vector3.Lerp(refPosition.transform.position, refPosition.endPointUnityPosition.position, clampValue);
            transform.position = interpolatedUnity + localOffset;
        }
     }

    Vector3 ReferenceGPSToUnity(Vector2D_Double gps)
    {
        var earthRadius = 6371000f;

        // 위도와 경도를 라디안으로 변환
        var latRad = gps.x * Mathf.Deg2Rad;
        var lonRad = gps.y * Mathf.Deg2Rad;
        var lat0Rad = refPosition.startPoint.x * Mathf.Deg2Rad;
        var lon0Rad = refPosition.startPoint.y * Mathf.Deg2Rad;
        
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

    // ENI TC 중 모델에 누락된 부분이 있어 해당 적용 전까지 사용할 예정
    // 특정 오브젝트가 있을때 해당 오브젝트 경로에서 round의 반경을 가지고 있는 distance 알람 을 누락하는 기능.
    public bool ExceptionDistance(DistanceData data)
    {
        for (int i = 0; i < exceptionPosList.Count; i++)
        {
            (Vector3, Vector3, float) exceptionPos = exceptionPosList[i];

            float round = exceptionPos.Item3;

            Vector3 startPoint = data.p1;
            bool startCheck = false;
            Vector3 AB = exceptionPos.Item2 - exceptionPos.Item1;
            float abLen = AB.magnitude;
            Vector3 AP = startPoint - exceptionPos.Item1;
            float area2 = Vector3.Cross(AB, AP).magnitude;
            float startDis = area2 / abLen;
            if (startDis < round)
                startCheck = true;

            Vector3 endPoint = data.p2;
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
}
//~pjh