using UnityEngine;
using PortSystem.Data;

public class CraneController : MonoBehaviour
{
    public int pierId;
    public int craneId;

    [Header("Joints (Drag your models here)")]
    public Transform bodyTransform; // 회전하는 몸통
    public Transform jibTransform;  // 위아래로 움직이는 붐대

    public void Initialize(int pId, int cId)
    {
        this.pierId = pId;
        this.craneId = cId;
    }

    public void UpdateMovement(StCraneAttitude data)
    {
        // StCraneAttitude의 pose 배열에 각도가 들어옴
        // 보통 pose[0]: 선회(Swing), pose[1]: 기복(Luffing) 일 확률이 높음
        // 데이터 값을 찍어보며 맞춰 봐야함

        if (bodyTransform != null)
        {
            // Y축 회전 (Swing)
            float swingAngle = data.pose[0];
            bodyTransform.localRotation = Quaternion.Euler(0, swingAngle, 0);
        }

        if (jibTransform != null)
        {
            // X축 회전 (Luffing)
            float luffingAngle = data.pose[1];
            jibTransform.localRotation = Quaternion.Euler(luffingAngle, 0, 0);
        }
    }
}