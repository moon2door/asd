using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// GPS만 가지고 크레인에 붙은 GPS 센서의 로컬(XZ) 위치 오프셋을 추정한다.
///
/// 전제:
/// - 각 프레임마다 다음 정보를 받는다:
///     (1) GPS의 월드 좌표 (Unity 좌표계)
///     (2) 크레인 본체의 heading / yaw (도 단위 or 라디안 가능, 여기선 도 단위 가정)
///
/// 우리가 "모르는 것":
/// - 크레인 회전 중심(선회축)의 실제 월드 좌표
///
/// 우리가 "구하고 싶은 것":
/// - GPS 센서가 크레인 몸체 기준으로 어디에 달려 있는지 (XZ 평면 오프셋, 고정 벡터)
///
/// 수학:
/// GPS(t) = C + R(t) * L
///   C: (알 수 없음) 회전 중심의 월드 XZ 위치
///   L: (알고 싶은 값) GPS 로컬 XZ 오프셋
///   R(t): yaw로 만든 2D 회전
///
/// 두 시점 t0, t1 에 대해
/// GPS(t1) - GPS(t0) = ( R(t1) - R(t0) ) * L
///
/// 이 식을 이용하면 C는 없어지고, L만 남는다.
/// 여러 시점 쌍에서 L을 구해보고 평균하면 안정화된 L을 얻을 수 있다.
///
/// 주의:
/// - GPS가 안 움직인 구간(또는 yaw 거의 안 변한 구간)은 정보가 거의 없으므로 샘플에서 제외한다.
/// - 높이(Y)는 무시한다. XZ 평면만 사용한다.
/// - yaw=0일 때의 크레인 전방을 +Z, 오른쪽을 +X이라고 두고 L은 그 좌표계 기준으로 리턴된다.
///
/// 사용 흐름:
/// 1) 운용 중 매 프레임 AddSample(gpsPosWorld, bodyYawDeg)를 호출해서 데이터 쌓기
///    - gpsPosWorld: 현재 GPS 월드 좌표 (Vector3)
///    - bodyYawDeg: 현재 크레인 몸체 yaw (도 단위)
///
/// 2) 어느 정도 쌓이면 SolveOffset() 호출
///    - estimatedLocalOffsetXZ에 결과 저장
///    - Debug.Log로 결과 출력
///
/// 3) estimatedLocalOffsetXZ는 "크레인 로컬 좌표(몸체 기준)에서 GPS까지의 오프셋(XZ)"
///    → 추후: 몸체 중심 월드 위치를 추정할 때나, GPS 예측위치 계산할 때 사용할 수 있음.
///
/// 제한:
/// - SolveOffset()은 "최근 일정 구간 동안 크레인 베이스(중심 C)가 크게 안 움직였다"는 가정 하에 더 잘 맞는다.
///   만약 크레인이 실제로 트럭째로 이동 중이라면, 그 구간마다 따로 돌리는 게 좋다.
/// </summary>
public class CraneGpsCalibrator_RadiusAngle : MonoBehaviour
{
    /// <summary>
    /// 단일 샘플: GPS 위치(XZ만 유효) + 본체 yaw(도)
    /// </summary>
    public struct Sample
    {
        public Vector2 gpsXZ;  // GPS의 월드 XZ 좌표
        public float yawDeg;   // 크레인 몸체의 heading/yaw (도 단위)
    }

    // ----------------------------------------------------
    // 저장소
    // ----------------------------------------------------

    [Tooltip("수집된 샘플들 (최근 순서대로 쌓임)")]
    public List<Sample> samples = new List<Sample>();

    [Tooltip("최종 추정된 GPS 로컬 오프셋 (XZ). yaw=0일 때 기준으로, x=우측(+X), z=전방(+Z).")]
    public Vector2 estimatedLocalOffsetXZ;

    // ----------------------------------------------------
    // 파라미터
    // ----------------------------------------------------

    [Tooltip("GPS 위치가 이전 샘플과 이 거리(m) 이하로만 바뀌면 '안 움직인 것'으로 보고 샘플을 추가하지 않는다.")]
    public float minPositionChangeEps = 0.01f;

    [Tooltip("두 샘플 사이 yaw 차이가 이 각도(deg) 이하이면 회전 변화가 너무 적다고 보고 그 쌍은 해석에 쓰지 않는다.")]
    public float minYawDeltaDeg = 0.5f;

    [Tooltip("SolveOffset() 시에 사용할 최대 과거 샘플 개수 (너무 많으면 오래된 움직임까지 섞여 오염될 수 있음)")]
    public int maxSolveSamples = 50;

    [Tooltip("L 후보들 중 평균에서 k*표준편차보다 많이 벗어나는 값은 버린다.")]
    public float outlierK = 2.0f;

    // 내부 캐시: 마지막으로 들어온 GPS 위치, 중복 판단용
    private bool hasLast = false;
    private Vector2 lastGpsXZ;

    // ----------------------------------------------------
    // 외부에서 매 프레임 호출하는 함수
    // ----------------------------------------------------

    /// <summary>
    /// 샘플 하나 추가.
    /// gpsWorldPos : 현재 GPS 월드 좌표 (Vector3, y는 무시됨)
    /// bodyYawDeg  : 현재 크레인 본체 yaw (도 단위). 0도 = 로컬 전방(+Z), +90도 = 로컬 오른쪽(+X) 쪽이라고 가정.
    ///
    /// 중요한 동작:
    /// - GPS가 거의 안 움직였으면(거리 변화 거의 없으면) 이 샘플은 버린다.
    ///   -> 이유: 좌표가 안 바뀌는 상태는 정보가 없거나, 센서가 멈춘 상태일 수 있음
    /// </summary>
    public void AddSample(Vector3 gpsWorldPos, float bodyYawDeg)
    {
        Vector2 gpsXZ = new Vector2(gpsWorldPos.x, gpsWorldPos.z);

        if (hasLast)
        {
            float dist = (gpsXZ - lastGpsXZ).magnitude;
            if (dist < minPositionChangeEps)
            {
                // 움직임이 너무 작으면 의미 없는 반복 샘플로 간주하고 스킵
                // (GPS 좌표가 그대로라는 건 새 정보가 없다는 뜻)
                return;
            }
        }

        Sample s;
        s.gpsXZ = gpsXZ;
        s.yawDeg = bodyYawDeg;
        samples.Add(s);

        lastGpsXZ = gpsXZ;
        hasLast = true;
    }

    // ----------------------------------------------------
    // 보조 유틸: yaw(deg) -> 2D 회전 행렬 R
    //
    // yaw=0일 때, 우리는 로컬 +Z가 "앞", +X가 "오른쪽"이라고 놓는다.
    // 그럼 yaw가 θ일 때,
    //   forward(+Z) -> world 방향 [ sinθ, cosθ ]
    //   right(+X)   -> world 방향 [ cosθ, -sinθ ] 같은 케이스 등등
    //
    // 여기서는 "로컬 (x,z)"를 [lx, lz]라고 했을 때
    // worldXZ = R * [lx, lz]
    //
    // R = [ cosθ   -sinθ
    //       sinθ    cosθ ]
    //
    // 이건 Unity에서 'yaw 회전'의 표준 2D 회전.
    //
    // 즉 L_local = (lx, lz),
    // world     = ( lx*cosθ - lz*sinθ,
    //               lx*sinθ + lz*cosθ )
    //
    // 위 행렬 R(θ)를 반환한다.
    // ---------------------------------------

    // 간단한 2x2 행렬 struct
    struct Matrix2x2
    {
        public float m00, m01;
        public float m10, m11;

        // subtract (this - other)
        public static Matrix2x2 Sub(Matrix2x2 a, Matrix2x2 b)
        {
            Matrix2x2 r;
            r.m00 = a.m00 - b.m00;
            r.m01 = a.m01 - b.m01;
            r.m10 = a.m10 - b.m10;
            r.m11 = a.m11 - b.m11;
            return r;
        }

        // inverse of 2x2
        public bool Inverse(out Matrix2x2 inv)
        {
            float det = m00 * m11 - m01 * m10;
            if (Mathf.Abs(det) < 1e-6f)
            {
                inv = default(Matrix2x2);
                return false;
            }

            float invDet = 1.0f / det;
            inv.m00 = m11 * invDet;
            inv.m01 = -m01 * invDet;
            inv.m10 = -m10 * invDet;
            inv.m11 = m00 * invDet;
            return true;
        }

        // multiply 2x2 * Vector2
        public Vector2 MulVec(Vector2 v)
        {
            return new Vector2(
                m00 * v.x + m01 * v.y,
                m10 * v.x + m11 * v.y
            );
        }
    }

    // ----------------------------------------------------
    // SolveOffset : 현재까지 모은 samples로 L(local offset)을 추정한다.
    //
    // 아이디어:
    //   GPS_i = C + R_i * L
    //   GPS_0 = C + R_0 * L
    //
    //   (GPS_i - GPS_0) = (R_i - R_0) * L
    //
    // -> L = (R_i - R_0)^(-1) * (GPS_i - GPS_0)
    //
    // 이걸 여러 i에 대해 구하면 L 후보가 여러 개 생긴다.
    // 그 후보들 중 이상치 제거 후 평균낸 것이 최종 L.
    //
    // 이 함수는 estimatedLocalOffsetXZ 를 갱신한다.
    // ----------------------------------------------------
    struct Row { public Matrix2x2 dR; public Vector2 dG; public float wBase; }
    [ContextMenu("SolveOffset_AllPairs")]
    public void SolveOffset_AllPairs()
    {
        int n = samples.Count;
        if (n < 2)
        {
            Debug.LogWarning("[OT] SolveOffset: not enough samples");
            return;
        }
        // 너무 오래된 건 제외: 최근 maxSolveSamples개만 사용
        int startIndex = Mathf.Max(0, n - maxSolveSamples);

        List<Vector2> craneCenterList;
        Vector2 center = Vector2.zero;
        List<Sample> slice = samples.GetRange(startIndex, n- startIndex);
        if (LeastSquaresIntersection(slice, out center) == false)
        {
            Debug.LogWarning("[CRANE] SolveOffset: not valid samples");
            return;
        }

        Vector2 gpsOffset = Compute(slice, center);
        estimatedLocalOffsetXZ = gpsOffset;
    }

    // 여러 직선(무한직선)의 최적 교점(최소제곱)
    bool LeastSquaresIntersection(List<Sample> items, out Vector2 x, float eps = 1e-6f)
    {
        x = default;
        if (items == null || items.Count < 2) return false;

        float a = 0f, b = 0f, c = 0f; // M = [a b; b c]
        float bx = 0f, by = 0f;     // b벡터 합

        float r = Mathf.Deg2Rad;
        for (int i = 0; i < items.Count; i++)
        {
            var pi = items[i].gpsXZ;
            float th = items[i].yawDeg * r;
            // 법선 n = (-sinθ, cosθ)
            float nx = -Mathf.Sin(th);
            float ny = Mathf.Cos(th);

            a += nx * nx;
            b += nx * ny;
            c += ny * ny;

            float c_i = nx * pi.x + ny * pi.y; // n·p
            bx += nx * c_i;
            by += ny * c_i;
        }

        // 2x2 역행렬
        float det = a * c - b * b;
        if (Mathf.Abs(det) < eps) return false; // 거의 평행들로 구성된 경우

        float inv00 = c / det;
        float inv01 = -b / det;
        float inv10 = -b / det;
        float inv11 = a / det;

        x = new Vector2(inv00 * bx + inv01 * by, inv10 * bx + inv11 * by);
        return true;
    }
    struct Result
    {
        public Vector2 meanResidual; // 평균 (벡터) = 평균(pi - center)
        public float meanDistance;   // 평균 |ri|
        public float rmsDistance;    // sqrt( mean(|ri|^2) )
        public float stdDistance;    // 거리의 표준편차
        public int count;            // 사용된 표본 수
    }
    Vector2 Compute(List<Sample> pts, Vector2 center)
    {
        Result r = new Result { count = 0 };
        if (pts == null || pts.Count == 0) return Vector2.zero;

        Vector2 sumResidual = Vector2.zero;
        float sumAbs = 0f;       // Σ |ri|
        float sumSq = 0f;        // Σ |ri|^2

        for (int i = 0; i < pts.Count; i++)
        {
            Vector2 offset= pts[i].gpsXZ - center;   // ri
            Vector2 res = Vector2.zero;
            float angleZ = pts[i].yawDeg * Mathf.Deg2Rad;
            float c = Mathf.Cos(angleZ);
            float s = Mathf.Sin(angleZ);
            res.x = offset.x * c + offset.y * s;
            res.y = -offset.x * s + offset.y * c;
            sumResidual += res;

            float d2 = res.sqrMagnitude;     // |ri|^2
            sumSq += d2;
            sumAbs += Mathf.Sqrt(d2);
        }

        int n = pts.Count;
        r.count = n;
        r.meanResidual = sumResidual / n;            // 평균 잔차 벡터
        r.meanDistance = sumAbs / n;                 // 평균 거리
        r.rmsDistance = Mathf.Sqrt(sumSq / n);      // RMS

        // 표준편차(거리): dd = |ri|,  std = sqrt( E[dd^2] - (E[dd])^2 )
        float E_dd2 = sumSq / n;
        float E_dd = r.meanDistance;
        float var = Mathf.Max(0f, E_dd2 - E_dd * E_dd);
        r.stdDistance = Mathf.Sqrt(var);

        return r.meanResidual;
    }
    /// <summary>
    /// Vector2 후보들에서 평균을 낸다.
    /// 절차:
    /// 1) 1차 평균 µ 계산
    /// 2) 각 벡터와 µ 사이의 거리(유클리드)로 표준편차 비슷한 scale 추정
    /// 3) 평균에서 k * std 이상 벗어나면 제외
    /// 4) 남은 것만 다시 평균
    /// </summary>
    Vector2 AverageNoOutlierVector2(List<Vector2> vals, float k)
    {
        if (vals.Count == 0) return Vector2.zero;

        // 1차 평균
        Vector2 mean = Vector2.zero;
        for (int i = 0; i < vals.Count; i++)
        {
            mean += vals[i];
        }
        mean /= vals.Count;

        // 분산 비슷한 척도: 거리^2 평균
        float sumSq = 0f;
        for (int i = 0; i < vals.Count; i++)
        {
            Vector2 diff = vals[i] - mean;
            sumSq += diff.sqrMagnitude;
        }

        float variance = sumSq / vals.Count;
        float stdDev = Mathf.Sqrt(variance); // '평균적인 거리' 느낌

        if (stdDev < 1e-6f)
        {
            return mean; // 거의 다 같은 위치에 모여있으면 그대로
        }

        float maxDist = stdDev * k;

        Vector2 refined = Vector2.zero;
        int refinedCount = 0;
        for (int i = 0; i < vals.Count; i++)
        {
            Vector2 diff = vals[i] - mean;
            float dist = diff.magnitude;
            if (dist <= maxDist)
            {
                refined += vals[i];
                refinedCount++;
            }
        }

        if (refinedCount == 0)
        {
            return mean;
        }

        refined /= refinedCount;
        return refined;
    }
}
