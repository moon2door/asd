using UnityEngine;

[RequireComponent(typeof(ParticleSystem))]
public class PointCloudRenderer : MonoBehaviour
{
    private ParticleSystem ps;
    private ParticleSystem.Particle[] particles;

    [Header("Settings")]
    public float pointSize = 0.1f;
    public Color pointColor = Color.cyan;

    private void Awake()
    {
        ps = GetComponent<ParticleSystem>();

        // 파티클 시스템 초기 설정 (코드에서 강제로 맞춤)
        var main = ps.main;
        main.startLifetime = float.MaxValue; // 죽지 않음
        main.startSpeed = 0;                 // 움직이지 않음
        main.simulationSpace = ParticleSystemSimulationSpace.World; // 월드 좌표 사용
        main.maxParticles = 100000;          // 최대 점 개수 (넉넉하게)

        var emission = ps.emission;
        emission.enabled = false;            // 자동 발사 끔 (우리가 수동으로 찍을 것임)

        var shape = ps.shape;
        shape.enabled = false;               // 모양 필요 없음
    }

    // 외부에서 좌표 배열을 던져주면 화면에 그림
    public void DrawPoints(Vector3[] positions)
    {
        if (ps == null) return;

        // 배열 크기 확보 (메모리 재할당 최소화)
        if (particles == null || particles.Length < positions.Length)
        {
            particles = new ParticleSystem.Particle[positions.Length];
        }

        // 파티클 데이터 채우기
        for (int i = 0; i < positions.Length; i++)
        {
            particles[i].position = positions[i];
            particles[i].startColor = pointColor;
            particles[i].startSize = pointSize;

            // 수명을 무한대로 설정해서 사라지지 않게 합니다.
            particles[i].startLifetime = float.MaxValue;
            particles[i].remainingLifetime = float.MaxValue;
        }

        // 화면에 반영 (SetParticles가 가장 빠름)
        ps.SetParticles(particles, positions.Length);
    }

    // 점 지우기
    public void Clear()
    {
        if (ps != null) ps.Clear();
    }
}