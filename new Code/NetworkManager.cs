using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;
using PortSystem.Data;

public class NetworkManager : MonoBehaviour
{
    // 어디서든 접근 가능한 싱글톤 패턴
    public static NetworkManager Instance;

    [Header("Network Settings")]
    public string serverIp = "127.0.0.1";
    public int serverPort = 9098;

    private TcpClient client;
    private NetworkStream stream;
    private bool isRunning = false;

    // 메인 스레드에서 처리할 행동들을 담아두는 큐 (스레드 안전)
    private readonly Queue<Action> mainThreadActions = new Queue<Action>();

    // 데이터 수신 이벤트 (구독자 패턴)
    // 데이터가 도착하면 이 이벤트들이 발동
    public event Action<StCraneAttitude> OnCraneDataReceived;
    public event Action<StSystemStatus> OnSystemStatusReceived;
    public event Action<StPointCloud> OnPointCloudReceived;
    public event Action<StDistanceResult> OnDistanceResultReceived;

    private void Awake()
    {
        // 싱글톤 설정
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
    }

    private void Start()
    {
        ConnectToServer();
    }

    private void Update()
    {
        // 백그라운드 스레드에서 받은 데이터를 메인 스레드(Unity)에서 처리
        // (Unity의 UI나 Transform은 메인 스레드에서만 건드릴 수 있기 때문)
        lock (mainThreadActions)
        {
            while (mainThreadActions.Count > 0)
            {
                mainThreadActions.Dequeue().Invoke();
            }
        }
    }

    public async void ConnectToServer()
    {
        if (isRunning) return;

        try
        {
            Debug.Log($"[Network] {serverIp}:{serverPort} 연결 시도 중...");
            client = new TcpClient();

            // 비동기 연결 (화면 멈춤 없음)
            await client.ConnectAsync(serverIp, serverPort);

            stream = client.GetStream();
            isRunning = true;
            Debug.Log("[Network] 연결 성공!");

            // 데이터 수신 루프 시작
            _ = ReceiveLoop();
        }
        catch (Exception e)
        {
            Debug.LogError($"[Network] 연결 실패: {e.Message}");
        }
    }

    // 무한히 데이터를 기다리고 받는 루프
    private async Task ReceiveLoop()
    {
        // 헤더 버퍼 (6바이트) 미리 할당
        byte[] headerBuffer = new byte[6];

        while (isRunning && client.Connected)
        {
            try
            {
                // 1. 헤더 읽기 (6바이트가 찰 때까지 기다림)
                int bytesRead = await ReadExactAsync(headerBuffer, 6);
                if (bytesRead == 0) break; // 연결 끊김

                // 헤더 파싱 (PortSystem.Data의 SocketHeader 사용)
                StSocketHeader header = new StSocketHeader(headerBuffer);

                // 2. 바디(Payload) 읽기
                if (header.PayloadLength > 0)
                {
                    byte[] bodyBuffer = new byte[header.PayloadLength];
                    int bodyRead = await ReadExactAsync(bodyBuffer, header.PayloadLength);

                    if (bodyRead != header.PayloadLength) break; // 데이터 깨짐

                    // 3. 데이터 처리 (메인 스레드 큐로 보냄)
                    EnqueueAction(() => ProcessPacket(header, bodyBuffer));
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[Network] 수신 에러: {e.Message}");
                break;
            }
        }
        Disconnect();
    }

    // TCP 패킷이 잘려서 들어올 경우를 대비해, 원하는 바이트만큼 꽉 채워서 읽는 함수
    private async Task<int> ReadExactAsync(byte[] buffer, int length)
    {
        int totalRead = 0;
        while (totalRead < length)
        {
            int read = await stream.ReadAsync(buffer, totalRead, length - totalRead);
            if (read == 0) return 0;
            totalRead += read;
        }
        return totalRead;
    }

    private void ProcessPacket(StSocketHeader header, byte[] body)
    {
        // 1. 시스템 상태
        if (header.Type == 0x01 && header.Code == 0x0c)
        {
            StSystemStatus status = new StSystemStatus(body);
            OnSystemStatusReceived?.Invoke(status);
            return;
        }

        // 2. 크레인 데이터
        else if (header.Type == 4 && header.Code == 40)
        {
            try
            {
                StCraneAttitude temp = new StCraneAttitude();
                int parsedLength = temp.FromByte(body);

                // 파싱은 성공했더라도, 내용물이 정상인지 체크
                if (parsedLength > 0)
                {
                    // 부두 ID가 0~10 사이이고, 크레인 ID도 정상 범위인 경우만 진짜로 인정
                    bool isValidId = (temp.pierId >= 0 && temp.pierId <= 10) &&
                                                (temp.craneId >= 0 && temp.craneId < 50);

                    if (isValidId)
                    {
                        // 콘솔 도배를 막기 위해 / 진짜 패킷을 찾았을 때 / 만 로그를 한 번 찍고 싶다면 아래 주석을 풀면됨
                        // Debug.Log($"[Real Packet Found] Type: {header.Type}, Code: {header.Code} -> Pier: {temp.pierId}, Crane: {temp.craneId}");

                        OnCraneDataReceived?.Invoke(temp);
                    }
                    else
                    {
                        // 가짜 패킷 (쓰레기 값) → 확인하고 싶을떄만 주석 풀면 될듯
                        // Debug.LogWarning($"[Filtered] 가짜 패킷 걸러짐: Type {header.Type}, Code {header.Code} (PierID가 {temp.pierId}로 나옴)");
                    }
                }
            }
            catch (Exception)
            {
                // 파싱 에러는 무시
                // Debug.Log("파킹에러가 뜨긴했는데 그냥 무시해도됨");
            }
        }

        // 3. 포인트 데이터 / 충돌감지 (일단 JIB5 기준)
        else if (header.Type == 1 && header.Code == 4)
        {
            try
            {
                // 1. 메가 패킷 파싱 (점 + 위험정보)
                StFullPointCloud fullData = new StFullPointCloud(body);

                // 2. 포인트 클라우드 전송
                if (fullData.points.Count > 0)
                {
                    StPointCloud pointWrapper = new StPointCloud();
                    pointWrapper.points = fullData.points;
                    OnPointCloudReceived?.Invoke(pointWrapper);
                }

                Debug.Log($"[Network Check] 파싱된 리스크 개수: {fullData.risks?.Count ?? -1}");

                // 3. ★ 충돌 정보 전송 (새로 추가)
                if (fullData.risks.Count > 0)
                {
                    StDistanceResult riskWrapper = new StDistanceResult(); // 빈 생성자 필요
                    riskWrapper.risks = fullData.risks;
                    OnDistanceResultReceived?.Invoke(riskWrapper);
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[비상] Code 4 처리 실패: {e.Message}");
            }
        }
    }

    private void EnqueueAction(Action action)
    {
        lock (mainThreadActions)
        {
            mainThreadActions.Enqueue(action);
        }
    }

    private void Disconnect()
    {
        isRunning = false;
        client?.Close();
        Debug.Log("[Network] 연결 종료");
    }

    private void OnDestroy()
    {
        Disconnect();
    }
}