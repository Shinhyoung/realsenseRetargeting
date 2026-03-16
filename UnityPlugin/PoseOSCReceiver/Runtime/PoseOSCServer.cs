// ──────────────────────────────────────────────
// PoseOSCServer.cs — UDP 수신 + OSC 파싱 + 포즈 관리
// MonoBehaviour. 씬에 빈 오브젝트로 추가하면 동작.
// ──────────────────────────────────────────────
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

namespace PoseOSCReceiver
{
    public class PoseOSCServer : MonoBehaviour
    {
        [Header("OSC 설정")]
        [Tooltip("수신 포트 (Python 송신 포트와 동일하게)")]
        public int port = 9000;

        [Header("상태")]
        [Tooltip("현재 수신 중인 사람 수")]
        [SerializeField] int activePoseCount;

        [Tooltip("초당 수신 패킷 수")]
        [SerializeField] float packetsPerSecond;

        /// <summary>
        /// trackID → 최신 PoseSnapshot. 메인 스레드에서 읽기 전용으로 사용.
        /// </summary>
        public IReadOnlyDictionary<int, PoseSnapshot> Poses => _poses;

        // ── 내부 ──
        readonly Dictionary<int, PoseSnapshot> _poses = new();
        readonly object _lock = new();

        UdpClient _udp;
        Thread _recvThread;
        volatile bool _running;

        // 패킷 카운터
        int _packetCount;
        float _packetTimer;

        // 스레드 → 메인 스레드 전달용 큐
        readonly Queue<List<OscMessage>> _messageQueue = new();

        // 오래된 포즈 제거 (초)
        const float STALE_TIMEOUT = 2f;

        void OnEnable()
        {
            StartServer();
        }

        void OnDisable()
        {
            StopServer();
        }

        void Update()
        {
            // 수신 스레드에서 전달된 메시지를 메인 스레드에서 처리
            lock (_lock)
            {
                while (_messageQueue.Count > 0)
                {
                    var batch = _messageQueue.Dequeue();
                    ProcessMessages(batch);
                    _packetCount++;
                }
            }

            // Stale 포즈 제거
            CleanupStalePoses();

            // FPS 카운터
            _packetTimer += Time.deltaTime;
            if (_packetTimer >= 1f)
            {
                packetsPerSecond = _packetCount / _packetTimer;
                _packetCount = 0;
                _packetTimer = 0f;
            }

            activePoseCount = _poses.Count;
        }

        // ── 서버 시작/종료 ──

        void StartServer()
        {
            if (_running) return;

            try
            {
                _udp = new UdpClient(port);
                _udp.Client.ReceiveTimeout = 500;
                _running = true;

                _recvThread = new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = "OSC_Recv"
                };
                _recvThread.Start();

                Debug.Log($"[PoseOSC] 서버 시작 — 포트 {port}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[PoseOSC] 서버 시작 실패: {e.Message}");
            }
        }

        void StopServer()
        {
            _running = false;

            _udp?.Close();
            _udp = null;

            if (_recvThread != null && _recvThread.IsAlive)
                _recvThread.Join(1000);
            _recvThread = null;

            Debug.Log("[PoseOSC] 서버 종료");
        }

        // ── 수신 스레드 ──

        void ReceiveLoop()
        {
            IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);

            while (_running)
            {
                try
                {
                    byte[] data = _udp.Receive(ref remote);
                    if (data == null || data.Length == 0) continue;

                    var messages = OscParser.Parse(data, data.Length);
                    if (messages.Count == 0) continue;

                    lock (_lock)
                    {
                        _messageQueue.Enqueue(messages);
                    }
                }
                catch (SocketException)
                {
                    // timeout — 정상
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (Exception e)
                {
                    if (_running)
                        Debug.LogWarning($"[PoseOSC] 수신 오류: {e.Message}");
                }
            }
        }

        // ── 메시지 처리 ──

        void ProcessMessages(List<OscMessage> messages)
        {
            foreach (var msg in messages)
            {
                // 주소 형식: /pose/{trackID}/{jointName}
                if (!msg.Address.StartsWith("/pose/")) continue;
                if (msg.Floats == null || msg.Floats.Length < 3) continue;

                string[] parts = msg.Address.Split('/');
                // parts: ["", "pose", "0", "L_Shoulder"]
                if (parts.Length < 4) continue;

                if (!int.TryParse(parts[2], out int trackID)) continue;
                string joint = parts[3];

                // RealSense → Unity 좌표 변환
                // RS: X=오른쪽, Y=아래, Z=전방 (m)
                // Unity: X=오른쪽, Y=위, Z=전방 (m)
                float rsX = msg.Floats[0];
                float rsY = msg.Floats[1];
                float rsZ = msg.Floats[2];
                Vector3 unityPos = new Vector3(rsX, -rsY, rsZ);

                if (!_poses.TryGetValue(trackID, out var snapshot))
                {
                    snapshot = new PoseSnapshot { TrackID = trackID };
                    _poses[trackID] = snapshot;
                }

                snapshot.Joints[joint] = unityPos;
                snapshot.Timestamp = Time.time;
            }
        }

        void CleanupStalePoses()
        {
            List<int> stale = null;
            foreach (var kv in _poses)
            {
                if (Time.time - kv.Value.Timestamp > STALE_TIMEOUT)
                {
                    stale ??= new List<int>();
                    stale.Add(kv.Key);
                }
            }

            if (stale != null)
            {
                foreach (int id in stale)
                    _poses.Remove(id);
            }
        }

        // ── 공용 API ──

        /// <summary>
        /// 특정 trackID의 포즈 데이터 가져오기
        /// </summary>
        public PoseSnapshot GetPose(int trackID)
        {
            return _poses.TryGetValue(trackID, out var s) ? s : null;
        }

        /// <summary>
        /// 첫 번째 사람의 포즈 (단일 인물용)
        /// </summary>
        public PoseSnapshot GetFirstPose()
        {
            foreach (var kv in _poses)
                return kv.Value;
            return null;
        }
    }
}
