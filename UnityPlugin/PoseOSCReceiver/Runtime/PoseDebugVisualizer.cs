// ──────────────────────────────────────────────
// PoseDebugVisualizer.cs — 수신된 관절을 Gizmo/구체로 시각화
// 디버깅 및 테스트용
// ──────────────────────────────────────────────
using UnityEngine;

namespace PoseOSCReceiver
{
    public class PoseDebugVisualizer : MonoBehaviour
    {
        [Header("참조")]
        public PoseOSCServer oscServer;

        [Header("표시 설정")]
        [Tooltip("대상 trackID (-1 = 모든 사람)")]
        public int trackID = -1;

        [Tooltip("관절 구체 크기")]
        public float jointRadius = 0.03f;

        [Tooltip("관절 색상")]
        public Color jointColor = Color.green;

        [Tooltip("본 연결선 색상")]
        public Color boneColor = Color.cyan;

        [Tooltip("위치 오프셋 (시각화 위치 조정)")]
        public Vector3 offset = Vector3.zero;

        // 본 연결 정의 (부모 → 자식)
        static readonly (string, string)[] BoneConnections = new[]
        {
            // 얼굴
            (JointNames.Nose, JointNames.L_Eye),
            (JointNames.Nose, JointNames.R_Eye),
            (JointNames.L_Eye, JointNames.L_Ear),
            (JointNames.R_Eye, JointNames.R_Ear),
            // 상체
            (JointNames.L_Shoulder, JointNames.R_Shoulder),
            (JointNames.L_Shoulder, JointNames.L_Elbow),
            (JointNames.R_Shoulder, JointNames.R_Elbow),
            (JointNames.L_Elbow, JointNames.L_Wrist),
            (JointNames.R_Elbow, JointNames.R_Wrist),
            // 몸통
            (JointNames.L_Shoulder, JointNames.L_Hip),
            (JointNames.R_Shoulder, JointNames.R_Hip),
            (JointNames.L_Hip, JointNames.R_Hip),
            // 하체
            (JointNames.L_Hip, JointNames.L_Knee),
            (JointNames.R_Hip, JointNames.R_Knee),
            (JointNames.L_Knee, JointNames.L_Ankle),
            (JointNames.R_Knee, JointNames.R_Ankle),
        };

        void OnDrawGizmos()
        {
            if (oscServer == null || oscServer.Poses == null) return;

            foreach (var kv in oscServer.Poses)
            {
                if (trackID >= 0 && kv.Key != trackID) continue;

                var pose = kv.Value;

                // 관절 구체
                Gizmos.color = jointColor;
                foreach (string name in JointNames.All)
                {
                    if (!pose.HasJoint(name)) continue;
                    Vector3 pos = pose.GetJoint(name) + offset;
                    Gizmos.DrawSphere(pos, jointRadius);
                }

                // 본 연결선
                Gizmos.color = boneColor;
                foreach (var (from, to) in BoneConnections)
                {
                    if (!pose.HasJoint(from) || !pose.HasJoint(to)) continue;
                    Vector3 a = pose.GetJoint(from) + offset;
                    Vector3 b = pose.GetJoint(to) + offset;
                    Gizmos.DrawLine(a, b);
                }
            }
        }

        void OnGUI()
        {
            if (oscServer == null) return;

            int y = 10;
            GUI.color = Color.white;
            GUI.Label(new Rect(10, y, 300, 20), $"[PoseOSC] Active: {oscServer.Poses.Count}명");
            y += 20;

            foreach (var kv in oscServer.Poses)
            {
                if (trackID >= 0 && kv.Key != trackID) continue;

                GUI.Label(new Rect(10, y, 400, 20), $"  Track #{kv.Key} — {kv.Value.Joints.Count} joints");
                y += 18;

                foreach (var joint in kv.Value.Joints)
                {
                    var v = joint.Value;
                    GUI.Label(new Rect(30, y, 500, 20),
                        $"{joint.Key}: ({v.x:F3}, {v.y:F3}, {v.z:F3})");
                    y += 16;
                }
            }
        }
    }
}
