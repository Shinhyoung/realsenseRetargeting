// ──────────────────────────────────────────────
// PoseData.cs — 포즈 관절 데이터 구조체
// ──────────────────────────────────────────────
using System.Collections.Generic;
using UnityEngine;

namespace PoseOSCReceiver
{
    /// <summary>
    /// YOLO26 Pose 17개 관절 이름 (COCO 키포인트)
    /// </summary>
    public static class JointNames
    {
        public const string Nose        = "Nose";
        public const string L_Eye       = "L_Eye";
        public const string R_Eye       = "R_Eye";
        public const string L_Ear       = "L_Ear";
        public const string R_Ear       = "R_Ear";
        public const string L_Shoulder  = "L_Shoulder";
        public const string R_Shoulder  = "R_Shoulder";
        public const string L_Elbow     = "L_Elbow";
        public const string R_Elbow     = "R_Elbow";
        public const string L_Wrist     = "L_Wrist";
        public const string R_Wrist     = "R_Wrist";
        public const string L_Hip       = "L_Hip";
        public const string R_Hip       = "R_Hip";
        public const string L_Knee      = "L_Knee";
        public const string R_Knee      = "R_Knee";
        public const string L_Ankle     = "L_Ankle";
        public const string R_Ankle     = "R_Ankle";

        public static readonly string[] All = new[]
        {
            Nose, L_Eye, R_Eye, L_Ear, R_Ear,
            L_Shoulder, R_Shoulder, L_Elbow, R_Elbow,
            L_Wrist, R_Wrist, L_Hip, R_Hip,
            L_Knee, R_Knee, L_Ankle, R_Ankle
        };
    }

    /// <summary>
    /// 한 사람의 포즈 데이터 (trackID 기준)
    /// </summary>
    public class PoseSnapshot
    {
        public int TrackID;
        public float Timestamp;
        public Dictionary<string, Vector3> Joints = new();

        /// <summary>
        /// 관절 위치 반환. 없으면 Vector3.zero
        /// </summary>
        public Vector3 GetJoint(string name)
        {
            return Joints.TryGetValue(name, out var v) ? v : Vector3.zero;
        }

        /// <summary>
        /// 두 관절 사이의 방향 벡터 (normalized)
        /// </summary>
        public Vector3 GetDirection(string from, string to)
        {
            Vector3 a = GetJoint(from);
            Vector3 b = GetJoint(to);
            Vector3 dir = b - a;
            return dir.sqrMagnitude > 0.0001f ? dir.normalized : Vector3.zero;
        }

        /// <summary>
        /// 해당 관절에 유효한 데이터가 있는지
        /// </summary>
        public bool HasJoint(string name)
        {
            return Joints.ContainsKey(name);
        }
    }
}
