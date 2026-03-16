// ──────────────────────────────────────────────
// PoseRetargeting.cs — 수신된 관절 위치로 아바타 본 회전 적용
// Humanoid Avatar에 부착하여 사용
// ──────────────────────────────────────────────
using UnityEngine;

namespace PoseOSCReceiver
{
    public class PoseRetargeting : MonoBehaviour
    {
        [Header("참조")]
        [Tooltip("PoseOSCServer 컴포넌트")]
        public PoseOSCServer oscServer;

        [Tooltip("대상 trackID (0 = 첫 번째 사람)")]
        public int trackID = 0;

        [Header("설정")]
        [Tooltip("회전 스무딩 (0=즉시, 1=매우 느림)")]
        [Range(0f, 0.95f)]
        public float smoothing = 0.5f;

        [Tooltip("위치 추적 활성화 (아바타 루트 이동)")]
        public bool enableRootMotion = true;

        [Tooltip("루트 위치 스무딩")]
        [Range(0f, 0.95f)]
        public float rootSmoothing = 0.3f;

        // Humanoid Animator
        Animator _animator;

        // 본 레퍼런스
        Transform _leftUpperArm, _rightUpperArm;
        Transform _leftLowerArm, _rightLowerArm;
        Transform _leftUpperLeg, _rightUpperLeg;
        Transform _leftLowerLeg, _rightLowerLeg;
        Transform _spine;
        Transform _head;
        Transform _hips;

        // Bind Pose 방향 (T-Pose 기준)
        Vector3 _bindLeftUpperArm, _bindRightUpperArm;
        Vector3 _bindLeftLowerArm, _bindRightLowerArm;
        Vector3 _bindLeftUpperLeg, _bindRightUpperLeg;
        Vector3 _bindLeftLowerLeg, _bindRightLowerLeg;
        Vector3 _bindSpineDir;

        // Bind Pose 월드 회전 (T-Pose 기준)
        Quaternion _bindRotLeftUpperArm, _bindRotRightUpperArm;
        Quaternion _bindRotLeftLowerArm, _bindRotRightLowerArm;
        Quaternion _bindRotLeftUpperLeg, _bindRotRightUpperLeg;
        Quaternion _bindRotLeftLowerLeg, _bindRotRightLowerLeg;
        Quaternion _bindRotSpine, _bindRotHead;

        // 스무딩용 현재 회전값
        Quaternion _curLeftUpperArm, _curRightUpperArm;
        Quaternion _curLeftLowerArm, _curRightLowerArm;
        Quaternion _curLeftUpperLeg, _curRightUpperLeg;
        Quaternion _curLeftLowerLeg, _curRightLowerLeg;
        Quaternion _curSpine;
        Quaternion _curHead;

        Vector3 _curRootPos;

        void Start()
        {
            _animator = GetComponent<Animator>();
            if (_animator == null || !_animator.isHuman)
            {
                Debug.LogError("[PoseRetargeting] Humanoid Animator가 필요합니다.");
                enabled = false;
                return;
            }

            // 본 가져오기
            _leftUpperArm  = _animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            _rightUpperArm = _animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
            _leftLowerArm  = _animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
            _rightLowerArm = _animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
            _leftUpperLeg  = _animator.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
            _rightUpperLeg = _animator.GetBoneTransform(HumanBodyBones.RightUpperLeg);
            _leftLowerLeg  = _animator.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
            _rightLowerLeg = _animator.GetBoneTransform(HumanBodyBones.RightLowerLeg);
            _spine = _animator.GetBoneTransform(HumanBodyBones.Spine);
            _head  = _animator.GetBoneTransform(HumanBodyBones.Head);
            _hips  = _animator.GetBoneTransform(HumanBodyBones.Hips);

            // T-Pose 기준 본 방향 저장
            _bindLeftUpperArm  = GetBindDirection(_leftUpperArm, _leftLowerArm);
            _bindRightUpperArm = GetBindDirection(_rightUpperArm, _rightLowerArm);
            _bindLeftLowerArm  = GetBindDirection(_leftLowerArm,
                _animator.GetBoneTransform(HumanBodyBones.LeftHand));
            _bindRightLowerArm = GetBindDirection(_rightLowerArm,
                _animator.GetBoneTransform(HumanBodyBones.RightHand));
            _bindLeftUpperLeg  = GetBindDirection(_leftUpperLeg, _leftLowerLeg);
            _bindRightUpperLeg = GetBindDirection(_rightUpperLeg, _rightLowerLeg);
            _bindLeftLowerLeg  = GetBindDirection(_leftLowerLeg,
                _animator.GetBoneTransform(HumanBodyBones.LeftFoot));
            _bindRightLowerLeg = GetBindDirection(_rightLowerLeg,
                _animator.GetBoneTransform(HumanBodyBones.RightFoot));

            // Spine bind 방향: hip 중심 → shoulder 중심
            if (_hips != null && _spine != null)
                _bindSpineDir = (_spine.position - _hips.position).normalized;
            if (_bindSpineDir.sqrMagnitude < 0.01f)
                _bindSpineDir = Vector3.up;

            // T-Pose 기준 월드 회전 저장
            _bindRotLeftUpperArm  = _leftUpperArm  != null ? _leftUpperArm.rotation  : Quaternion.identity;
            _bindRotRightUpperArm = _rightUpperArm != null ? _rightUpperArm.rotation : Quaternion.identity;
            _bindRotLeftLowerArm  = _leftLowerArm  != null ? _leftLowerArm.rotation  : Quaternion.identity;
            _bindRotRightLowerArm = _rightLowerArm != null ? _rightLowerArm.rotation : Quaternion.identity;
            _bindRotLeftUpperLeg  = _leftUpperLeg  != null ? _leftUpperLeg.rotation  : Quaternion.identity;
            _bindRotRightUpperLeg = _rightUpperLeg != null ? _rightUpperLeg.rotation : Quaternion.identity;
            _bindRotLeftLowerLeg  = _leftLowerLeg  != null ? _leftLowerLeg.rotation  : Quaternion.identity;
            _bindRotRightLowerLeg = _rightLowerLeg != null ? _rightLowerLeg.rotation : Quaternion.identity;
            _bindRotSpine = _spine != null ? _spine.rotation : Quaternion.identity;
            _bindRotHead  = _head  != null ? _head.rotation  : Quaternion.identity;

            // 초기 회전값을 bind pose 기준으로 설정 (포즈 수신 전 T-Pose 유지)
            _curLeftUpperArm  = _bindRotLeftUpperArm;
            _curRightUpperArm = _bindRotRightUpperArm;
            _curLeftLowerArm  = _bindRotLeftLowerArm;
            _curRightLowerArm = _bindRotRightLowerArm;
            _curLeftUpperLeg  = _bindRotLeftUpperLeg;
            _curRightUpperLeg = _bindRotRightUpperLeg;
            _curLeftLowerLeg  = _bindRotLeftLowerLeg;
            _curRightLowerLeg = _bindRotRightLowerLeg;
            _curSpine = _bindRotSpine;
            _curHead  = _bindRotHead;
            _curRootPos = transform.position;
        }

        void LateUpdate()
        {
            if (oscServer == null) return;

            PoseSnapshot pose = oscServer.GetPose(trackID)
                             ?? oscServer.GetFirstPose();
            if (pose == null) return;

            float t = 1f - smoothing;

            // ── 1. 루트 모션 (Hip 중심) — 부모부터 먼저 ──
            if (enableRootMotion && _hips != null
                && pose.HasJoint(JointNames.L_Hip) && pose.HasJoint(JointNames.R_Hip))
            {
                Vector3 hipCenter = (pose.GetJoint(JointNames.L_Hip) + pose.GetJoint(JointNames.R_Hip)) * 0.5f;
                float rt = 1f - rootSmoothing;
                _curRootPos = Vector3.Lerp(_curRootPos, hipCenter, rt);
                _hips.position = _curRootPos;
            }

            // ── 2. 몸통: Spine (hip→shoulder 방향) ──
            if (pose.HasJoint(JointNames.L_Shoulder) && pose.HasJoint(JointNames.R_Shoulder)
                && pose.HasJoint(JointNames.L_Hip) && pose.HasJoint(JointNames.R_Hip))
            {
                Vector3 shoulderMid = (pose.GetJoint(JointNames.L_Shoulder) + pose.GetJoint(JointNames.R_Shoulder)) * 0.5f;
                Vector3 hipMid = (pose.GetJoint(JointNames.L_Hip) + pose.GetJoint(JointNames.R_Hip)) * 0.5f;
                Vector3 spineDir = (shoulderMid - hipMid).normalized;

                if (spineDir.sqrMagnitude > 0.01f && _spine != null)
                {
                    Quaternion delta = Quaternion.FromToRotation(_bindSpineDir, spineDir);
                    Quaternion target = delta * _bindRotSpine;
                    _curSpine = Quaternion.Slerp(_curSpine, target, t);
                    _spine.rotation = _curSpine;
                }
            }

            // ── 3. 머리: Nose 방향 ──
            if (pose.HasJoint(JointNames.Nose) && pose.HasJoint(JointNames.L_Shoulder)
                && pose.HasJoint(JointNames.R_Shoulder) && _head != null)
            {
                Vector3 shoulderMid = (pose.GetJoint(JointNames.L_Shoulder) + pose.GetJoint(JointNames.R_Shoulder)) * 0.5f;
                Vector3 headDir = (pose.GetJoint(JointNames.Nose) - shoulderMid).normalized;

                if (headDir.sqrMagnitude > 0.01f)
                {
                    Quaternion delta = Quaternion.FromToRotation(_bindSpineDir, headDir);
                    Quaternion target = delta * _bindRotHead;
                    _curHead = Quaternion.Slerp(_curHead, target, t);
                    _head.rotation = _curHead;
                }
            }

            // ── 4. 상체: 팔 (상완 → 하완 순서) ──
            ApplyBoneRotation(_leftUpperArm, ref _curLeftUpperArm,
                _bindLeftUpperArm, _bindRotLeftUpperArm,
                pose.GetDirection(JointNames.L_Shoulder, JointNames.L_Elbow), t);

            ApplyBoneRotation(_rightUpperArm, ref _curRightUpperArm,
                _bindRightUpperArm, _bindRotRightUpperArm,
                pose.GetDirection(JointNames.R_Shoulder, JointNames.R_Elbow), t);

            ApplyBoneRotation(_leftLowerArm, ref _curLeftLowerArm,
                _bindLeftLowerArm, _bindRotLeftLowerArm,
                pose.GetDirection(JointNames.L_Elbow, JointNames.L_Wrist), t);

            ApplyBoneRotation(_rightLowerArm, ref _curRightLowerArm,
                _bindRightLowerArm, _bindRotRightLowerArm,
                pose.GetDirection(JointNames.R_Elbow, JointNames.R_Wrist), t);

            // ── 5. 하체: 다리 (대퇴 → 하퇴 순서) ──
            ApplyBoneRotation(_leftUpperLeg, ref _curLeftUpperLeg,
                _bindLeftUpperLeg, _bindRotLeftUpperLeg,
                pose.GetDirection(JointNames.L_Hip, JointNames.L_Knee), t);

            ApplyBoneRotation(_rightUpperLeg, ref _curRightUpperLeg,
                _bindRightUpperLeg, _bindRotRightUpperLeg,
                pose.GetDirection(JointNames.R_Hip, JointNames.R_Knee), t);

            ApplyBoneRotation(_leftLowerLeg, ref _curLeftLowerLeg,
                _bindLeftLowerLeg, _bindRotLeftLowerLeg,
                pose.GetDirection(JointNames.L_Knee, JointNames.L_Ankle), t);

            ApplyBoneRotation(_rightLowerLeg, ref _curRightLowerLeg,
                _bindRightLowerLeg, _bindRotRightLowerLeg,
                pose.GetDirection(JointNames.R_Knee, JointNames.R_Ankle), t);
        }

        // ── 유틸리티 ──

        void ApplyBoneRotation(Transform bone, ref Quaternion current,
            Vector3 bindDir, Quaternion bindRot, Vector3 poseDir, float t)
        {
            if (bone == null || poseDir == Vector3.zero) return;

            Quaternion delta = Quaternion.FromToRotation(bindDir, poseDir);
            Quaternion target = delta * bindRot;
            current = Quaternion.Slerp(current, target, t);
            bone.rotation = current;
        }

        Vector3 GetBindDirection(Transform parent, Transform child)
        {
            if (parent == null || child == null) return Vector3.down;
            return (child.position - parent.position).normalized;
        }
    }
}
