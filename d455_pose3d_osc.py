"""
d455_pose3d_osc.py
RealSense D455 + MediaPipe Pose → 전신 관절 OSC 송신 (27 관절)

OSC 메시지 형식:
  주소: /pose/<joint_name>
  인수: float x, float y, float z  (단위: m, 카메라 기준 좌표)

  전신 관절 목록:
    머리:  Nose, L_Eye, R_Eye, L_Ear, R_Ear
    상체:  L_Shoulder, R_Shoulder, L_Elbow, R_Elbow, L_Wrist, R_Wrist
    손:    L_Thumb, R_Thumb, L_Index, R_Index, L_Pinky, R_Pinky
    하체:  L_Hip, R_Hip, L_Knee, R_Knee, L_Ankle, R_Ankle
    발:    L_Heel, R_Heel, L_Foot, R_Foot

필요 패키지:
  pip install python-osc
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    PoseLandmarker,
    PoseLandmarkerOptions,
    PoseLandmark,
    PoseLandmarksConnections,
    RunningMode,
)
from pythonosc.udp_client import SimpleUDPClient
from pythonosc.osc_bundle_builder import OscBundleBuilder
from pythonosc.osc_message_builder import OscMessageBuilder
import pythonosc.osc_bundle_builder as osc_bundle_builder

# ══════════════════════════════════════════
#  칼만 필터 (관절 좌표 지터 제거)
# ══════════════════════════════════════════
# Q: 프로세스 노이즈 (클수록 빠른 움직임 추적↑, 스무딩↓)
# R: 측정 노이즈  (클수록 스무딩↑, 반응 속도↓)
KALMAN_Q = 0.001   # 관절이 프레임 간 얼마나 변할 수 있는지
KALMAN_R = 0.02    # 센서(깊이 카메라) 측정 노이즈 수준


class _KF1D:
    """1축 스칼라 칼만 필터 (등속도 없는 상수 위치 모델)"""
    __slots__ = ("x", "P", "Q", "R", "ready")

    def __init__(self, q: float, r: float):
        self.Q = q
        self.R = r
        self.x = 0.0
        self.P = 1.0
        self.ready = False

    def update(self, z: float) -> float:
        if not self.ready:
            self.x = z
            self.ready = True
            return z
        # 예측
        P_pred = self.P + self.Q
        # 칼만 게인
        K = P_pred / (P_pred + self.R)
        # 상태 갱신
        self.x += K * (z - self.x)
        self.P = (1.0 - K) * P_pred
        return self.x


class JointKalmanFilter:
    """관절 1개에 대한 XYZ 3축 칼만 필터"""
    __slots__ = ("fx", "fy", "fz")

    def __init__(self, q: float = KALMAN_Q, r: float = KALMAN_R):
        self.fx = _KF1D(q, r)
        self.fy = _KF1D(q, r)
        self.fz = _KF1D(q, r)

    def update(self, x: float, y: float, z: float):
        return (self.fx.update(x),
                self.fy.update(y),
                self.fz.update(z))


# ══════════════════════════════════════════
#  OSC 설정
# ══════════════════════════════════════════
OSC_IP   = "127.0.0.1"
OSC_PORT = 9000

osc_client = SimpleUDPClient(OSC_IP, OSC_PORT)
print(f"[OSC] 송신 대상: {OSC_IP}:{OSC_PORT}")

# ══════════════════════════════════════════
#  모델 경로
# ══════════════════════════════════════════
MODEL_PATH = r"c:\0.shinhyoung\Project\realsenseTest\pose_landmarker_lite.task"

# ══════════════════════════════════════════
#  전신 관절 정의 (27개)
# ══════════════════════════════════════════
LANDMARKS = {
    # ── 머리 ──────────────────────────────
    "Nose":      PoseLandmark.NOSE,
    "L_Eye":     PoseLandmark.LEFT_EYE,
    "R_Eye":     PoseLandmark.RIGHT_EYE,
    "L_Ear":     PoseLandmark.LEFT_EAR,
    "R_Ear":     PoseLandmark.RIGHT_EAR,
    # ── 상체 ──────────────────────────────
    "L_Shoulder":PoseLandmark.LEFT_SHOULDER,
    "R_Shoulder":PoseLandmark.RIGHT_SHOULDER,
    "L_Elbow":   PoseLandmark.LEFT_ELBOW,
    "R_Elbow":   PoseLandmark.RIGHT_ELBOW,
    "L_Wrist":   PoseLandmark.LEFT_WRIST,
    "R_Wrist":   PoseLandmark.RIGHT_WRIST,
    # ── 손 ────────────────────────────────
    "L_Thumb":   PoseLandmark.LEFT_THUMB,
    "R_Thumb":   PoseLandmark.RIGHT_THUMB,
    "L_Index":   PoseLandmark.LEFT_INDEX,
    "R_Index":   PoseLandmark.RIGHT_INDEX,
    "L_Pinky":   PoseLandmark.LEFT_PINKY,
    "R_Pinky":   PoseLandmark.RIGHT_PINKY,
    # ── 하체 ──────────────────────────────
    "L_Hip":     PoseLandmark.LEFT_HIP,
    "R_Hip":     PoseLandmark.RIGHT_HIP,
    "L_Knee":    PoseLandmark.LEFT_KNEE,
    "R_Knee":    PoseLandmark.RIGHT_KNEE,
    "L_Ankle":   PoseLandmark.LEFT_ANKLE,
    "R_Ankle":   PoseLandmark.RIGHT_ANKLE,
    # ── 발 ────────────────────────────────
    "L_Heel":    PoseLandmark.LEFT_HEEL,
    "R_Heel":    PoseLandmark.RIGHT_HEEL,
    "L_Foot":    PoseLandmark.LEFT_FOOT_INDEX,
    "R_Foot":    PoseLandmark.RIGHT_FOOT_INDEX,
}

# 화면에 좌표를 표시할 주요 관절 (전체 표시 시 화면 넘침 방지)
DISPLAY_JOINTS = [
    "Nose",
    "L_Shoulder", "R_Shoulder",
    "L_Elbow",    "R_Elbow",
    "L_Wrist",    "R_Wrist",
    "L_Hip",      "R_Hip",
    "L_Knee",     "R_Knee",
    "L_Ankle",    "R_Ankle",
]

CONNECTIONS = list(PoseLandmarksConnections.POSE_LANDMARKS)

# ══════════════════════════════════════════
#  MediaPipe PoseLandmarker 초기화
# ══════════════════════════════════════════
options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=RunningMode.VIDEO,
    num_poses=1,
    min_pose_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)
landmarker = PoseLandmarker.create_from_options(options)

# ══════════════════════════════════════════
#  RealSense 파이프라인 설정
# ══════════════════════════════════════════
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)

align   = rs.align(rs.stream.color)
profile = pipeline.start(config)

color_intrinsics = (
    profile.get_stream(rs.stream.color)
    .as_video_stream_profile()
    .get_intrinsics()
)

frame_timestamp_ms = 0

# 관절별 칼만 필터 (첫 수신 시 자동 생성)
kalman_filters: dict[str, JointKalmanFilter] = {}
kalman_enabled = True   # 'k' 키로 ON/OFF


# ══════════════════════════════════════════
#  OSC Bundle 송신
# ══════════════════════════════════════════
def send_osc_bundle(joint_data: dict):
    """모든 관절을 하나의 OSC Bundle로 묶어 UDP 송신"""
    builder = OscBundleBuilder(osc_bundle_builder.IMMEDIATELY)
    for name, (x, y, z) in joint_data.items():
        msg = OscMessageBuilder(address=f"/pose/{name}")
        msg.add_arg(float(x))
        msg.add_arg(float(y))
        msg.add_arg(float(z))
        builder.add_content(msg.build())
    bundle = builder.build()
    osc_client._sock.sendto(bundle.dgram, (OSC_IP, OSC_PORT))


# ══════════════════════════════════════════
#  메인 루프
# ══════════════════════════════════════════
try:
    while True:
        frames  = pipeline.wait_for_frames()
        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        h, w = color_image.shape[:2]

        # ── MediaPipe 추론 ──
        rgb      = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        frame_timestamp_ms += 33
        results = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

        # ── 깊이 JET 컬러맵 ──
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )

        if results.pose_landmarks:
            landmarks  = results.pose_landmarks[0]
            joint_data = {}   # OSC 송신용: 전체 27개

            # 스켈레톤 연결선
            for conn in CONNECTIONS:
                s = landmarks[conn.start]
                e = landmarks[conn.end]
                cv2.line(color_image,
                         (int(s.x * w), int(s.y * h)),
                         (int(e.x * w), int(e.y * h)),
                         (200, 200, 200), 1)

            # 모든 랜드마크 점 (초록)
            for lm in landmarks:
                px, py = int(lm.x * w), int(lm.y * h)
                cv2.circle(color_image, (px, py), 3, (0, 255, 0), -1)

            # ── 전신 3D 좌표 계산 ──────────────────
            y_offset = 18
            for name, lm_id in LANDMARKS.items():
                lm = landmarks[lm_id.value]
                px, py = int(lm.x * w), int(lm.y * h)
                px_c = max(0, min(px, w - 1))
                py_c = max(0, min(py, h - 1))

                depth_val = depth_frame.get_distance(px_c, py_c)
                point_3d  = rs.rs2_deproject_pixel_to_point(
                    color_intrinsics, [px_c, py_c], depth_val
                )
                x3, y3, z3 = point_3d

                # 유효한 깊이만 칼만 필터 적용 후 송신
                if depth_val > 0.0:
                    if kalman_enabled:
                        if name not in kalman_filters:
                            kalman_filters[name] = JointKalmanFilter()
                        x3, y3, z3 = kalman_filters[name].update(x3, y3, z3)
                    joint_data[name] = (x3, y3, z3)

                # 주요 관절만 화면 표시
                if name in DISPLAY_JOINTS:
                    color  = (0, 255, 255) if depth_val > 0 else (0, 100, 200)
                    cv2.circle(color_image, (px, py), 5, (0, 0, 255), -1)
                    text = f"{name}:({x3:+.2f},{y3:+.2f},{z3:.2f})"
                    cv2.putText(color_image, text, (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)
                    y_offset += 17

            # ── OSC Bundle 송신 (전체 27개) ────────
            if joint_data:
                try:
                    send_osc_bundle(joint_data)
                except Exception as e:
                    print(f"[OSC] 송신 오류: {e}")

            # 상태 표시
            cv2.putText(
                color_image,
                f"OSC -> {OSC_IP}:{OSC_PORT}  joints:{len(joint_data)}/27",
                (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1,
            )

        else:
            # 포즈 미검출 시 (0,0,0) 송신
            try:
                send_osc_bundle({n: (0.0, 0.0, 0.0) for n in LANDMARKS})
            except Exception:
                pass
            cv2.putText(color_image, "No Pose Detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # ── 칼만 필터 상태 표시 (우상단, 항상 표시) ──
        kalman_label = "Kalman: ON  [K]" if kalman_enabled else "Kalman: OFF [K]"
        kalman_color = (0, 255, 100) if kalman_enabled else (0, 80, 255)
        cv2.putText(color_image, kalman_label,
                    (w - 175, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, kalman_color, 2)

        # ── 화면 표시 ──
        display = np.hstack((color_image, depth_colormap))
        cv2.imshow("D455 Full Body OSC | 'q' to quit", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("k"):
            kalman_enabled = not kalman_enabled
            # OFF 시 필터 상태 초기화 (재활성화 시 누적값 없이 시작)
            if not kalman_enabled:
                kalman_filters.clear()
            print(f"[Kalman] {'ON' if kalman_enabled else 'OFF'}")

finally:
    landmarker.close()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[완료] 프로그램 종료")
