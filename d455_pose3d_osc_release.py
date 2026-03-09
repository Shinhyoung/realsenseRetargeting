"""
d455_pose3d_osc_release.py
Release (exe) 빌드용 — 모델 경로를 exe 기준 상대 경로로 변경
나머지 로직은 d455_pose3d_osc.py 와 동일
"""

import sys
import os
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
KALMAN_Q = 0.001
KALMAN_R = 0.02


class _KF1D:
    """1축 스칼라 칼만 필터 (상수 위치 모델)"""
    __slots__ = ("x", "P", "Q", "R", "ready")

    def __init__(self, q: float, r: float):
        self.Q = q; self.R = r
        self.x = 0.0; self.P = 1.0; self.ready = False

    def update(self, z: float) -> float:
        if not self.ready:
            self.x = z; self.ready = True; return z
        P_pred = self.P + self.Q
        K = P_pred / (P_pred + self.R)
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
#  다인원 설정
# ══════════════════════════════════════════
MAX_PERSONS = 3

PERSON_COLORS = [
    (0,   255, 255),
    (0,   255,   0),
    (255, 100,   0),
]

# ══════════════════════════════════════════
#  OSC 설정
# ══════════════════════════════════════════
OSC_IP   = "127.0.0.1"
OSC_PORT = 9000

osc_client = SimpleUDPClient(OSC_IP, OSC_PORT)
print(f"[OSC] 송신 대상: {OSC_IP}:{OSC_PORT}  (최대 {MAX_PERSONS}명)")

# ══════════════════════════════════════════
#  모델 경로 (exe 기준 상대 경로)
# ══════════════════════════════════════════
if getattr(sys, 'frozen', False):
    BASE_DIR = sys._MEIPASS if hasattr(sys, '_MEIPASS') else os.path.dirname(sys.executable)
else:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))

MODEL_PATH = os.path.join(BASE_DIR, "pose_landmarker_lite.task")
if not os.path.exists(MODEL_PATH):
    # onedir 모드: exe와 같은 폴더에 있음
    MODEL_PATH = os.path.join(os.path.dirname(sys.executable), "pose_landmarker_lite.task")

print(f"[모델] {MODEL_PATH}")

# ══════════════════════════════════════════
#  전신 관절 정의 (27개)
# ══════════════════════════════════════════
LANDMARKS = {
    "Nose":       PoseLandmark.NOSE,
    "L_Eye":      PoseLandmark.LEFT_EYE,
    "R_Eye":      PoseLandmark.RIGHT_EYE,
    "L_Ear":      PoseLandmark.LEFT_EAR,
    "R_Ear":      PoseLandmark.RIGHT_EAR,
    "L_Shoulder": PoseLandmark.LEFT_SHOULDER,
    "R_Shoulder": PoseLandmark.RIGHT_SHOULDER,
    "L_Elbow":    PoseLandmark.LEFT_ELBOW,
    "R_Elbow":    PoseLandmark.RIGHT_ELBOW,
    "L_Wrist":    PoseLandmark.LEFT_WRIST,
    "R_Wrist":    PoseLandmark.RIGHT_WRIST,
    "L_Thumb":    PoseLandmark.LEFT_THUMB,
    "R_Thumb":    PoseLandmark.RIGHT_THUMB,
    "L_Index":    PoseLandmark.LEFT_INDEX,
    "R_Index":    PoseLandmark.RIGHT_INDEX,
    "L_Pinky":    PoseLandmark.LEFT_PINKY,
    "R_Pinky":    PoseLandmark.RIGHT_PINKY,
    "L_Hip":      PoseLandmark.LEFT_HIP,
    "R_Hip":      PoseLandmark.RIGHT_HIP,
    "L_Knee":     PoseLandmark.LEFT_KNEE,
    "R_Knee":     PoseLandmark.RIGHT_KNEE,
    "L_Ankle":    PoseLandmark.LEFT_ANKLE,
    "R_Ankle":    PoseLandmark.RIGHT_ANKLE,
    "L_Heel":     PoseLandmark.LEFT_HEEL,
    "R_Heel":     PoseLandmark.RIGHT_HEEL,
    "L_Foot":     PoseLandmark.LEFT_FOOT_INDEX,
    "R_Foot":     PoseLandmark.RIGHT_FOOT_INDEX,
}

DISPLAY_JOINTS = [
    "Nose",
    "L_Shoulder", "R_Shoulder",
    "L_Wrist",    "R_Wrist",
    "L_Hip",      "R_Hip",
    "L_Ankle",    "R_Ankle",
]

CONNECTIONS = list(PoseLandmarksConnections.POSE_LANDMARKS)

# ══════════════════════════════════════════
#  MediaPipe PoseLandmarker 초기화
# ══════════════════════════════════════════
options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=RunningMode.VIDEO,
    num_poses=MAX_PERSONS,
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

kalman_filters: dict[int, dict[str, JointKalmanFilter]] = {}
kalman_filters_2d: dict[int, dict[int, tuple]] = {}
kalman_enabled = True


# ══════════════════════════════════════════
#  OSC Bundle 송신
# ══════════════════════════════════════════
def send_osc_bundle(person_id: int, joint_data: dict):
    builder = OscBundleBuilder(osc_bundle_builder.IMMEDIATELY)
    for name, (x, y, z) in joint_data.items():
        msg = OscMessageBuilder(address=f"/pose/{person_id}/{name}")
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

        rgb      = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        frame_timestamp_ms += 33
        results = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )

        detected_count = 0

        if results.pose_landmarks:
            detected_count = len(results.pose_landmarks)

            for person_id, landmarks in enumerate(results.pose_landmarks):
                color = PERSON_COLORS[person_id % len(PERSON_COLORS)]

                if person_id not in kalman_filters:
                    kalman_filters[person_id] = {}
                if person_id not in kalman_filters_2d:
                    kalman_filters_2d[person_id] = {}

                # 2D 픽셀 좌표 칼만 필터 적용
                filtered_px = []
                kf2d = kalman_filters_2d[person_id]
                for idx, lm in enumerate(landmarks):
                    raw_x, raw_y = lm.x * w, lm.y * h
                    if kalman_enabled:
                        if idx not in kf2d:
                            kf2d[idx] = (_KF1D(KALMAN_Q, KALMAN_R),
                                         _KF1D(KALMAN_Q, KALMAN_R))
                        fx, fy = kf2d[idx]
                        raw_x = fx.update(raw_x)
                        raw_y = fy.update(raw_y)
                    filtered_px.append((int(raw_x), int(raw_y)))

                # 스켈레톤 연결선
                for conn in CONNECTIONS:
                    cv2.line(color_image,
                             filtered_px[conn.start],
                             filtered_px[conn.end],
                             color, 1)

                # 랜드마크 점
                for pt in filtered_px:
                    cv2.circle(color_image, pt, 3, color, -1)

                # 전신 3D 좌표 계산 + OSC 송신
                joint_data = {}
                min_y = int(min(lm.y for lm in landmarks) * h)
                y_offset = max(18, min_y)

                for name, lm_id in LANDMARKS.items():
                    lm = landmarks[lm_id.value]
                    px, py = filtered_px[lm_id.value]
                    px_c = max(0, min(px, w - 1))
                    py_c = max(0, min(py, h - 1))

                    depth_val = depth_frame.get_distance(px_c, py_c)
                    point_3d  = rs.rs2_deproject_pixel_to_point(
                        color_intrinsics, [px_c, py_c], depth_val
                    )
                    x3, y3, z3 = point_3d

                    if depth_val > 0.0:
                        if kalman_enabled:
                            kf = kalman_filters[person_id]
                            if name not in kf:
                                kf[name] = JointKalmanFilter()
                            x3, y3, z3 = kf[name].update(x3, y3, z3)
                        joint_data[name] = (x3, y3, z3)

                    if name in DISPLAY_JOINTS:
                        cv2.circle(color_image, (px, py), 6, color, -1)
                        if name == "Nose":
                            cv2.putText(color_image,
                                        f"P{person_id}",
                                        (px + 8, py),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        text = f"P{person_id} {name}:({x3:+.2f},{z3:.2f}m)"
                        cv2.putText(color_image, text, (10, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
                        y_offset += 15

                if joint_data:
                    try:
                        send_osc_bundle(person_id, joint_data)
                    except Exception as e:
                        print(f"[OSC] P{person_id} 송신 오류: {e}")

        else:
            cv2.putText(color_image, "No Pose Detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(color_image,
                    f"OSC -> {OSC_IP}:{OSC_PORT}  Persons:{detected_count}/{MAX_PERSONS}",
                    (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)

        kalman_label = "Kalman: ON  [K]" if kalman_enabled else "Kalman: OFF [K]"
        kalman_color = (0, 255, 100) if kalman_enabled else (0, 80, 255)
        cv2.putText(color_image, kalman_label,
                    (w - 175, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, kalman_color, 2)

        display = np.hstack((color_image, depth_colormap))
        cv2.imshow("D455 Multi-Person OSC | 'q' quit  'k' kalman", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("k"):
            kalman_enabled = not kalman_enabled
            if not kalman_enabled:
                kalman_filters.clear()
                kalman_filters_2d.clear()
            print(f"[Kalman] {'ON' if kalman_enabled else 'OFF'}")

finally:
    landmarker.close()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[완료] 프로그램 종료")
