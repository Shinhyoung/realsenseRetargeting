import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    PoseLandmarker,
    PoseLandmarkerOptions,
    PoseLandmarksConnections,
    RunningMode,
)

# ── 모델 경로 ──
MODEL_PATH = r"c:\0.shinhyoung\Project\realsenseTest\pose_landmarker_lite.task"

# 스켈레톤 연결선
CONNECTIONS = list(PoseLandmarksConnections.POSE_LANDMARKS)

# ── MediaPipe PoseLandmarker 초기화 ──
options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=RunningMode.VIDEO,
    num_poses=1,
    min_pose_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)
landmarker = PoseLandmarker.create_from_options(options)

# ── RealSense 파이프라인 설정 ──
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

pipeline.start(config)

frame_timestamp_ms = 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame(1)
        if not ir_frame:
            continue

        # IR 영상 (1채널 grayscale)
        ir_image = np.asanyarray(ir_frame.get_data())
        h, w = ir_image.shape[:2]

        # 1채널 → 3채널 BGR 변환 (MediaPipe 입력용)
        ir_bgr = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)

        # ── MediaPipe Pose 추론 ──
        ir_rgb = cv2.cvtColor(ir_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=ir_rgb)
        frame_timestamp_ms += 33
        results = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

        # 시각화용 복사본
        display = ir_bgr.copy()

        if results.pose_landmarks:
            landmarks = results.pose_landmarks[0]

            # 스켈레톤 연결선
            for conn in CONNECTIONS:
                s = landmarks[conn.start]
                e = landmarks[conn.end]
                pt1 = (int(s.x * w), int(s.y * h))
                pt2 = (int(e.x * w), int(e.y * h))
                cv2.line(display, pt1, pt2, (0, 255, 0), 2)

            # 랜드마크 점
            for lm in landmarks:
                px, py = int(lm.x * w), int(lm.y * h)
                cv2.circle(display, (px, py), 4, (0, 0, 255), -1)

        cv2.imshow("D455 Left IR + Pose | Press 'q' to quit", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    landmarker.close()
    pipeline.stop()
    cv2.destroyAllWindows()
