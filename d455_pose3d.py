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

# ── 모델 경로 ──
MODEL_PATH = r"c:\0.shinhyoung\Project\realsenseTest\pose_landmarker_lite.task"

# ── 추적할 랜드마크 (좌/우 어깨, 팔꿈치, 손목) ──
LANDMARKS = {
    "L_Shoulder": PoseLandmark.LEFT_SHOULDER,
    "R_Shoulder": PoseLandmark.RIGHT_SHOULDER,
    "L_Elbow":    PoseLandmark.LEFT_ELBOW,
    "R_Elbow":    PoseLandmark.RIGHT_ELBOW,
    "L_Wrist":    PoseLandmark.LEFT_WRIST,
    "R_Wrist":    PoseLandmark.RIGHT_WRIST,
}

# 스켈레톤 연결선 (어깨-팔꿈치-손목)
CONNECTIONS = list(PoseLandmarksConnections.POSE_LANDMARKS)

# ── MediaPipe PoseLandmarker 초기화 (VIDEO 모드) ──
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
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 깊이 → 컬러 정렬
align = rs.align(rs.stream.color)

# 파이프라인 시작
profile = pipeline.start(config)

# 깊이 intrinsics (픽셀 → 3D 변환용)
depth_intrinsics = (
    profile.get_stream(rs.stream.depth)
    .as_video_stream_profile()
    .get_intrinsics()
)

frame_timestamp_ms = 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        h, w = color_image.shape[:2]

        # ── MediaPipe Pose 추론 ──
        rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        frame_timestamp_ms += 33  # ~30fps
        results = landmarker.detect_for_video(mp_image, frame_timestamp_ms)

        # ── 깊이 JET 컬러맵 ──
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )

        if results.pose_landmarks:
            landmarks = results.pose_landmarks[0]  # 첫 번째 사람

            # 스켈레톤 연결선 그리기
            for conn in CONNECTIONS:
                start = landmarks[conn.start]
                end = landmarks[conn.end]
                pt1 = (int(start.x * w), int(start.y * h))
                pt2 = (int(end.x * w), int(end.y * h))
                cv2.line(color_image, pt1, pt2, (255, 255, 255), 2)

            # 모든 랜드마크 점 그리기
            for lm in landmarks:
                px, py = int(lm.x * w), int(lm.y * h)
                cv2.circle(color_image, (px, py), 3, (0, 255, 0), -1)

            # 3D 좌표 계산 및 표시
            y_offset = 25
            for name, lm_id in LANDMARKS.items():
                lm = landmarks[lm_id.value]
                px, py = int(lm.x * w), int(lm.y * h)

                # 이미지 범위 내 클리핑
                px_c = max(0, min(px, w - 1))
                py_c = max(0, min(py, h - 1))

                # 깊이값 (m)
                depth_val = depth_frame.get_distance(px_c, py_c)

                # 픽셀 → 3D 좌표 (카메라 기준, 단위: m)
                point_3d = rs.rs2_deproject_pixel_to_point(
                    depth_intrinsics, [px_c, py_c], depth_val
                )
                x3, y3, z3 = point_3d

                # 랜드마크 위치에 빨간 원
                cv2.circle(color_image, (px, py), 6, (0, 0, 255), -1)

                # 좌표 텍스트 (컬러 영상 좌측 상단)
                coord_text = f"{name}: ({x3:+.3f}, {y3:+.3f}, {z3:.3f})m"
                cv2.putText(
                    color_image, coord_text, (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                )
                y_offset += 22

        # 나란히 표시
        display = np.hstack((color_image, depth_colormap))
        cv2.imshow("D455 Pose 3D | Press 'q' to quit", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    landmarker.close()
    pipeline.stop()
    cv2.destroyAllWindows()
