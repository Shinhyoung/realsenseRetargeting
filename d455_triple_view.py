import pyrealsense2 as rs
import numpy as np
import cv2

# ── RealSense 파이프라인 설정 ──
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  # 왼쪽 IR

pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        ir_frame = frames.get_infrared_frame(1)  # 왼쪽 IR (index 1)

        if not color_frame or not depth_frame or not ir_frame:
            continue

        # numpy 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        ir_image = np.asanyarray(ir_frame.get_data())

        # 깊이 → JET 컬러맵
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )

        # IR 히스토그램 이퀄라이즈
        ir_equalized = cv2.equalizeHist(ir_image)

        # 3개 창 표시
        cv2.imshow("Color", color_image)
        cv2.imshow("Depth (JET)", depth_colormap)
        cv2.imshow("Left IR (Equalized)", ir_equalized)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
