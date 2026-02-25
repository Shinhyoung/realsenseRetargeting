import pyrealsense2 as rs
import numpy as np
import cv2

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()

# 컬러 및 깊이 스트림 활성화 (640x480, 30fps)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# IMU (가속도계) 스트림 활성화
config.enable_stream(rs.stream.accel)

# 깊이 → 컬러 정렬 객체
align = rs.align(rs.stream.color)

# 파이프라인 시작
profile = pipeline.start(config)

# 깊이 스케일 정보
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# IMU 값 저장 변수
accel_data = [0.0, 0.0, 0.0]

try:
    while True:
        # 프레임 수신
        frames = pipeline.wait_for_frames()

        # IMU 데이터 추출
        for frame in frames:
            if frame.is_motion_frame():
                motion = frame.as_motion_frame()
                if motion.get_profile().stream_type() == rs.stream.accel:
                    accel = motion.get_motion_data()
                    accel_data = [accel.x, accel.y, accel.z]

        # 깊이를 컬러에 정렬
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 깊이 이미지에 JET 컬러맵 적용
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )

        # IMU 가속도 텍스트를 깊이 이미지에 표시
        imu_text = f"Accel X:{accel_data[0]:+.2f} Y:{accel_data[1]:+.2f} Z:{accel_data[2]:+.2f}"
        cv2.putText(depth_colormap, imu_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 컬러와 깊이를 나란히 배치
        images = np.hstack((color_image, depth_colormap))

        cv2.imshow("D455 Color + Depth (Aligned) | Press 'q' to quit", images)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
