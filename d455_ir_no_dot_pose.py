import pyrealsense2 as rs
import numpy as np
import cv2
import os

# ── RealSense 파이프라인 설정 ──
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# ── IR 도트 패턴(emitter) 끄기 ──
depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.emitter_enabled, 0)
print("[INFO] IR emitter disabled (dot pattern OFF)")

# ── 카메라 내부 파라미터(intrinsics) → undistort 맵 생성 ──
def build_undistort_map(stream_profile):
    intr = stream_profile.as_video_stream_profile().get_intrinsics()
    cam_mtx = np.array([
        [intr.fx, 0,       intr.ppx],
        [0,       intr.fy, intr.ppy],
        [0,       0,       1       ],
    ], dtype=np.float64)
    dist = np.array(intr.coeffs, dtype=np.float64)
    w, h = intr.width, intr.height
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(cam_mtx, dist, (w, h), 0, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(cam_mtx, dist, None, new_mtx, (w, h), cv2.CV_16SC2)
    print(f"  [{stream_profile.stream_type()}] distortion model: {intr.model}, coeffs: {intr.coeffs}")
    return map1, map2

# ── IR 캘리브레이션 파일이 있으면 사용, 없으면 SDK intrinsics 사용 ──
IR_CALIB_PATH = os.path.join(os.path.dirname(__file__), "ir_calibration.npz")

if os.path.exists(IR_CALIB_PATH):
    calib = np.load(IR_CALIB_PATH)
    ir_cam_mtx = calib["cam_mtx"]
    ir_dist = calib["dist"]
    ir_new_mtx, _ = cv2.getOptimalNewCameraMatrix(ir_cam_mtx, ir_dist, (640, 480), 0, (640, 480))
    ir_map1, ir_map2 = cv2.initUndistortRectifyMap(ir_cam_mtx, ir_dist, None, ir_new_mtx, (640, 480), cv2.CV_16SC2)
    print(f"[INFO] IR calibration loaded from {IR_CALIB_PATH}")
else:
    ir_map1, ir_map2 = build_undistort_map(profile.get_stream(rs.stream.infrared, 1))
    print("[INFO] IR: using SDK intrinsics (run calibrate_ir.py for better results)")

COLOR_CALIB_PATH = os.path.join(os.path.dirname(__file__), "color_calibration.npz")

if os.path.exists(COLOR_CALIB_PATH):
    calib_c = np.load(COLOR_CALIB_PATH)
    color_cam_mtx = calib_c["cam_mtx"]
    color_dist = calib_c["dist"]
    color_new_mtx, _ = cv2.getOptimalNewCameraMatrix(color_cam_mtx, color_dist, (640, 480), 0, (640, 480))
    color_map1, color_map2 = cv2.initUndistortRectifyMap(color_cam_mtx, color_dist, None, color_new_mtx, (640, 480), cv2.CV_16SC2)
    print(f"[INFO] Color calibration loaded from {COLOR_CALIB_PATH}")
else:
    color_map1, color_map2 = build_undistort_map(profile.get_stream(rs.stream.color))
    print("[INFO] Color: using SDK intrinsics (run calibrate_color.py for better results)")

print("[INFO] Undistortion maps ready")

try:
    while True:
        frames = pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame(1)
        color_frame = frames.get_color_frame()
        if not ir_frame or not color_frame:
            continue

        color_image = cv2.remap(np.asanyarray(color_frame.get_data()), color_map1, color_map2, cv2.INTER_LINEAR)

        ir_image = cv2.remap(np.asanyarray(ir_frame.get_data()), ir_map1, ir_map2, cv2.INTER_LINEAR)

        # # 이진화 (임계값 200 이상 → 흰색, 나머지 → 검정)
        # _, binary = cv2.threshold(ir_image, 200, 255, cv2.THRESH_BINARY)
        #
        # # 흰색 dot 컨투어 검출
        # contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #
        # # 시각화용 3채널 변환
        # display = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        #
        # dot_count = 0
        # for cnt in contours:
        #     area = cv2.contourArea(cnt)
        #     if area < 3:
        #         continue
        #
        #     # 최소 외접원 지름 필터: 5px 이하만 허용
        #     _, radius = cv2.minEnclosingCircle(cnt)
        #     diameter = radius * 2
        #     if diameter > 5:
        #         continue
        #
        #     # 원형도 필터: 4π·A / P² (1.0 = 완벽한 원)
        #     perimeter = cv2.arcLength(cnt, True)
        #     if perimeter == 0:
        #         continue
        #     circularity = 4 * np.pi * area / (perimeter * perimeter)
        #     if circularity < 0.5:
        #         continue
        #
        #     M = cv2.moments(cnt)
        #     if M["m00"] == 0:
        #         continue
        #     cx = int(M["m10"] / M["m00"])
        #     cy = int(M["m01"] / M["m00"])
        #
        #     # 중심점 표시
        #     cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
        #     cv2.putText(display, f"({cx},{cy})", (cx + 8, cy - 5),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        #     dot_count += 1
        #
        # # 검출 개수 표시
        # cv2.putText(display, f"Dots: {dot_count}", (10, 25),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 밝기 200 이상 픽셀만 추출 (나머지는 0)
        bright_mask = np.where(ir_image >= 200, ir_image, 0).astype(np.uint8)

        # 모폴로지 dilate
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated = cv2.dilate(bright_mask, kernel, iterations=1)

        # dilate 결과를 이진화
        _, binary = cv2.threshold(dilated, 1, 255, cv2.THRESH_BINARY)

        # 3채널 변환 (hstack을 위해)
        dilated_bgr = cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR)
        binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        combined = np.hstack((dilated_bgr, binary_bgr))
        cv2.imshow("Dilated (>=200) | Binary | 'q' to quit", combined)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
