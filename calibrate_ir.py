"""
D455 IR 카메라 체커보드 캘리브레이션
───────────────────────────────────
사용법:
  1. 체커보드(9x6 내부 코너 기본)를 IR 카메라 앞에 놓는다.
  2. 'c' 키 : 현재 프레임 캡처 (최소 10장 권장, 다양한 각도/위치)
  3. 's' 키 : 캡처된 프레임으로 캘리브레이션 수행 → ir_calibration.npz 저장
  4. 'q' 키 : 종료
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import os

# ── 체커보드 파라미터 (내부 코너 수) ──
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 25.0  # mm (실제 체커보드 사각형 크기, 정확하지 않아도 왜곡 보정에는 문제없음)

SAVE_PATH = os.path.join(os.path.dirname(__file__), "ir_calibration.npz")

# 3D 월드 좌표 (z=0 평면)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ── RealSense 설정 ──
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.emitter_enabled, 0)

obj_points = []  # 3D
img_points = []  # 2D
capture_count = 0

print("=" * 50)
print("IR Checkerboard Calibration")
print("  'c' = capture  |  's' = save & calibrate  |  'q' = quit")
print(f"  Checkerboard: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners")
print("=" * 50)

try:
    while True:
        frames = pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame(1)
        if not ir_frame:
            continue

        ir_image = np.asanyarray(ir_frame.get_data())
        display = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)

        # 체커보드 코너 검출
        found, corners = cv2.findChessboardCorners(ir_image, CHECKERBOARD, None)
        if found:
            corners_refined = cv2.cornerSubPix(ir_image, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners_refined, found)

        cv2.putText(display, f"Captured: {capture_count} | 'c'=capture 's'=calibrate 'q'=quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("IR Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("c") and found:
            obj_points.append(objp)
            img_points.append(corners_refined)
            capture_count += 1
            print(f"  [Captured #{capture_count}]")

        elif key == ord("c") and not found:
            print("  [!] Checkerboard not detected - try adjusting position")

        elif key == ord("s"):
            if capture_count < 3:
                print(f"  [!] Need at least 3 captures (current: {capture_count})")
                continue

            print("\n  Calibrating...")
            ret, cam_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, ir_image.shape[::-1], None, None
            )
            print(f"  RMS error: {ret:.4f}")
            print(f"  Camera matrix:\n{cam_mtx}")
            print(f"  Distortion coeffs: {dist.ravel()}")

            np.savez(SAVE_PATH, cam_mtx=cam_mtx, dist=dist)
            print(f"\n  Saved to: {SAVE_PATH}")
            break

        elif key == ord("q"):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
