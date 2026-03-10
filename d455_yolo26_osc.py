"""
d455_yolo26_osc.py
RealSense D455 + YOLO26 Pose → 전신 관절 OSC 송신 (다인원 지원, ID 추적)

YOLO26 Pose는 COCO 17 키포인트를 검출합니다:
  Nose, L_Eye, R_Eye, L_Ear, R_Ear,
  L_Shoulder, R_Shoulder, L_Elbow, R_Elbow, L_Wrist, R_Wrist,
  L_Hip, R_Hip, L_Knee, R_Knee, L_Ankle, R_Ankle

OSC 메시지 형식:
  주소: /pose/<track_id>/<joint_name>
  인수: float x, float y, float z  (단위: m, 카메라 기준 좌표)

필요 패키지:
  pip install ultralytics python-osc pyrealsense2 opencv-python numpy
"""

import time
import torch
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from pythonosc.udp_client import SimpleUDPClient
from pythonosc.osc_bundle_builder import OscBundleBuilder
from pythonosc.osc_message_builder import OscMessageBuilder
import pythonosc.osc_bundle_builder as osc_bundle_builder

# ══════════════════════════════════════════
#  칼만 필터 (관절 좌표 지터 제거)
# ══════════════════════════════════════════
KALMAN_Q = 0.01    # 프로세스 노이즈 (클수록 빠른 움직임 추적↑)
KALMAN_R = 0.02    # 측정 노이즈   (클수록 스무딩↑)
KALMAN_Q_MIN = 0.001
KALMAN_Q_MAX = 0.5
KALMAN_Q_STEP = 2.0  # 배율 (×2 / ÷2)


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

    def set_q(self, q: float):
        self.fx.Q = q
        self.fy.Q = q
        self.fz.Q = q


def update_all_kalman_q(new_q: float):
    """모든 활성 칼만 필터의 Q 값을 일괄 업데이트"""
    global KALMAN_Q
    KALMAN_Q = new_q
    for filters in kalman_filters.values():
        for jkf in filters.values():
            jkf.set_q(new_q)
    for filters_2d in kalman_filters_2d.values():
        for (fx, fy) in filters_2d.values():
            fx.Q = new_q
            fy.Q = new_q


# ══════════════════════════════════════════
#  YOLO26 Pose 설정
# ══════════════════════════════════════════
YOLO_MODELS = {
    "n": "yolo26n-pose.pt",   # Nano:   빠름, 정확도 보통
    "m": "yolo26m-pose.pt",   # Medium: 느림, 정확도 높음
}
current_model_key = "n"          # 시작 모델
CONF_THRESHOLD = 0.5             # 검출 신뢰도 임계값

# COCO 17 키포인트 이름 매핑
KEYPOINT_NAMES = [
    "Nose",
    "L_Eye",    "R_Eye",
    "L_Ear",    "R_Ear",
    "L_Shoulder", "R_Shoulder",
    "L_Elbow",  "R_Elbow",
    "L_Wrist",  "R_Wrist",
    "L_Hip",    "R_Hip",
    "L_Knee",   "R_Knee",
    "L_Ankle",  "R_Ankle",
]

# 화면에 좌표를 표시할 주요 관절
DISPLAY_JOINTS = {
    "Nose",
    "L_Shoulder", "R_Shoulder",
    "L_Wrist",    "R_Wrist",
    "L_Hip",      "R_Hip",
    "L_Ankle",    "R_Ankle",
}

# 스켈레톤 연결선 (COCO 17 키포인트 인덱스 쌍)
SKELETON = [
    (0, 1), (0, 2),     # Nose → Eyes
    (1, 3), (2, 4),     # Eyes → Ears
    (5, 6),             # L_Shoulder → R_Shoulder
    (5, 7), (7, 9),     # L_Shoulder → L_Elbow → L_Wrist
    (6, 8), (8, 10),    # R_Shoulder → R_Elbow → R_Wrist
    (5, 11), (6, 12),   # Shoulders → Hips
    (11, 12),           # L_Hip → R_Hip
    (11, 13), (13, 15), # L_Hip → L_Knee → L_Ankle
    (12, 14), (14, 16), # R_Hip → R_Knee → R_Ankle
]

# 사람별 화면 표시 색상 (BGR) — 트래킹 ID 기반
PERSON_COLORS = [
    (0,   255, 255),   # 노란색
    (0,   255,   0),   # 초록
    (255, 100,   0),   # 파랑
    (255,   0, 255),   # 마젠타
    (0,   165, 255),   # 주황
    (255, 255,   0),   # 시안
]

# ══════════════════════════════════════════
#  OSC 설정
# ══════════════════════════════════════════
OSC_IP   = "127.0.0.1"
OSC_PORT = 9000

osc_client = SimpleUDPClient(OSC_IP, OSC_PORT)

# ══════════════════════════════════════════
#  GPU / CPU 설정
# ══════════════════════════════════════════
gpu_available = torch.cuda.is_available()
gpu_name = torch.cuda.get_device_name(0) if gpu_available else "N/A"
use_gpu = gpu_available   # GPU 있으면 기본 GPU 사용
current_device = "cuda" if use_gpu else "cpu"

print(f"[GPU] {'사용 가능: ' + gpu_name if gpu_available else '사용 불가 — CPU 모드'}")

# ══════════════════════════════════════════
#  YOLO26 모델 로드
# ══════════════════════════════════════════
# 모델 캐시: {(model_key, device): YOLO}
loaded_models: dict[tuple, YOLO] = {}


def load_model(key: str, device: str = None) -> YOLO:
    """모델을 로드하고 지정 디바이스로 이동, 캐시 저장"""
    if device is None:
        device = current_device
    cache_key = (key, device)
    if cache_key not in loaded_models:
        name = YOLO_MODELS[key]
        print(f"[YOLO26] 모델 로딩: {name} → {device.upper()} ...")
        m = YOLO(name)
        m.to(device)
        loaded_models[cache_key] = m
        print(f"[YOLO26] {name} ({device.upper()}) 로드 완료")
    return loaded_models[cache_key]


model = load_model(current_model_key, current_device)
print(f"[OSC] 송신 대상: {OSC_IP}:{OSC_PORT}")

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

# 칼만 필터: {track_id: {joint_name: JointKalmanFilter}}
kalman_filters: dict[int, dict[str, JointKalmanFilter]] = {}
# 2D 표시용 칼만 필터: {track_id: {kp_index: (KF_x, KF_y)}}
kalman_filters_2d: dict[int, dict[int, tuple]] = {}
kalman_enabled = True

# ══════════════════════════════════════════
#  장시간 구동 안정성: 사라진 ID 칼만 필터 정리
# ══════════════════════════════════════════
STALE_TIMEOUT = 5.0  # 초: 이 시간 동안 미검출이면 필터 삭제
kalman_last_seen: dict[int, float] = {}  # {track_id: last_seen_time}


def cleanup_stale_filters(current_ids: set):
    """현재 프레임에 없는 track_id의 칼만 필터를 일정 시간 후 삭제"""
    now = time.time()
    for tid in current_ids:
        kalman_last_seen[tid] = now
    stale = [tid for tid, t in kalman_last_seen.items()
             if now - t > STALE_TIMEOUT and tid not in current_ids]
    for tid in stale:
        kalman_filters.pop(tid, None)
        kalman_filters_2d.pop(tid, None)
        kalman_last_seen.pop(tid, None)


# ══════════════════════════════════════════
#  OSC Bundle 송신  /pose/<track_id>/<joint_name>
# ══════════════════════════════════════════
def send_osc_bundle(track_id: int, joint_data: dict):
    """한 사람의 관절을 OSC Bundle로 묶어 UDP 송신"""
    global osc_client
    builder = OscBundleBuilder(osc_bundle_builder.IMMEDIATELY)
    for name, (x, y, z) in joint_data.items():
        msg = OscMessageBuilder(address=f"/pose/{track_id}/{name}")
        msg.add_arg(float(x))
        msg.add_arg(float(y))
        msg.add_arg(float(z))
        builder.add_content(msg.build())
    bundle = builder.build()
    try:
        osc_client._sock.sendto(bundle.dgram, (OSC_IP, OSC_PORT))
    except OSError:
        # 소켓 오류 시 재생성
        osc_client = SimpleUDPClient(OSC_IP, OSC_PORT)
        osc_client._sock.sendto(bundle.dgram, (OSC_IP, OSC_PORT))


# ══════════════════════════════════════════
#  메인 루프
# ══════════════════════════════════════════
print("[시작] 'q' 종료 | 'k' 칼만 ON/OFF | '[' ']' Q값 조절 | 'n' Nano | 'm' Medium | 'g' GPU/CPU")

frame_count = 0
rs_retry_count = 0
MAX_RS_RETRIES = 5

try:
    while True:
        # ── RealSense 프레임 수신 (USB 끊김 대응) ──
        try:
            frames  = pipeline.wait_for_frames(timeout_ms=5000)
            rs_retry_count = 0
        except RuntimeError as e:
            rs_retry_count += 1
            print(f"[RealSense] 프레임 수신 실패 ({rs_retry_count}/{MAX_RS_RETRIES}): {e}")
            if rs_retry_count >= MAX_RS_RETRIES:
                print("[RealSense] 카메라 재연결 시도...")
                try:
                    pipeline.stop()
                    time.sleep(2)
                    profile = pipeline.start(config)
                    rs_retry_count = 0
                    print("[RealSense] 재연결 성공")
                except Exception as re_err:
                    print(f"[RealSense] 재연결 실패: {re_err}")
                    time.sleep(3)
            continue

        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        h, w = color_image.shape[:2]

        # ── YOLO26 추론 + 트래킹 ──
        results = model.track(
            color_image,
            persist=True,
            conf=CONF_THRESHOLD,
            verbose=False,
        )

        # ── 깊이 JET 컬러맵 ──
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )

        detected_count = 0
        result = results[0]

        if result.keypoints is not None and len(result.keypoints) > 0:
            keypoints_data = result.keypoints.data  # (N, 17, 3) — x, y, conf
            # 트래킹 ID 가져오기
            if result.boxes.id is not None:
                track_ids = result.boxes.id.int().cpu().tolist()
            else:
                track_ids = list(range(len(keypoints_data)))

            detected_count = len(keypoints_data)

            for person_idx, (kps, track_id) in enumerate(
                zip(keypoints_data, track_ids)
            ):
                color = PERSON_COLORS[track_id % len(PERSON_COLORS)]
                kps_np = kps.cpu().numpy()  # (17, 3)

                # 칼만 필터 딕셔너리 초기화
                if track_id not in kalman_filters:
                    kalman_filters[track_id] = {}
                if track_id not in kalman_filters_2d:
                    kalman_filters_2d[track_id] = {}

                # ── 2D 픽셀 좌표 칼만 필터 적용 ──
                filtered_px = []
                kf2d = kalman_filters_2d[track_id]
                for idx in range(17):
                    raw_x, raw_y, conf = kps_np[idx]
                    if kalman_enabled and conf > 0.3:
                        if idx not in kf2d:
                            kf2d[idx] = (_KF1D(KALMAN_Q, KALMAN_R),
                                         _KF1D(KALMAN_Q, KALMAN_R))
                        fx, fy = kf2d[idx]
                        raw_x = fx.update(raw_x)
                        raw_y = fy.update(raw_y)
                    filtered_px.append((int(raw_x), int(raw_y), conf))

                # ── 스켈레톤 연결선 ──
                for (i, j) in SKELETON:
                    if filtered_px[i][2] > 0.3 and filtered_px[j][2] > 0.3:
                        pt1 = (filtered_px[i][0], filtered_px[i][1])
                        pt2 = (filtered_px[j][0], filtered_px[j][1])
                        cv2.line(color_image, pt1, pt2, color, 2)

                # ── 랜드마크 점 ──
                for idx, (px, py, conf) in enumerate(filtered_px):
                    if conf > 0.3:
                        cv2.circle(color_image, (px, py), 4, color, -1)

                # ── 전신 3D 좌표 계산 + OSC 송신 ──
                joint_data = {}
                y_offset = 25 + person_idx * 170

                for idx, name in enumerate(KEYPOINT_NAMES):
                    px, py, conf = filtered_px[idx]
                    if conf < 0.3:
                        continue

                    px_c = max(0, min(px, w - 1))
                    py_c = max(0, min(py, h - 1))

                    depth_val = depth_frame.get_distance(px_c, py_c)
                    point_3d  = rs.rs2_deproject_pixel_to_point(
                        color_intrinsics, [px_c, py_c], depth_val
                    )
                    x3, y3, z3 = point_3d

                    if depth_val > 0.0:
                        if kalman_enabled:
                            kf = kalman_filters[track_id]
                            if name not in kf:
                                kf[name] = JointKalmanFilter()
                            x3, y3, z3 = kf[name].update(x3, y3, z3)
                        joint_data[name] = (x3, y3, z3)

                    # 주요 관절 화면 표시
                    if name in DISPLAY_JOINTS:
                        cv2.circle(color_image, (px, py), 6, color, -1)
                        if name == "Nose":
                            cv2.putText(color_image,
                                        f"ID:{track_id}",
                                        (px + 8, py),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        text = f"ID:{track_id} {name}:({x3:+.2f},{z3:.2f}m)"
                        cv2.putText(color_image, text, (10, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
                        y_offset += 15

                # ── OSC 송신 ──
                if joint_data:
                    try:
                        send_osc_bundle(track_id, joint_data)
                    except Exception as e:
                        print(f"[OSC] ID:{track_id} 송신 오류: {e}")

        else:
            cv2.putText(color_image, "No Pose Detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # ── 사라진 ID 칼만 필터 정리 (메모리 누수 방지) ──
        current_track_ids = set()
        if result.boxes.id is not None:
            current_track_ids = set(result.boxes.id.int().cpu().tolist())
        cleanup_stale_filters(current_track_ids)

        # ── GPU 메모리 정리 (100프레임마다) ──
        frame_count += 1
        if frame_count % 100 == 0 and current_device == "cuda":
            torch.cuda.empty_cache()

        # ── 상태 표시 (하단) ──
        model_label = YOLO_MODELS[current_model_key]
        device_label = current_device.upper()
        cv2.putText(color_image,
                    f"{model_label} ({device_label}) | OSC -> {OSC_IP}:{OSC_PORT}  Persons:{detected_count}  [N]ano [M]edium [G]PU/CPU",
                    (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 200, 0), 1)

        # ── 칼만 필터 상태 (우상단) ──
        kalman_label = "Kalman: ON  [K]" if kalman_enabled else "Kalman: OFF [K]"
        kalman_color = (0, 255, 100) if kalman_enabled else (0, 80, 255)
        cv2.putText(color_image, kalman_label,
                    (w - 175, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, kalman_color, 2)

        # ── 디바이스 상태 (우상단 2번째 줄) ──
        dev_color = (0, 255, 100) if current_device == "cuda" else (0, 80, 255)
        cv2.putText(color_image, f"Device: {device_label} [G]",
                    (w - 175, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, dev_color, 1)

        # ── 칼만 Q 값 (우상단 3번째 줄) ──
        if kalman_enabled:
            q_color = (200, 200, 200)
            cv2.putText(color_image, f"Q: {KALMAN_Q:.4f} [/]",
                        (w - 175, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, q_color, 1)

        # ── 화면 표시 ──
        display = np.hstack((color_image, depth_colormap))
        cv2.imshow("D455 YOLO26 Pose OSC | 'q' quit  'k' kalman", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("k"):
            kalman_enabled = not kalman_enabled
            if not kalman_enabled:
                kalman_filters.clear()
                kalman_filters_2d.clear()
            print(f"[Kalman] {'ON' if kalman_enabled else 'OFF'}")
        elif key == ord("n") and current_model_key != "n":
            current_model_key = "n"
            model = load_model("n", current_device)
            kalman_filters.clear()
            kalman_filters_2d.clear()
            print(f"[YOLO26] Nano 모델로 전환 ({current_device.upper()})")
        elif key == ord("m") and current_model_key != "m":
            current_model_key = "m"
            model = load_model("m", current_device)
            kalman_filters.clear()
            kalman_filters_2d.clear()
            print(f"[YOLO26] Medium 모델로 전환 ({current_device.upper()})")
        elif key == ord("g") and gpu_available:
            current_device = "cpu" if current_device == "cuda" else "cuda"
            model = load_model(current_model_key, current_device)
            kalman_filters.clear()
            kalman_filters_2d.clear()
            print(f"[Device] {current_device.upper()} 로 전환")
        elif key == ord("]") and kalman_enabled:
            new_q = min(KALMAN_Q * KALMAN_Q_STEP, KALMAN_Q_MAX)
            update_all_kalman_q(new_q)
            print(f"[Kalman] Q 증가: {KALMAN_Q:.4f} (반응↑ 스무딩↓)")
        elif key == ord("[") and kalman_enabled:
            new_q = max(KALMAN_Q / KALMAN_Q_STEP, KALMAN_Q_MIN)
            update_all_kalman_q(new_q)
            print(f"[Kalman] Q 감소: {KALMAN_Q:.4f} (반응↓ 스무딩↑)")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[완료] 프로그램 종료")
