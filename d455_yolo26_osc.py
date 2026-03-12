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
import sys
import os
import datetime
import tkinter as tk
from tkinter import ttk
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
#  시작 설정 창 (tkinter)
# ══════════════════════════════════════════
def show_settings_dialog():
    """시작 시 설정 창을 표시하고 사용자 입력을 반환"""
    settings = {"ok": False}

    root = tk.Tk()
    root.title("Motion Retargeting — 설정")
    root.resizable(False, False)

    # 창을 화면 중앙에 배치
    win_w, win_h = 400, 300
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    x = (screen_w - win_w) // 2
    y = (screen_h - win_h) // 2
    root.geometry(f"{win_w}x{win_h}+{x}+{y}")

    frame = ttk.Frame(root, padding=20)
    frame.pack(fill="both", expand=True)

    # 그리드 중앙 정렬: 양쪽 컬럼 동일 비중
    frame.columnconfigure(0, weight=1)
    frame.columnconfigure(1, weight=1)

    # 제목
    ttk.Label(frame, text="Motion Retargeting",
              font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 15))

    # OSC IP
    ttk.Label(frame, text="OSC IP:").grid(row=1, column=0, sticky="e", padx=(0, 10), pady=4)
    ip_var = tk.StringVar(value="127.0.0.1")
    ip_entry = ttk.Entry(frame, textvariable=ip_var, width=20)
    ip_entry.grid(row=1, column=1, sticky="w", pady=4)

    # OSC Port
    ttk.Label(frame, text="OSC Port:").grid(row=2, column=0, sticky="e", padx=(0, 10), pady=4)
    port_var = tk.StringVar(value="9000")
    port_entry = ttk.Entry(frame, textvariable=port_var, width=20)
    port_entry.grid(row=2, column=1, sticky="w", pady=4)

    # IR 모드
    ttk.Label(frame, text="입력 영상:").grid(row=3, column=0, sticky="e", padx=(0, 10), pady=4)
    ir_var = tk.BooleanVar(value=False)
    ir_frame = ttk.Frame(frame)
    ir_frame.grid(row=3, column=1, sticky="w", pady=4)
    ttk.Radiobutton(ir_frame, text="Color (기본)", variable=ir_var, value=False).pack(side="left")
    ttk.Radiobutton(ir_frame, text="IR", variable=ir_var, value=True).pack(side="left", padx=(10, 0))

    # IR 도트 프로젝터
    dot_var = tk.BooleanVar(value=False)
    dot_check = ttk.Checkbutton(frame, text="IR 도트 프로젝터 OFF (깨끗한 IR 영상)",
                                 variable=dot_var)
    dot_check.grid(row=4, column=0, columnspan=2, pady=(4, 0))

    # 안내 텍스트
    ttk.Label(frame, text="실행 중 'I' 키로 Color/IR 전환 가능",
              foreground="gray").grid(row=5, column=0, columnspan=2, pady=(8, 0))

    def on_start():
        try:
            port = int(port_var.get())
            if not (1 <= port <= 65535):
                raise ValueError
        except ValueError:
            ttk.Label(frame, text="포트는 1~65535 숫자", foreground="red").grid(
                row=7, column=0, columnspan=2)
            return
        settings["ip"] = ip_var.get().strip()
        settings["port"] = port
        settings["use_ir"] = ir_var.get()
        settings["dot_off"] = dot_var.get()
        settings["ok"] = True
        root.destroy()

    def on_cancel():
        root.destroy()

    btn_frame = ttk.Frame(frame)
    btn_frame.grid(row=6, column=0, columnspan=2, pady=(15, 0))
    ttk.Button(btn_frame, text="시작", command=on_start, width=12).pack(side="left", padx=5)
    ttk.Button(btn_frame, text="취소", command=on_cancel, width=12).pack(side="left", padx=5)

    root.bind("<Return>", lambda e: on_start())
    root.bind("<Escape>", lambda e: on_cancel())
    ip_entry.focus()

    root.mainloop()
    return settings


# ── 설정 창 표시 ──
user_settings = show_settings_dialog()
if not user_settings["ok"]:
    print("[취소] 프로그램 종료")
    sys.exit(0)

OSC_IP = user_settings["ip"]
OSC_PORT = user_settings["port"]
use_ir = user_settings["use_ir"]
dot_projector_off = user_settings["dot_off"]

print(f"[설정] OSC={OSC_IP}:{OSC_PORT}  입력={'IR' if use_ir else 'Color'}"
      f"{'  도트OFF' if dot_projector_off else ''}")

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
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)

align   = rs.align(rs.stream.color)
profile = pipeline.start(config)

# IR 도트 프로젝터 제어
if dot_projector_off:
    device = profile.get_device()
    for sensor in device.sensors:
        if sensor.supports(rs.option.emitter_enabled):
            sensor.set_option(rs.option.emitter_enabled, 0)
            print("[IR] 도트 프로젝터 OFF")

# IR 센서 참조 (노출 조절용)
ir_sensor = None
rs_device = profile.get_device()
for sensor in rs_device.sensors:
    if sensor.get_info(rs.camera_info.name) == "Stereo Module":
        ir_sensor = sensor
        break

# IR 노출 설정
IR_EXPOSURE_MIN = 1
IR_EXPOSURE_MAX = 165000
IR_EXPOSURE_STEP = 500  # 고정 증감량 (μs)
ir_auto_exposure = True
ir_exposure_val = 8500  # 기본값

if ir_sensor and ir_sensor.supports(rs.option.exposure):
    # 항상 Auto 노출로 시작
    ir_sensor.set_option(rs.option.enable_auto_exposure, 1)
    ir_auto_exposure = True
    ir_exposure_val = int(ir_sensor.get_option(rs.option.exposure))


def set_ir_exposure(value: float):
    """IR 센서 노출 수동 설정"""
    global ir_exposure_val, ir_auto_exposure
    if ir_sensor is None:
        return
    if ir_auto_exposure:
        # 현재 자동 노출 값을 기준으로 시작
        ir_exposure_val = int(ir_sensor.get_option(rs.option.exposure))
        ir_auto_exposure = False
        ir_sensor.set_option(rs.option.enable_auto_exposure, 0)
        # value를 현재 auto 값 기준으로 재계산
        if value > ir_exposure_val:
            value = ir_exposure_val + IR_EXPOSURE_STEP
        else:
            value = ir_exposure_val - IR_EXPOSURE_STEP
    clamped = max(IR_EXPOSURE_MIN, min(int(value), IR_EXPOSURE_MAX))
    ir_sensor.set_option(rs.option.exposure, clamped)
    # 실제 적용된 값 확인
    ir_exposure_val = int(ir_sensor.get_option(rs.option.exposure))


def toggle_ir_auto_exposure():
    """IR 자동/수동 노출 전환"""
    global ir_auto_exposure, ir_exposure_val
    if ir_sensor is None:
        return
    ir_auto_exposure = not ir_auto_exposure
    ir_sensor.set_option(rs.option.enable_auto_exposure, 1 if ir_auto_exposure else 0)
    if not ir_auto_exposure:
        ir_exposure_val = int(ir_sensor.get_option(rs.option.exposure))


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
#  OSC 데이터 녹화 (txt 파일 저장)
# ══════════════════════════════════════════
recording = False
record_file = None
record_start_time = 0.0
record_frame_count = 0


def start_recording():
    """녹화 시작 — 타임스탬프 파일명으로 txt 생성"""
    global recording, record_file, record_start_time, record_frame_count
    os.makedirs("recordings", exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filepath = f"recordings/osc_stream_{ts}.txt"
    record_file = open(filepath, "w", encoding="utf-8")
    record_file.write(f"# OSC Data Recording — {ts}\n")
    record_file.write(f"# Format: timestamp(s) address x y z\n")
    record_file.write(f"# Coordinate: RealSense camera origin (m)\n\n")
    record_start_time = time.time()
    record_frame_count = 0
    recording = True
    print(f"[REC] 녹화 시작: {filepath}")


def stop_recording():
    """녹화 종료 — 파일 닫기"""
    global recording, record_file
    if record_file:
        record_file.write(f"\n# Total frames: {record_frame_count}\n")
        record_file.close()
        print(f"[REC] 녹화 종료 ({record_frame_count} 프레임 저장)")
    record_file = None
    recording = False


def record_osc_data(track_id: int, joint_data: dict):
    """현재 프레임의 OSC 데이터를 txt에 기록"""
    global record_frame_count
    if not recording or record_file is None:
        return
    t = time.time() - record_start_time
    for name, (x, y, z) in joint_data.items():
        record_file.write(f"{t:.4f}\t/pose/{track_id}/{name}\t{x:.6f}\t{y:.6f}\t{z:.6f}\n")
    record_frame_count += 1


# ══════════════════════════════════════════
#  메인 루프
# ══════════════════════════════════════════
print("[시작] 'q' 종료 | 'k' 칼만 | '[' ']' Q값 | 'i' IR전환 | '+' '-' IR노출 | 'a' 자동노출 | 'n' Nano | 'm' Medium | 'g' GPU/CPU | 'r' 녹화")

frame_count = 0
rs_retry_count = 0
MAX_RS_RETRIES = 5
rs_reconnect_count = 0
MAX_RS_RECONNECTS = 10  # 최대 재연결 시도 횟수

# FPS 측정
fps_start_time = time.time()
fps_frame_count = 0
current_fps = 0.0
infer_time_ms = 0.0

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
                rs_reconnect_count += 1
                if rs_reconnect_count > MAX_RS_RECONNECTS:
                    print(f"[RealSense] 재연결 {MAX_RS_RECONNECTS}회 초과 — 프로그램 종료")
                    break
                print(f"[RealSense] 카메라 재연결 시도 ({rs_reconnect_count}/{MAX_RS_RECONNECTS})...")
                try:
                    pipeline.stop()
                    time.sleep(2)
                    profile = pipeline.start(config)
                    color_intrinsics = (
                        profile.get_stream(rs.stream.color)
                        .as_video_stream_profile()
                        .get_intrinsics()
                    )
                    rs_retry_count = 0
                    rs_reconnect_count = 0  # 성공 시 카운터 리셋
                    print("[RealSense] 재연결 성공 (intrinsics 갱신)")
                except Exception as re_err:
                    print(f"[RealSense] 재연결 실패: {re_err}")
                    time.sleep(3)
            continue

        aligned = align.process(frames)

        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        ir_frame    = frames.get_infrared_frame(1)
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        h, w = color_image.shape[:2]

        # ── IR 영상 처리 ──
        if ir_frame:
            ir_raw = np.asanyarray(ir_frame.get_data())
            # IR → 3채널 BGR (YOLO 입력 + 화면 표시용)
            ir_normalized = cv2.normalize(ir_raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            ir_bgr = cv2.cvtColor(ir_normalized, cv2.COLOR_GRAY2BGR)
            # IR 해상도를 컬러에 맞춤
            if ir_bgr.shape[:2] != (h, w):
                ir_bgr = cv2.resize(ir_bgr, (w, h))
        else:
            ir_bgr = None

        # ── YOLO 입력 영상 선택 ──
        if use_ir and ir_bgr is not None:
            yolo_input = ir_bgr
            display_image = ir_bgr.copy()
        else:
            yolo_input = color_image
            display_image = color_image

        # ── YOLO26 추론 + 트래킹 ──
        t_infer_start = time.time()
        results = model.track(
            yolo_input,
            persist=True,
            conf=CONF_THRESHOLD,
            tracker="botsort.yaml",
            verbose=False,
        )
        infer_time_ms = (time.time() - t_infer_start) * 1000

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
                        cv2.line(display_image, pt1, pt2, color, 2)

                # ── 랜드마크 점 ──
                for idx, (px, py, conf) in enumerate(filtered_px):
                    if conf > 0.3:
                        cv2.circle(display_image, (px, py), 4, color, -1)

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
                        cv2.circle(display_image, (px, py), 6, color, -1)
                        if name == "Nose":
                            cv2.putText(display_image,
                                        f"ID:{track_id}",
                                        (px + 8, py),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        text = f"ID:{track_id} {name}:({x3:+.2f},{z3:.2f}m)"
                        cv2.putText(display_image, text, (10, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
                        y_offset += 15

                # ── OSC 송신 ──
                if joint_data:
                    try:
                        send_osc_bundle(track_id, joint_data)
                        record_osc_data(track_id, joint_data)
                    except Exception as e:
                        print(f"[OSC] ID:{track_id} 송신 오류: {e}")

        else:
            cv2.putText(display_image, "No Pose Detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # ── 사라진 ID 칼만 필터 정리 (메모리 누수 방지) ──
        current_track_ids = set()
        if result.boxes.id is not None:
            current_track_ids = set(result.boxes.id.int().cpu().tolist())
        cleanup_stale_filters(current_track_ids)

        # ── FPS 측정 ──
        frame_count += 1
        fps_frame_count += 1
        elapsed = time.time() - fps_start_time
        if elapsed >= 1.0:
            current_fps = fps_frame_count / elapsed
            fps_frame_count = 0
            fps_start_time = time.time()

        # ── GPU 메모리 정리 (100프레임마다) ──
        if frame_count % 100 == 0 and current_device == "cuda":
            torch.cuda.empty_cache()

        # ── 상태 표시 (하단) ──
        model_label = YOLO_MODELS[current_model_key]
        device_label = current_device.upper()
        input_label = "IR" if use_ir else "Color"
        cv2.putText(display_image,
                    f"{model_label} ({device_label}) | {input_label} | OSC -> {OSC_IP}:{OSC_PORT}  Persons:{detected_count}  [N]ano [M]edium [G]PU [I]R",
                    (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 200, 0), 1)

        # ── 칼만 필터 상태 (우상단) ──
        kalman_label = "Kalman: ON  [K]" if kalman_enabled else "Kalman: OFF [K]"
        kalman_color = (0, 255, 100) if kalman_enabled else (0, 80, 255)
        cv2.putText(display_image, kalman_label,
                    (w - 175, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, kalman_color, 2)

        # ── 디바이스 상태 (우상단 2번째 줄) ──
        dev_color = (0, 255, 100) if current_device == "cuda" else (0, 80, 255)
        cv2.putText(display_image, f"Device: {device_label} [G]",
                    (w - 175, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, dev_color, 1)

        # ── 칼만 Q 값 (우상단 3번째 줄) ──
        if kalman_enabled:
            q_color = (200, 200, 200)
            cv2.putText(display_image, f"Q: {KALMAN_Q:.4f} [/]",
                        (w - 175, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, q_color, 1)

        # ── 입력 소스 표시 (우상단 4번째 줄) ──
        ir_color = (0, 200, 255) if use_ir else (200, 200, 200)
        cv2.putText(display_image, f"Input: {input_label} [I]",
                    (w - 175, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, ir_color, 1)

        # ── FPS + 추론 시간 표시 (좌상단) ──
        fps_color = (0, 255, 0) if current_fps >= 25 else (0, 200, 255) if current_fps >= 15 else (0, 0, 255)
        cv2.putText(display_image, f"FPS: {current_fps:.1f}  Infer: {infer_time_ms:.1f}ms",
                    (10, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, fps_color, 1)

        # ── 녹화 상태 표시 (우상단 5번째 줄) ──
        if recording:
            rec_elapsed = time.time() - record_start_time
            rec_text = f"REC {rec_elapsed:.1f}s  F:{record_frame_count} [R]"
            cv2.putText(display_image, rec_text,
                        (w - 220, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        else:
            cv2.putText(display_image, "REC: OFF [R]",
                        (w - 175, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

        # ── IR 노출 표시 (우상단 6번째 줄, IR 모드일 때만) ──
        if use_ir and ir_sensor is not None:
            if ir_auto_exposure:
                exp_text = "Exp: Auto [A]  [+/-]"
                exp_color = (0, 255, 100)
            else:
                exp_text = f"Exp: {ir_exposure_val} [A]  [+/-]"
                exp_color = (0, 200, 255)
            cv2.putText(display_image, exp_text,
                        (w - 220, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, exp_color, 1)

        # ── 화면 표시 ──
        display = np.hstack((display_image, depth_colormap))
        cv2.imshow("D455 YOLO26 Pose OSC | 'q' quit  'k' kalman  'i' IR", display)

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
        elif key == ord("i"):
            use_ir = not use_ir
            kalman_filters.clear()
            kalman_filters_2d.clear()
            kalman_last_seen.clear()
            label = "IR" if use_ir else "Color"
            print(f"[Input] {label} 모드로 전환")
        elif key in (ord("="), ord("+")) and use_ir and ir_sensor is not None:
            set_ir_exposure(ir_exposure_val + IR_EXPOSURE_STEP)
            print(f"[IR] 노출 증가: {ir_exposure_val}")
        elif key == ord("-") and use_ir and ir_sensor is not None:
            set_ir_exposure(ir_exposure_val - IR_EXPOSURE_STEP)
            print(f"[IR] 노출 감소: {ir_exposure_val}")
        elif key == ord("a") and use_ir and ir_sensor is not None:
            toggle_ir_auto_exposure()
            label = "Auto" if ir_auto_exposure else f"Manual ({ir_exposure_val})"
            print(f"[IR] 노출: {label}")
        elif key == ord("r"):
            if recording:
                stop_recording()
            else:
                start_recording()

finally:
    if recording:
        stop_recording()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("[완료] 프로그램 종료")
