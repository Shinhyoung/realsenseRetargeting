# RealSense Retargeting

Intel RealSense D435/D455 카메라 또는 **일반 웹카메라**로 인체 관절 좌표를 실시간 추출하고, OSC 프로토콜로 Unreal Engine 5 / Unity 등에 전송하는 프로젝트입니다.
**다인원 동시 추적**을 지원하며, MediaPipe 와 YOLO26 Pose 두 가지 엔진을 제공합니다.

```
RealSense D435/D455 (Color / IR / Depth) → YOLO26 Pose → 3D 좌표 (칼만 필터)
    → OSC UDP /pose/<id>/<joint> → UE5 / Unity 아바타 제어

웹카메라 (Color only) → YOLO26 Pose → 2D 정규화 좌표 (z=0)
    → OSC UDP /pose/<id>/<joint> → UE5 / Unity 아바타 제어
```

---

## 시스템 구성

```
realsenseRetargeting/
├── d455_pose3d_osc.py          # MediaPipe: 다인원 포즈 추출 + OSC 송신 (27관절)
├── d455_yolo26_osc.py          # YOLO26:   다인원 포즈 추출 + OSC 송신 (17관절, GPU/웹캠지원)
├── botsort.yaml                # BoT-SORT + ReID 트래커 설정
├── d455_pose3d_osc_release.py  # Release 빌드용 (exe)
├── build_release.py            # PyInstaller Release 빌드 (MediaPipe 버전)
├── build_yolo26_release.py     # PyInstaller Release 빌드 (YOLO26 버전)
├── create_launcher.py          # run_d455.bat 런처 파일 생성기
├── d455_pose3d.py              # 기본: 포즈 3D 시각화 (OSC 없음)
├── d455_viewer.py              # 카메라 뷰어 (컬러 + 깊이)
├── d455_triple_view.py         # 3분할 뷰 (컬러 / 깊이 / IR)
├── d455_depth_pose3d.py        # 깊이 기반 포즈 시각화
├── d455_ir_pose.py             # IR 영상 포즈 추출
├── d455_ir_no_dot_pose.py      # IR 영상 포즈 (도트 프로젝터 OFF)
├── calibrate_color.py          # 컬러 카메라 캘리브레이션
├── calibrate_ir.py             # IR 카메라 캘리브레이션
├── generate_checkerboard.py    # 캘리브레이션용 체커보드 생성
├── recordings/                 # OSC 녹화 데이터 (txt)
├── UnityPlugin/                # Unity OSC 수신 플러그인
│   └── PoseOSCReceiver/
├── Release/                    # exe 빌드 결과물
│   └── D455_YOLO26_PoseOSC/
└── UnrealProject/              # UE5 C++ 프로젝트
    ├── PoseRetargeting.uproject
    ├── Config/
    └── Source/PoseRetargeting/
        ├── OSCPoseReceiver.h/.cpp          # OSC 서버 + 다인원 쿼터니언 계산
        ├── PoseRetargetingAnimInstance.h/.cpp  # SLERP 스무딩 + Anim Graph
        └── PoseRetargeting.Build.cs
```

---

## MediaPipe vs YOLO26 비교

| 항목 | d455_pose3d_osc.py (MediaPipe) | d455_yolo26_osc.py (YOLO26) |
|------|-------------------------------|----------------------------|
| 키포인트 | 27개 (손/발 포함) | 17개 (COCO 표준) |
| 최대 인원 | 3명 (설정값) | 무제한 (자동 검출) |
| ID 추적 | 인덱스 순서 (불안정) | BoT-SORT + ReID (가림 후 재식별) |
| 겹침 처리 | 약함 | 강함 |
| GPU 지원 | 미지원 | CUDA GPU 지원 (G키 전환) |
| 모델 전환 | 불가 | N/M 키로 Nano/Medium 실시간 전환 |
| IR 입력 | 미지원 | Color/IR 실시간 전환 (I키) |
| 웹카메라 | 미지원 | 자동 감지 + 2D 좌표 전송 |
| 카메라 분기 | D455 전용 | D435/D455 자동 판별 + 웹카메라 폴백 |
| 설정 창 | 없음 | 카메라 선택 + IP/Port/IR 설정 |
| 외부 모델 | pose_landmarker_lite.task 필요 | 자동 다운로드 |

---

## 설치 환경 (Python)

### 1. Conda 환경 생성

```bash
conda create -n d455_env python=3.10
conda activate d455_env
```

### 2. 공통 패키지 설치

```bash
pip install pyrealsense2
pip install opencv-python
pip install python-osc
```

### 3-A. MediaPipe 버전 추가 패키지

```bash
pip install mediapipe
```

[pose_landmarker_lite.task](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker) 다운로드 후 경로 설정 (`MODEL_PATH`)

### 3-B. YOLO26 버전 추가 패키지

```bash
pip install ultralytics
```

### 4. GPU 사용 (YOLO26, 선택사항)

```bash
# CUDA 12.8 기준 (NVIDIA GPU 필요)
pip install --force-reinstall torch torchvision --index-url https://download.pytorch.org/whl/cu128
```

---

## 설치 환경 (Unreal Engine)

| 항목 | 버전 |
|------|------|
| Unreal Engine | 5.7 |
| 빌드 도구 | Visual Studio 2022 |
| OSC 플러그인 | 엔진 내장 (활성화 필요) |

### UE5 빌드

```bash
"C:/Program Files/Epic Games/UE_5.7/Engine/Build/BatchFiles/Build.bat" ^
  PoseRetargetingEditor Development Win64 ^
  -Project="UnrealProject/PoseRetargeting.uproject" ^
  -WaitMutex
```

---

## 실행 방법

### Python — YOLO26 버전

```bash
conda activate d455_env
python d455_yolo26_osc.py
```

실행 시 **설정 창**(Motion Retargeting)이 표시됩니다:

| 설정 항목 | 기본값 | 설명 |
|----------|--------|------|
| 카메라 | 자동 감지 | RealSense 또는 웹카메라 선택 |
| OSC IP | 127.0.0.1 | OSC 수신측 IP |
| OSC Port | 9000 | OSC 수신측 포트 |
| 입력 영상 | Color | Color 또는 IR 선택 (RealSense만) |
| 도트 프로젝터 OFF | 해제 | IR 모드 시 도트 패턴 제거 (RealSense만) |

> 웹카메라 선택 시 IR/Depth 관련 옵션은 자동으로 비활성화됩니다.

#### 단축키

| 키 | 동작 | 웹카메라 |
|----|------|----------|
| `K` | 칼만 필터 ON/OFF | O |
| `[` | 칼만 Q 값 감소 (스무딩 강화, ÷2) | O |
| `]` | 칼만 Q 값 증가 (반응 빠름, ×2) | O |
| `N` | Nano 모델 전환 (빠름) | O |
| `M` | Medium 모델 전환 (정확) | O |
| `G` | GPU / CPU 전환 | O |
| `R` | OSC 데이터 녹화 시작/중지 | O |
| `Q` | 종료 | O |
| `I` | Color / IR 입력 전환 | X (비활성) |
| `+` | IR 노출 증가 (+500μs) | X (비활성) |
| `-` | IR 노출 감소 (-500μs) | X (비활성) |
| `A` | IR 자동/수동 노출 전환 | X (비활성) |

### Python — MediaPipe 버전

```bash
conda activate d455_env
python d455_pose3d_osc.py
```

| 키 | 동작 |
|----|------|
| `K` | 칼만 필터 ON/OFF |
| `Q` | 종료 |

### Python — Release (exe)

```bash
# YOLO26 버전 빌드
python build_yolo26_release.py

# 실행 (conda 환경 불필요)
Release/D455_YOLO26_PoseOSC/D455_YOLO26_PoseOSC.exe
```

| 항목 | 내용 |
|------|------|
| 빌드 도구 | PyInstaller (onefile) |
| 포함 파일 | exe, YOLO 모델(.pt), CUDA DLL, torch, pyrealsense2 등 |
| GPU 지원 | CUDA DLL 포함 (RTX GPU 자동 사용) |
| 모델 | yolo26n-pose.pt (Nano), yolo26m-pose.pt (Medium) |

> MediaPipe 버전 빌드: `python build_release.py` → `Release/D455_PoseOSC/D455_PoseOSC.exe`

### Unreal Engine (OSC 수신 + 아바타 제어)

1. `UnrealProject/PoseRetargeting.uproject` 더블클릭
2. 레벨에 `OSCPoseReceiver` 액터 배치
3. AnimBP를 `PoseRetargetingAnimInstance` 기반으로 생성
4. 아바타마다 `PersonIndex` 설정 (0 / 1 / 2 ...)
5. Play → Python 실행 시 각 아바타에 해당 사람 포즈 적용

---

## OSC 통신 규격

| 항목 | 값 |
|------|----|
| 프로토콜 | UDP |
| 송신 IP | 설정 가능 (기본 127.0.0.1) |
| 포트 | 설정 가능 (기본 9000) |
| 메시지 주소 | `/pose/<id>/<joint_name>` |
| 인수 | `float x, float y, float z` (RealSense: m 단위 3D, 웹카메라: 0~1 정규화 2D + z=0) |
| 전송 방식 | OSC Bundle (한 사람의 관절을 1개 Bundle로 묶어 전송) |

> OSC는 표준 프로토콜이므로 UE5, Unity, TouchDesigner, Max/MSP 등 어떤 수신측에서도 동일하게 사용 가능합니다.

**예시**
```
/pose/0/L_Shoulder  →  첫 번째 사람 왼쪽 어깨
/pose/1/R_Wrist     →  두 번째 사람 오른쪽 손목
```

### 전송 관절 목록

**MediaPipe (27개)**

| 부위 | 관절 |
|------|------|
| 머리 | Nose, L_Eye, R_Eye, L_Ear, R_Ear |
| 상체 | L_Shoulder, R_Shoulder, L_Elbow, R_Elbow, L_Wrist, R_Wrist |
| 손 | L_Thumb, R_Thumb, L_Index, R_Index, L_Pinky, R_Pinky |
| 하체 | L_Hip, R_Hip, L_Knee, R_Knee, L_Ankle, R_Ankle |
| 발 | L_Heel, R_Heel, L_Foot, R_Foot |

**YOLO26 (17개, COCO 표준)**

| 부위 | 관절 |
|------|------|
| 머리 | Nose, L_Eye, R_Eye, L_Ear, R_Ear |
| 상체 | L_Shoulder, R_Shoulder, L_Elbow, R_Elbow, L_Wrist, R_Wrist |
| 하체 | L_Hip, R_Hip, L_Knee, R_Knee, L_Ankle, R_Ankle |

---

## 좌표계 변환

```
RealSense (m)          →    Unreal Engine (cm)
  X = 오른쪽               Y = X_RS × 100
  Y = 아래쪽               Z = -Y_RS × 100
  Z = 깊이 (전방)          X = Z_RS × 100
```

```
RealSense (m)          →    Unity (m)
  X = 오른쪽               X =  X_RS
  Y = 아래쪽               Y = -Y_RS
  Z = 깊이 (전방)          Z =  Z_RS
```

---

## 칼만 필터

깊이 카메라 노이즈와 랜드마크 지터를 제거하기 위해
각 관절의 XYZ 축에 독립적으로 1D 칼만 필터를 적용합니다.
사람별로 별도의 필터 세트를 유지하며, 2D 화면 표시와 3D OSC 전송 모두에 적용됩니다.

```python
KALMAN_Q = 0.01    # 프로세스 노이즈 (클수록 빠른 추적)
KALMAN_R = 0.02    # 측정 노이즈   (클수록 강한 스무딩)
```

**실시간 조절 (YOLO26 버전)**

| 키 | 동작 | Q 값 범위 |
|----|------|----------|
| `K` | 칼만 필터 ON/OFF | - |
| `[` | Q 값 ÷2 (스무딩 강화) | 최소 0.001 |
| `]` | Q 값 ×2 (반응 빠름) | 최대 0.5 |

```
0.001 ← [스무딩 강] ── 0.01(기본) ── [반응 빠름] → 0.5
```

> 화면 우상단에 현재 Q 값이 실시간 표시됩니다

---

## 웹카메라 지원

RealSense 카메라가 없는 환경에서도 **일반 웹카메라**로 동작합니다.

| 항목 | RealSense | 웹카메라 |
|------|-----------|----------|
| 좌표 | 3D (미터 단위) | 2D 정규화 (x/w, y/h), z=0 |
| 깊이 영상 | JET 컬러맵 표시 | 없음 (Color만 표시) |
| IR 입력 | Color/IR 전환 가능 | 사용 불가 |
| IR 단축키 | I, +, -, A | 비활성 |
| D435 자동 감지 | IR 이미터 자동 ON | - |

> 시작 시 연결된 카메라를 자동 탐색하여 설정 창에 표시합니다.
> RealSense와 웹카메라가 동시에 연결된 경우 드롭다운에서 선택할 수 있습니다.

---

## IR 영상 입력

YOLO26 버전은 Color 영상 외에 **IR(적외선) 영상**을 입력으로 사용할 수 있습니다.
어두운 환경에서도 포즈 검출이 가능합니다.

| 항목 | 설명 |
|------|------|
| 전환 방식 | 설정 창에서 선택 또는 실행 중 `I` 키 |
| 도트 프로젝터 | 설정 창에서 OFF 가능 (깨끗한 IR 영상) |
| 노출 조절 | `+`/`-` 키로 500μs 단위 증감 |
| 자동 노출 | `A` 키로 Auto/Manual 전환 |
| 초기 노출 | Auto 모드로 시작, +/- 시 현재 Auto 값 기준으로 Manual 전환 |

---

## 화면 표시 (HUD)

| 위치 | 표시 내용 |
|------|----------|
| 좌측 상단 | 검출된 사람별 관절 3D 좌표 (ID, 관절명, X/Z값) |
| 좌측 하단 | FPS, YOLO 추론 시간 (ms) |
| 좌측 최하단 | 모델, 디바이스, 입력 소스, OSC 상태, 검출 인원 |
| 우측 상단 | 칼만 필터 ON/OFF, 디바이스, Q 값, 입력 소스 |
| 우측 상단 (녹화) | REC 상태, 경과 시간, 프레임 수 |
| 우측 상단 (IR) | IR 노출 값 (Auto/Manual) |
| 오른쪽 절반 | 깊이 영상 (JET 컬러맵) |

---

## ID 추적 (BoT-SORT + ReID)

YOLO26 버전은 **BoT-SORT + ReID**를 사용하여 사람의 고유 ID를 추적합니다.
장애물에 가려졌다 재등장해도 외형 특징(appearance feature)을 기반으로 기존 ID를 복원합니다.

| 항목 | 값 |
|------|----|
| 트래커 | BoT-SORT (botsort.yaml) |
| ReID 모델 | yolo26n-cls.pt (자동 다운로드) |
| 트랙 유지 시간 | 120프레임 (~4초 @ 30fps) |
| 외형 매칭 임계값 | 0.25 (낮을수록 재매칭 민감) |
| 모션 보상 | Sparse Optical Flow |

> `botsort.yaml`의 `track_buffer`, `appearance_thresh` 값을 조정하여 환경에 맞게 튜닝할 수 있습니다.

---

## OSC 데이터 녹화

실행 중 `R` 키를 눌러 OSC 데이터를 텍스트 파일로 녹화할 수 있습니다.

| 항목 | 설명 |
|------|------|
| 저장 위치 | `recordings/osc_stream_YYYYMMDD_HHMMSS.txt` |
| 형식 | `timestamp(s) \t /pose/id/joint \t x \t y \t z` |
| 좌표 단위 | 미터 (카메라 기준) |
| 시작/중지 | `R` 키 토글 |

---

## 장시간 구동 안정성

| 기능 | 설명 |
|------|------|
| 칼만 필터 메모리 관리 | 5초간 미검출 ID의 필터 자동 삭제 |
| GPU 메모리 관리 | 100프레임마다 CUDA 캐시 정리 |
| RealSense USB 끊김 대응 | 5회 실패 시 카메라 자동 재연결 (intrinsics 갱신) |
| 재연결 횟수 제한 | 최대 10회 재연결 실패 시 안전 종료 |
| OSC 소켓 복구 | 소켓 오류 시 자동 재생성 |

---

## 하드웨어 요구사항

| 항목 | 필수/선택 | 설명 |
|------|----------|------|
| Intel RealSense D435/D455 | 선택 | 3D 좌표 추출 (USB 3.0), D435 자동 감지 |
| 웹카메라 | 선택 | RealSense 없을 시 2D 좌표 추출 (USB) |
| NVIDIA GPU | 선택 | YOLO26 CUDA 가속 (RTX 계열 권장) |

> RealSense 또는 웹카메라 중 하나는 반드시 필요합니다.
