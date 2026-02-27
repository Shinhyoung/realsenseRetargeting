# RealSense Retargeting

Intel RealSense D455 카메라로 인체 관절 3D 좌표를 실시간 추출하고, OSC 프로토콜로 Unreal Engine 5 아바타에 리타게팅하는 프로젝트입니다.
**최대 3명 동시 추적**을 지원합니다.

```
D455 카메라 → MediaPipe Pose (최대 3명) → 3D 좌표 (칼만 필터)
    → OSC UDP /pose/<person_id>/<joint> → UE5 아바타 뼈대 제어
```

---

## 시스템 구성

```
realsenseRetargeting/
├── d455_pose3d_osc.py          # 메인: 다인원 포즈 추출 + OSC 송신
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
└── UnrealProject/              # UE5 C++ 프로젝트
    ├── PoseRetargeting.uproject
    ├── Config/
    └── Source/PoseRetargeting/
        ├── OSCPoseReceiver.h/.cpp          # OSC 서버 + 다인원 쿼터니언 계산
        ├── PoseRetargetingAnimInstance.h/.cpp  # SLERP 스무딩 + Anim Graph
        └── PoseRetargeting.Build.cs
```

---

## 설치 환경 (Python)

### 1. Conda 환경 생성

```bash
conda create -n d455_env python=3.10
conda activate d455_env
```

### 2. 패키지 설치

```bash
pip install pyrealsense2
pip install opencv-python
pip install mediapipe
pip install python-osc
```

### 3. MediaPipe 모델 다운로드

[pose_landmarker_lite.task](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker) 를 다운로드하여 아래 경로에 저장:

```
c:\0.shinhyoung\Project\realsenseTest\pose_landmarker_lite.task
```

> 경로를 변경하려면 `d455_pose3d_osc.py` 의 `MODEL_PATH` 수정

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

### Python (포즈 추출 + OSC 송신)

#### 방법 1 — 런처 파일 사용 (권장)

```bash
# 최초 1회 런처 파일 생성
python create_launcher.py

# 이후부터는 생성된 배치파일 더블클릭
C:\Users\<사용자명>\run_d455.bat
```

#### 방법 2 — 직접 실행

```bash
conda activate d455_env
python d455_pose3d_osc.py
```

**단축키**

| 키 | 동작 |
|----|------|
| `K` | 칼만 필터 ON / OFF 토글 |
| `Q` | 종료 |

**화면 표시**

| 색상 | 사람 |
|------|------|
| 노란색 | 0번 사람 |
| 초록색 | 1번 사람 |
| 파란색 | 2번 사람 |

### Unreal Engine (OSC 수신 + 아바타 제어)

1. `UnrealProject/PoseRetargeting.uproject` 더블클릭
2. 레벨에 `OSCPoseReceiver` 액터 배치 (MaxPersons: 3)
3. AnimBP를 `PoseRetargetingAnimInstance` 기반으로 생성
4. 아바타마다 `PersonIndex` 설정 (0 / 1 / 2)
5. Play → Python 실행 시 각 아바타에 해당 사람 포즈 적용

---

## OSC 통신 규격

| 항목 | 값 |
|------|----|
| 프로토콜 | UDP |
| 송신 IP | 127.0.0.1 |
| 포트 | 9000 |
| 메시지 주소 | `/pose/<person_id>/<joint_name>` |
| 인수 | `float x, float y, float z` (단위: m, 카메라 기준) |

**예시**
```
/pose/0/L_Shoulder  →  첫 번째 사람 왼쪽 어깨
/pose/1/R_Wrist     →  두 번째 사람 오른쪽 손목
```

### 전송 관절 목록 (27개)

| 부위 | 관절 |
|------|------|
| 머리 | Nose, L_Eye, R_Eye, L_Ear, R_Ear |
| 상체 | L_Shoulder, R_Shoulder, L_Elbow, R_Elbow, L_Wrist, R_Wrist |
| 손 | L_Thumb, R_Thumb, L_Index, R_Index, L_Pinky, R_Pinky |
| 하체 | L_Hip, R_Hip, L_Knee, R_Knee, L_Ankle, R_Ankle |
| 발 | L_Heel, R_Heel, L_Foot, R_Foot |

---

## 좌표계 변환

```
RealSense (m)          →    Unreal Engine (cm)
  X = 오른쪽               Y = X_RS × 100
  Y = 아래쪽               Z = -Y_RS × 100
  Z = 깊이 (전방)          X = Z_RS × 100
```

---

## UE5 아바타 제어 구조

```
OSCPoseReceiver (Actor)
  ├─ MaxPersons = 3
  ├─ PersonJoints[0..2]      → 사람별 27개 관절 위치 (cm)
  └─ PersonBoneResults[0..2] → 사람별 12개 뼈 쿼터니언
       Neck / Spine / UpperArm L,R / LowerArm L,R
       UpperLeg L,R / LowerLeg L,R / Foot L,R

PoseRetargetingAnimInstance (AnimBP)
  ├─ PersonIndex = 0 | 1 | 2   (어느 사람을 추적할지)
  └─ SLERP 스무딩 → FRotator 변환 → Anim Graph 공급
```

**다인원 아바타 설정**
- 아바타 3개를 레벨에 배치
- 각 AnimBP의 `PersonIndex`를 0, 1, 2 로 설정
- 각 아바타가 서로 다른 사람의 포즈를 독립적으로 추적

---

## 칼만 필터

깊이 카메라 노이즈와 MediaPipe 랜드마크 지터를 제거하기 위해
각 관절의 XYZ 축에 독립적으로 1D 칼만 필터를 적용합니다.
사람별로 별도의 필터 세트를 유지합니다.

```python
KALMAN_Q = 0.001   # 프로세스 노이즈 (클수록 빠른 추적)
KALMAN_R = 0.02    # 측정 노이즈   (클수록 강한 스무딩)
```

`K` 키로 실시간 ON/OFF 전환 가능

---

## 하드웨어 요구사항

- Intel RealSense D455 카메라 (USB 3.0 포트 연결)
- NVIDIA GPU (MediaPipe 가속, 선택사항)
