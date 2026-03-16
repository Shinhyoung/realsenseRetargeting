# PoseOSCReceiver — Unity 플러그인

RealSense D455 + YOLO26 Pose에서 OSC로 전송되는 17개 관절 3D 좌표를 수신하여
Unity Humanoid 아바타에 모션 리타게팅하는 플러그인입니다.

## 특징

- **외부 라이브러리 없음** — 자체 OSC 바이너리 파서 내장
- **UDP 멀티스레드 수신** — 메인 스레드 블로킹 없음
- **다중 인물 지원** — trackID 기반 분리
- **아바타 리타게팅** — Quaternion.FromToRotation 기반 본 회전
- **SLERP 스무딩** — 떨림 방지
- **디버그 시각화** — Gizmo + OnGUI로 관절 확인

## 파일 구조

```
PoseOSCReceiver/Runtime/
├── OscParser.cs            # OSC 1.0 바이너리 파서 (Bundle/Message)
├── PoseData.cs             # 관절 이름 상수 + PoseSnapshot 데이터 클래스
├── PoseOSCServer.cs        # UDP 수신 서버 (MonoBehaviour)
├── PoseRetargeting.cs      # 아바타 본 회전 적용 (Humanoid)
└── PoseDebugVisualizer.cs  # 디버그 Gizmo + GUI 시각화
```

## 설치 방법

1. `PoseOSCReceiver` 폴더 전체를 Unity 프로젝트의 `Assets/` 아래에 복사
2. 또는 `Assets/Plugins/PoseOSCReceiver/`에 배치

## 사용법

### 1단계: OSC 서버 설정

1. 빈 GameObject 생성 → `PoseOSCServer` 컴포넌트 추가
2. Inspector에서 `Port` = 9000 (Python 프로그램의 OSC 포트와 동일)

### 2단계: 아바타 리타게팅

1. Humanoid Avatar가 설정된 캐릭터 모델을 씬에 배치
2. 캐릭터에 `PoseRetargeting` 컴포넌트 추가
3. `OSC Server` 필드에 1단계의 PoseOSCServer 연결
4. `Track ID` = 0 (첫 번째 사람)
5. `Smoothing` 조절 (0=즉시반응, 0.5=부드러움, 0.9=매우 느림)

### 3단계: 디버그 시각화 (선택)

1. 빈 GameObject에 `PoseDebugVisualizer` 컴포넌트 추가
2. `OSC Server` 필드 연결
3. Scene 뷰에서 관절 Gizmo 확인, Game 뷰에서 좌표값 텍스트 확인

## 좌표 변환

```
RealSense (미터)          Unity (미터)
X = 오른쪽 (+)     →     X = 오른쪽 (+)   (동일)
Y = 아래쪽 (+)     →     Y = 위쪽 (+)     (반전: -Y)
Z = 전방 (+)       →     Z = 전방 (+)     (동일)
```

변환은 `PoseOSCServer.cs`에서 자동 처리됩니다.

## 리타게팅 관절 매핑

| 수신 관절 | 방향 계산 | Unity 본 |
|-----------|-----------|----------|
| L_Shoulder → L_Elbow | 왼쪽 상완 방향 | Left UpperArm |
| L_Elbow → L_Wrist | 왼쪽 하완 방향 | Left LowerArm |
| R_Shoulder → R_Elbow | 오른쪽 상완 방향 | Right UpperArm |
| R_Elbow → R_Wrist | 오른쪽 하완 방향 | Right LowerArm |
| L_Hip → L_Knee | 왼쪽 대퇴 방향 | Left UpperLeg |
| L_Knee → L_Ankle | 왼쪽 하퇴 방향 | Left LowerLeg |
| R_Hip → R_Knee | 오른쪽 대퇴 방향 | Right UpperLeg |
| R_Knee → R_Ankle | 오른쪽 하퇴 방향 | Right LowerLeg |
| Hip중심 → Shoulder중심 | 몸통 방향 | Spine |
| Shoulder중심 → Nose | 머리 방향 | Head |

## OSC 주소 형식

```
/pose/{trackID}/{관절명}  float x, float y, float z

예: /pose/0/L_Shoulder -0.18 -0.12 1.80
```

## 요구사항

- Unity 2021.3 이상
- .NET Standard 2.1 또는 .NET Framework
- Humanoid Avatar 설정된 캐릭터 모델
