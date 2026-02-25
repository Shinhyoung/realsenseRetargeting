#include "OSCPoseReceiver.h"
#include "OSCManager.h"
#include "OSCAddress.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"

AOSCPoseReceiver::AOSCPoseReceiver()
{
    PrimaryActorTick.bCanEverTick = true;
}

// ──────────────────────────────────────────────────────────────
//  BeginPlay: OSC 서버 생성
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::BeginPlay()
{
    Super::BeginPlay();

    OSCServer = UOSCManager::CreateOSCServer(
        ListenAddress, ListenPort,
        false, true,
        TEXT("PoseRetargetingServer"), this);

    if (OSCServer)
    {
        OSCServer->OnOscMessageReceived.AddDynamic(
            this, &AOSCPoseReceiver::HandleOSCMessage);
        UE_LOG(LogTemp, Log, TEXT("[OSCPoseReceiver] Listening %s:%d"),
               *ListenAddress, ListenPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[OSCPoseReceiver] OSC Server 생성 실패"));
    }
}

void AOSCPoseReceiver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (OSCServer) { OSCServer->Stop(); OSCServer = nullptr; }
    Super::EndPlay(EndPlayReason);
}

// ──────────────────────────────────────────────────────────────
//  Tick
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    TotalMessagesReceived = MessageCounter.Load();
    FlushAndComputeRotations();
    DrawDebugVisualization();
}

// ──────────────────────────────────────────────────────────────
//  OSC 메시지 수신  /pose/<name>  float x y z
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::HandleOSCMessage(
    const FOSCMessage& Message, const FString& IPAddress, int32 Port)
{
    const FOSCAddress& Addr = Message.GetAddress();
    FString FullPath = Addr.GetFullPath();

    TArray<FString> Parts;
    FullPath.ParseIntoArray(Parts, TEXT("/"), true);
    if (Parts.Num() < 2 || Parts[0] != TEXT("pose")) return;

    const FString& JointName = Parts[1];

    float RS_X = 0.f, RS_Y = 0.f, RS_Z = 0.f;
    if (!UOSCManager::GetFloat(Message, 0, RS_X)) return;
    if (!UOSCManager::GetFloat(Message, 1, RS_Y)) return;
    if (!UOSCManager::GetFloat(Message, 2, RS_Z)) return;

    StoreJoint(JointName, RS_X, RS_Y, RS_Z, RS_Z);
    ++MessageCounter;
}

// ──────────────────────────────────────────────────────────────
//  관절 기록 (임계 구역)
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::StoreJoint(
    const FString& Name, float RS_X, float RS_Y, float RS_Z, float DepthM)
{
    FJointData Data;
    Data.Position = ConvertCamToUnreal(RS_X, RS_Y, RS_Z);
    Data.bValid   = (DepthM > 0.01f);

    FScopeLock Lock(&JointMutex);
    PendingJoints.Add(Name, Data);
}

// ──────────────────────────────────────────────────────────────
//  내부 헬퍼: TMap 조회
// ──────────────────────────────────────────────────────────────
const FJointData* AOSCPoseReceiver::FindJoint(const FString& Name) const
{
    return Joints.Find(Name);
}

bool AOSCPoseReceiver::MidPoint(
    const FJointData* A, const FJointData* B, FVector& OutMid)
{
    if (!A || !B || !A->bValid || !B->bValid) return false;
    OutMid = (A->Position + B->Position) * 0.5f;
    return true;
}

// ──────────────────────────────────────────────────────────────
//  두 관절이 유효하면 쿼터니언 계산
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::TryComputeBone(
    FQuat& OutQuat,
    const FString& FromName, const FString& ToName,
    const FVector& BindDir)
{
    const FJointData* A = FindJoint(FromName);
    const FJointData* B = FindJoint(ToName);
    if (A && B && A->bValid && B->bValid)
        OutQuat = ComputeBoneQuat(A->Position, B->Position, BindDir);
}

// ──────────────────────────────────────────────────────────────
//  PendingJoints → Joints TMap 복사 + 전신 쿼터니언 계산
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::FlushAndComputeRotations()
{
    // ── 버퍼 스왑 ──────────────────────────────────────────────
    TMap<FString, FJointData> Snapshot;
    {
        FScopeLock Lock(&JointMutex);
        Snapshot = PendingJoints;
    }
    // 게임 스레드 TMap 갱신
    for (auto& Pair : Snapshot)
        Joints.Add(Pair.Key, Pair.Value);

    BoneResult.bHasData = false;

    // ── 목 (어깨 중심 → 코) ───────────────────────────────────
    {
        FVector ShoulderMid;
        const FJointData* LS = FindJoint(TEXT("L_Shoulder"));
        const FJointData* RS = FindJoint(TEXT("R_Shoulder"));
        const FJointData* Nose = FindJoint(TEXT("Nose"));
        if (MidPoint(LS, RS, ShoulderMid) && Nose && Nose->bValid)
        {
            FJointData ShMid; ShMid.Position = ShoulderMid; ShMid.bValid = true;
            FJointData NoseFake; NoseFake.Position = Nose->Position; NoseFake.bValid = true;
            BoneResult.Neck = ComputeBoneQuat(ShoulderMid, Nose->Position, BindDir_Neck);
            BoneResult.bHasData = true;
        }
    }

    // ── 척추 (골반 중심 → 어깨 중심) ─────────────────────────
    {
        FVector HipMid, ShoulderMid;
        const FJointData* LH = FindJoint(TEXT("L_Hip"));
        const FJointData* RH = FindJoint(TEXT("R_Hip"));
        const FJointData* LS = FindJoint(TEXT("L_Shoulder"));
        const FJointData* RS = FindJoint(TEXT("R_Shoulder"));
        if (MidPoint(LH, RH, HipMid) && MidPoint(LS, RS, ShoulderMid))
        {
            BoneResult.Spine = ComputeBoneQuat(HipMid, ShoulderMid, BindDir_Spine);
            BoneResult.bHasData = true;
        }
    }

    // ── 팔 ────────────────────────────────────────────────────
    TryComputeBone(BoneResult.UpperArm_L,
                   TEXT("L_Shoulder"), TEXT("L_Elbow"), BindDir_UpperArm_L);
    TryComputeBone(BoneResult.UpperArm_R,
                   TEXT("R_Shoulder"), TEXT("R_Elbow"), BindDir_UpperArm_R);
    TryComputeBone(BoneResult.LowerArm_L,
                   TEXT("L_Elbow"),    TEXT("L_Wrist"), BindDir_LowerArm_L);
    TryComputeBone(BoneResult.LowerArm_R,
                   TEXT("R_Elbow"),    TEXT("R_Wrist"), BindDir_LowerArm_R);

    // ── 다리 ──────────────────────────────────────────────────
    TryComputeBone(BoneResult.UpperLeg_L,
                   TEXT("L_Hip"),   TEXT("L_Knee"),  BindDir_UpperLeg_L);
    TryComputeBone(BoneResult.UpperLeg_R,
                   TEXT("R_Hip"),   TEXT("R_Knee"),  BindDir_UpperLeg_R);
    TryComputeBone(BoneResult.LowerLeg_L,
                   TEXT("L_Knee"),  TEXT("L_Ankle"), BindDir_LowerLeg_L);
    TryComputeBone(BoneResult.LowerLeg_R,
                   TEXT("R_Knee"),  TEXT("R_Ankle"), BindDir_LowerLeg_R);

    // ── 발 ────────────────────────────────────────────────────
    TryComputeBone(BoneResult.Foot_L,
                   TEXT("L_Ankle"), TEXT("L_Foot"),  BindDir_Foot_L);
    TryComputeBone(BoneResult.Foot_R,
                   TEXT("R_Ankle"), TEXT("R_Foot"),  BindDir_Foot_R);

    if (BoneResult.UpperArm_L != FQuat::Identity ||
        BoneResult.UpperLeg_L != FQuat::Identity)
        BoneResult.bHasData = true;
}

// ──────────────────────────────────────────────────────────────
//  좌표 변환: RealSense → Unreal
//    RS: X=우, Y=하, Z=깊이(m)
//    UE: X=전방, Y=우, Z=상(cm)
// ──────────────────────────────────────────────────────────────
FVector AOSCPoseReceiver::ConvertCamToUnreal(float RS_X, float RS_Y, float RS_Z)
{
    return FVector(RS_Z * 100.f, RS_X * 100.f, -RS_Y * 100.f);
}

// ──────────────────────────────────────────────────────────────
//  쿼터니언 계산: BindDir → normalize(To - From)
// ──────────────────────────────────────────────────────────────
FQuat AOSCPoseReceiver::ComputeBoneQuat(
    const FVector& FromPos, const FVector& ToPos, const FVector& BindDir)
{
    FVector CurrentDir = (ToPos - FromPos).GetSafeNormal();
    if (CurrentDir.IsNearlyZero()) return FQuat::Identity;

    FQuat Result = FQuat::FindBetweenVectors(BindDir.GetSafeNormal(), CurrentDir);
    Result.Normalize();
    return Result;
}

// ──────────────────────────────────────────────────────────────
//  관절 조회 (Blueprint용)
// ──────────────────────────────────────────────────────────────
bool AOSCPoseReceiver::GetJoint(
    const FString& Name, FVector& OutPosition, bool& OutValid) const
{
    if (const FJointData* J = Joints.Find(Name))
    {
        OutPosition = J->Position;
        OutValid    = J->bValid;
        return true;
    }
    OutPosition = FVector::ZeroVector;
    OutValid    = false;
    return false;
}

// ──────────────────────────────────────────────────────────────
//  디버그 시각화
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::DrawDebugVisualization()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // ── 화면 텍스트 ──────────────────────────────────────────
    if (bShowDebugText && GEngine)
    {
        int32 ValidCount = 0;
        for (auto& Pair : Joints) if (Pair.Value.bValid) ++ValidCount;

        GEngine->AddOnScreenDebugMessage(100, 0.05f,
            BoneResult.bHasData ? FColor::Green : FColor::Red,
            FString::Printf(TEXT("[OSCPoseReceiver] Port:%d | Msgs:%d | Joints:%d/27 | Bones:%s"),
                ListenPort, TotalMessagesReceived, ValidCount,
                BoneResult.bHasData ? TEXT("OK") : TEXT("WAIT")));

        // 주요 관절만 텍스트 표시
        static const TArray<FString> KeyJoints = {
            TEXT("Nose"),
            TEXT("L_Shoulder"), TEXT("R_Shoulder"),
            TEXT("L_Elbow"),    TEXT("R_Elbow"),
            TEXT("L_Wrist"),    TEXT("R_Wrist"),
            TEXT("L_Hip"),      TEXT("R_Hip"),
            TEXT("L_Knee"),     TEXT("R_Knee"),
            TEXT("L_Ankle"),    TEXT("R_Ankle"),
        };

        int32 Slot = 101;
        for (const FString& Key : KeyJoints)
        {
            if (const FJointData* J = Joints.Find(Key))
            {
                FColor C = J->bValid ? FColor::Cyan : FColor::Yellow;
                GEngine->AddOnScreenDebugMessage(Slot++, 0.05f, C,
                    FString::Printf(TEXT("  %s: (%.0f, %.0f, %.0f)cm [%s]"),
                        *Key,
                        J->Position.X, J->Position.Y, J->Position.Z,
                        J->bValid ? TEXT("OK") : TEXT("--")));
            }
        }
    }

    if (!bShowDebugSpheres) return;

    // ── 뷰포트 구체 (컬러 구분) ──────────────────────────────
    // 색상 그룹: 머리=흰색, 상체=파랑, 팔=청록, 하체=빨강, 발=주황
    auto DrawJoint = [&](const FString& Name, FColor Color)
    {
        if (const FJointData* J = Joints.Find(Name))
            if (J->bValid)
                DrawDebugSphere(World, J->Position,
                                DebugSphereRadius, 8, Color, false, -1.f, 0, 1.f);
    };

    auto DrawBone = [&](const FString& A, const FString& B, FColor Color)
    {
        const FJointData* JA = Joints.Find(A);
        const FJointData* JB = Joints.Find(B);
        if (JA && JB && JA->bValid && JB->bValid)
            DrawDebugLine(World, JA->Position, JB->Position,
                          Color, false, -1.f, 0, 2.f);
    };

    // 머리
    DrawJoint(TEXT("Nose"),      FColor::White);
    DrawJoint(TEXT("L_Eye"),     FColor::White);
    DrawJoint(TEXT("R_Eye"),     FColor::White);
    DrawJoint(TEXT("L_Ear"),     FColor::White);
    DrawJoint(TEXT("R_Ear"),     FColor::White);

    // 어깨
    DrawJoint(TEXT("L_Shoulder"),FColor::Blue);
    DrawJoint(TEXT("R_Shoulder"),FColor::Red);

    // 팔
    DrawJoint(TEXT("L_Elbow"),   FColor::Cyan);
    DrawJoint(TEXT("R_Elbow"),   FColor::Orange);
    DrawJoint(TEXT("L_Wrist"),   FColor::Cyan);
    DrawJoint(TEXT("R_Wrist"),   FColor::Orange);

    // 골반
    DrawJoint(TEXT("L_Hip"),     FColor::Blue);
    DrawJoint(TEXT("R_Hip"),     FColor::Red);

    // 다리
    DrawJoint(TEXT("L_Knee"),    FColor::Cyan);
    DrawJoint(TEXT("R_Knee"),    FColor::Orange);
    DrawJoint(TEXT("L_Ankle"),   FColor::Cyan);
    DrawJoint(TEXT("R_Ankle"),   FColor::Orange);
    DrawJoint(TEXT("L_Foot"),    FColor::Yellow);
    DrawJoint(TEXT("R_Foot"),    FColor::Yellow);

    // ── 연결선 ───────────────────────────────────────────────
    // 척추
    DrawBone(TEXT("L_Hip"),      TEXT("R_Hip"),      FColor::White);
    DrawBone(TEXT("L_Shoulder"), TEXT("R_Shoulder"), FColor::White);

    // 좌팔
    DrawBone(TEXT("L_Shoulder"), TEXT("L_Elbow"),    FColor::Blue);
    DrawBone(TEXT("L_Elbow"),    TEXT("L_Wrist"),    FColor::Cyan);

    // 우팔
    DrawBone(TEXT("R_Shoulder"), TEXT("R_Elbow"),    FColor::Red);
    DrawBone(TEXT("R_Elbow"),    TEXT("R_Wrist"),    FColor::Orange);

    // 좌다리
    DrawBone(TEXT("L_Hip"),      TEXT("L_Knee"),     FColor::Blue);
    DrawBone(TEXT("L_Knee"),     TEXT("L_Ankle"),    FColor::Cyan);
    DrawBone(TEXT("L_Ankle"),    TEXT("L_Foot"),     FColor::Yellow);

    // 우다리
    DrawBone(TEXT("R_Hip"),      TEXT("R_Knee"),     FColor::Red);
    DrawBone(TEXT("R_Knee"),     TEXT("R_Ankle"),    FColor::Orange);
    DrawBone(TEXT("R_Ankle"),    TEXT("R_Foot"),     FColor::Yellow);
}
