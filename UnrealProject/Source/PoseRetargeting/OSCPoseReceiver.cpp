#include "OSCPoseReceiver.h"
#include "OSCManager.h"
#include "OSCAddress.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"

// 사람별 디버그 색상 (최대 3명)
static const FColor PersonDebugColors[3] =
{
    FColor::Cyan,    // 0번
    FColor::Green,   // 1번
    FColor::Orange,  // 2번
};

AOSCPoseReceiver::AOSCPoseReceiver()
{
    PrimaryActorTick.bCanEverTick = true;
}

// ──────────────────────────────────────────────────────────────
//  BeginPlay: 배열 초기화 + OSC 서버 생성
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::BeginPlay()
{
    Super::BeginPlay();

    // MaxPersons 크기로 배열 초기화
    PersonJoints.SetNum(MaxPersons);       // FPersonJointMap 배열
    PersonBoneResults.SetNum(MaxPersons);
    PendingPersonJoints.SetNum(MaxPersons);

    OSCServer = UOSCManager::CreateOSCServer(
        ListenAddress, ListenPort,
        false, true,
        TEXT("PoseRetargetingServer"), this);

    if (OSCServer)
    {
        OSCServer->OnOscMessageReceived.AddDynamic(
            this, &AOSCPoseReceiver::HandleOSCMessage);
        UE_LOG(LogTemp, Log, TEXT("[OSCPoseReceiver] Listening %s:%d | MaxPersons:%d"),
               *ListenAddress, ListenPort, MaxPersons);
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
//  OSC 메시지 수신  /pose/<person_id>/<joint_name>  float x y z
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::HandleOSCMessage(
    const FOSCMessage& Message, const FString& IPAddress, int32 Port)
{
    const FOSCAddress& Addr = Message.GetAddress();
    FString FullPath = Addr.GetFullPath();

    // 파싱: /pose/0/L_Shoulder → ["pose","0","L_Shoulder"]
    TArray<FString> Parts;
    FullPath.ParseIntoArray(Parts, TEXT("/"), true);
    if (Parts.Num() < 3 || Parts[0] != TEXT("pose")) return;

    int32 PersonId = FCString::Atoi(*Parts[1]);
    if (PersonId < 0 || PersonId >= MaxPersons) return;

    const FString& JointName = Parts[2];

    float RS_X = 0.f, RS_Y = 0.f, RS_Z = 0.f;
    if (!UOSCManager::GetFloat(Message, 0, RS_X)) return;
    if (!UOSCManager::GetFloat(Message, 1, RS_Y)) return;
    if (!UOSCManager::GetFloat(Message, 2, RS_Z)) return;

    FJointData Data;
    Data.Position = ConvertCamToUnreal(RS_X, RS_Y, RS_Z);
    Data.bValid   = (RS_Z > 0.01f);

    FScopeLock Lock(&JointMutex);
    PendingPersonJoints[PersonId].Add(JointName, Data);
    ++MessageCounter;
}

// ──────────────────────────────────────────────────────────────
//  헬퍼: TMap 에서 관절 안전 조회
// ──────────────────────────────────────────────────────────────
const FJointData* AOSCPoseReceiver::FindJointInMap(
    const TMap<FString, FJointData>& JointsMap, const FString& Name)
{
    return JointsMap.Find(Name);
}

bool AOSCPoseReceiver::MidPoint(
    const FJointData* A, const FJointData* B, FVector& OutMid)
{
    if (!A || !B || !A->bValid || !B->bValid) return false;
    OutMid = (A->Position + B->Position) * 0.5f;
    return true;
}

void AOSCPoseReceiver::TryComputeBoneInMap(
    FQuat& OutQuat,
    const TMap<FString, FJointData>& JointsMap,
    const FString& FromName, const FString& ToName,
    const FVector& BindDir)
{
    const FJointData* A = FindJointInMap(JointsMap, FromName);
    const FJointData* B = FindJointInMap(JointsMap, ToName);
    if (A && B && A->bValid && B->bValid)
        OutQuat = ComputeBoneQuat(A->Position, B->Position, BindDir);
}

// ──────────────────────────────────────────────────────────────
//  PendingPersonJoints → PersonJoints 복사 + 전신 쿼터니언 계산
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::FlushAndComputeRotations()
{
    TArray<TMap<FString, FJointData>> Snapshot;
    {
        FScopeLock Lock(&JointMutex);
        Snapshot = PendingPersonJoints;
    }

    for (int32 PID = 0; PID < MaxPersons; ++PID)
    {
        for (auto& Pair : Snapshot[PID])
            PersonJoints[PID].Joints.Add(Pair.Key, Pair.Value);

        ComputeBonesForPerson(PID);
    }
}

void AOSCPoseReceiver::ComputeBonesForPerson(int32 PersonId)
{
    if (!PersonJoints.IsValidIndex(PersonId)) return;

    const TMap<FString, FJointData>& J = PersonJoints[PersonId].Joints;
    FPoseBoneResult& Bones = PersonBoneResults[PersonId];
    Bones.bHasData = false;

    // ── 목 (어깨 중심 → 코) ──
    {
        const FJointData* LS   = FindJointInMap(J, TEXT("L_Shoulder"));
        const FJointData* RS   = FindJointInMap(J, TEXT("R_Shoulder"));
        const FJointData* Nose = FindJointInMap(J, TEXT("Nose"));
        FVector ShoulderMid;
        if (MidPoint(LS, RS, ShoulderMid) && Nose && Nose->bValid)
        {
            Bones.Neck = ComputeBoneQuat(ShoulderMid, Nose->Position, BindDir_Neck);
            Bones.bHasData = true;
        }
    }

    // ── 척추 (골반 중심 → 어깨 중심) ──
    {
        const FJointData* LH = FindJointInMap(J, TEXT("L_Hip"));
        const FJointData* RH = FindJointInMap(J, TEXT("R_Hip"));
        const FJointData* LS = FindJointInMap(J, TEXT("L_Shoulder"));
        const FJointData* RS = FindJointInMap(J, TEXT("R_Shoulder"));
        FVector HipMid, ShoulderMid;
        if (MidPoint(LH, RH, HipMid) && MidPoint(LS, RS, ShoulderMid))
        {
            Bones.Spine = ComputeBoneQuat(HipMid, ShoulderMid, BindDir_Spine);
            Bones.bHasData = true;
        }
    }

    // ── 팔 ──
    TryComputeBoneInMap(Bones.UpperArm_L, J, TEXT("L_Shoulder"), TEXT("L_Elbow"), BindDir_UpperArm_L);
    TryComputeBoneInMap(Bones.UpperArm_R, J, TEXT("R_Shoulder"), TEXT("R_Elbow"), BindDir_UpperArm_R);
    TryComputeBoneInMap(Bones.LowerArm_L, J, TEXT("L_Elbow"),    TEXT("L_Wrist"), BindDir_LowerArm_L);
    TryComputeBoneInMap(Bones.LowerArm_R, J, TEXT("R_Elbow"),    TEXT("R_Wrist"), BindDir_LowerArm_R);

    // ── 다리 ──
    TryComputeBoneInMap(Bones.UpperLeg_L, J, TEXT("L_Hip"),  TEXT("L_Knee"),  BindDir_UpperLeg_L);
    TryComputeBoneInMap(Bones.UpperLeg_R, J, TEXT("R_Hip"),  TEXT("R_Knee"),  BindDir_UpperLeg_R);
    TryComputeBoneInMap(Bones.LowerLeg_L, J, TEXT("L_Knee"), TEXT("L_Ankle"), BindDir_LowerLeg_L);
    TryComputeBoneInMap(Bones.LowerLeg_R, J, TEXT("R_Knee"), TEXT("R_Ankle"), BindDir_LowerLeg_R);

    // ── 발 ──
    TryComputeBoneInMap(Bones.Foot_L, J, TEXT("L_Ankle"), TEXT("L_Foot"), BindDir_Foot_L);
    TryComputeBoneInMap(Bones.Foot_R, J, TEXT("R_Ankle"), TEXT("R_Foot"), BindDir_Foot_R);

    if (Bones.UpperArm_L != FQuat::Identity || Bones.UpperLeg_L != FQuat::Identity)
        Bones.bHasData = true;
}

// ──────────────────────────────────────────────────────────────
//  좌표 변환: RealSense → Unreal
// ──────────────────────────────────────────────────────────────
FVector AOSCPoseReceiver::ConvertCamToUnreal(float RS_X, float RS_Y, float RS_Z)
{
    return FVector(RS_Z * 100.f, RS_X * 100.f, -RS_Y * 100.f);
}

// ──────────────────────────────────────────────────────────────
//  쿼터니언 계산
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
bool AOSCPoseReceiver::GetJointForPerson(
    int32 PersonId, const FString& Name,
    FVector& OutPosition, bool& OutValid) const
{
    if (!PersonJoints.IsValidIndex(PersonId))
    {
        OutPosition = FVector::ZeroVector; OutValid = false; return false;
    }
    if (const FJointData* J = PersonJoints[PersonId].Joints.Find(Name))
    {
        OutPosition = J->Position; OutValid = J->bValid; return true;
    }
    OutPosition = FVector::ZeroVector; OutValid = false; return false;
}

int32 AOSCPoseReceiver::GetActivePersonCount() const
{
    int32 Count = 0;
    for (int32 PID = 0; PID < PersonBoneResults.Num(); ++PID)
        if (PersonBoneResults[PID].bHasData) ++Count;
    return Count;
}

// ──────────────────────────────────────────────────────────────
//  디버그 시각화 (사람별 다른 색상)
// ──────────────────────────────────────────────────────────────
void AOSCPoseReceiver::DrawDebugVisualization()
{
    UWorld* World = GetWorld();
    if (!World) return;

    if (bShowDebugText && GEngine)
    {
        int32 ActivePersons = GetActivePersonCount();
        GEngine->AddOnScreenDebugMessage(100, 0.05f,
            ActivePersons > 0 ? FColor::Green : FColor::Red,
            FString::Printf(TEXT("[OSCPoseReceiver] Port:%d | Msgs:%d | Persons:%d/%d"),
                ListenPort, TotalMessagesReceived, ActivePersons, MaxPersons));

        static const TArray<FString> KeyJoints = {
            TEXT("Nose"), TEXT("L_Shoulder"), TEXT("R_Shoulder"),
            TEXT("L_Wrist"), TEXT("R_Wrist"),
            TEXT("L_Hip"), TEXT("R_Hip"),
            TEXT("L_Ankle"), TEXT("R_Ankle"),
        };

        int32 Slot = 101;
        for (int32 PID = 0; PID < MaxPersons; ++PID)
        {
            if (!PersonBoneResults.IsValidIndex(PID) || !PersonBoneResults[PID].bHasData)
                continue;

            FColor PC = PersonDebugColors[PID % 3];
            GEngine->AddOnScreenDebugMessage(Slot++, 0.05f, PC,
                FString::Printf(TEXT("── Person %d ──────────────"), PID));

            for (const FString& Key : KeyJoints)
            {
                if (const FJointData* J = PersonJoints[PID].Joints.Find(Key))
                {
                    FColor C = J->bValid ? PC : FColor::Yellow;
                    GEngine->AddOnScreenDebugMessage(Slot++, 0.05f, C,
                        FString::Printf(TEXT("  %s: (%.0f, %.0f, %.0f)cm"),
                            *Key, J->Position.X, J->Position.Y, J->Position.Z));
                }
            }
        }
    }

    if (!bShowDebugSpheres) return;

    static const TArray<FString> BoneFrom = {
        TEXT("L_Shoulder"), TEXT("L_Elbow"),
        TEXT("R_Shoulder"), TEXT("R_Elbow"),
        TEXT("L_Hip"),      TEXT("L_Knee"),  TEXT("L_Ankle"),
        TEXT("R_Hip"),      TEXT("R_Knee"),  TEXT("R_Ankle"),
        TEXT("L_Hip"),      TEXT("R_Hip"),
        TEXT("L_Shoulder"), TEXT("R_Shoulder"),
    };
    static const TArray<FString> BoneTo = {
        TEXT("L_Elbow"),    TEXT("L_Wrist"),
        TEXT("R_Elbow"),    TEXT("R_Wrist"),
        TEXT("L_Knee"),     TEXT("L_Ankle"), TEXT("L_Foot"),
        TEXT("R_Knee"),     TEXT("R_Ankle"), TEXT("R_Foot"),
        TEXT("R_Hip"),      TEXT("R_Hip"),   // hip line
        TEXT("R_Shoulder"), TEXT("R_Shoulder"), // shoulder line
    };

    for (int32 PID = 0; PID < MaxPersons; ++PID)
    {
        if (!PersonJoints.IsValidIndex(PID)) continue;

        const TMap<FString, FJointData>& J = PersonJoints[PID].Joints;
        FColor PC = PersonDebugColors[PID % 3];

        // 관절 구체
        for (auto& Pair : J)
        {
            if (Pair.Value.bValid)
                DrawDebugSphere(World, Pair.Value.Position,
                                DebugSphereRadius, 8, PC, false, -1.f, 0, 1.f);
        }

        // 뼈대 연결선
        for (int32 i = 0; i < BoneFrom.Num(); ++i)
        {
            const FJointData* A = FindJointInMap(J, BoneFrom[i]);
            const FJointData* B = FindJointInMap(J, BoneTo[i]);
            if (A && B && A->bValid && B->bValid)
                DrawDebugLine(World, A->Position, B->Position,
                              PC, false, -1.f, 0, 2.f);
        }
    }
}
