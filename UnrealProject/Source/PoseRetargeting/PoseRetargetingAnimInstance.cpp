#include "PoseRetargetingAnimInstance.h"
#include "Kismet/GameplayStatics.h"

// ──────────────────────────────────────────────────────────────
//  초기화: Level 에서 AOSCPoseReceiver 자동 탐색
// ──────────────────────────────────────────────────────────────
void UPoseRetargetingAnimInstance::NativeInitializeAnimation()
{
    Super::NativeInitializeAnimation();

    if (OSCReceiver) return;

    UWorld* World = GetWorld();
    if (!World) return;

    TArray<AActor*> Found;
    UGameplayStatics::GetAllActorsOfClass(
        World, AOSCPoseReceiver::StaticClass(), Found);

    if (Found.Num() > 0)
    {
        OSCReceiver = Cast<AOSCPoseReceiver>(Found[0]);
        UE_LOG(LogTemp, Log,
            TEXT("[PoseRetargetingAnimInst] OSCPoseReceiver 자동 연결 완료"));
    }
    else
    {
        UE_LOG(LogTemp, Warning,
            TEXT("[PoseRetargetingAnimInst] OSCPoseReceiver 를 Level 에서 찾을 수 없습니다."));
    }
}

// ──────────────────────────────────────────────────────────────
//  매 프레임: 쿼터니언 SLERP → FRotator 출력 (전신 12 뼈)
// ──────────────────────────────────────────────────────────────
void UPoseRetargetingAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
    Super::NativeUpdateAnimation(DeltaSeconds);

    if (!OSCReceiver)
    {
        bPoseValid = false;
        return;
    }

    const FPoseBoneResult& R = OSCReceiver->BoneResult;
    bPoseValid = R.bHasData;
    if (!bPoseValid) return;

    // ── SLERP 스무딩 ──────────────────────────────────────────
    const float S  = SlerpSpeed;
    const float DT = DeltaSeconds;

    Cur_Neck      = SmoothQuat(Cur_Neck,      R.Neck,       S, DT);
    Cur_Spine     = SmoothQuat(Cur_Spine,     R.Spine,      S, DT);
    Cur_UpperArmL = SmoothQuat(Cur_UpperArmL, R.UpperArm_L, S, DT);
    Cur_UpperArmR = SmoothQuat(Cur_UpperArmR, R.UpperArm_R, S, DT);
    Cur_LowerArmL = SmoothQuat(Cur_LowerArmL, R.LowerArm_L, S, DT);
    Cur_LowerArmR = SmoothQuat(Cur_LowerArmR, R.LowerArm_R, S, DT);
    Cur_UpperLegL = SmoothQuat(Cur_UpperLegL, R.UpperLeg_L, S, DT);
    Cur_UpperLegR = SmoothQuat(Cur_UpperLegR, R.UpperLeg_R, S, DT);
    Cur_LowerLegL = SmoothQuat(Cur_LowerLegL, R.LowerLeg_L, S, DT);
    Cur_LowerLegR = SmoothQuat(Cur_LowerLegR, R.LowerLeg_R, S, DT);
    Cur_FootL     = SmoothQuat(Cur_FootL,     R.Foot_L,     S, DT);
    Cur_FootR     = SmoothQuat(Cur_FootR,     R.Foot_R,     S, DT);

    // ── FQuat → FRotator (Anim Graph Modify Transform 에 공급) ──
    Neck_Rot       = Cur_Neck.Rotator();
    Spine_Rot      = Cur_Spine.Rotator();
    UpperArm_L_Rot = Cur_UpperArmL.Rotator();
    UpperArm_R_Rot = Cur_UpperArmR.Rotator();
    LowerArm_L_Rot = Cur_LowerArmL.Rotator();
    LowerArm_R_Rot = Cur_LowerArmR.Rotator();
    UpperLeg_L_Rot = Cur_UpperLegL.Rotator();
    UpperLeg_R_Rot = Cur_UpperLegR.Rotator();
    LowerLeg_L_Rot = Cur_LowerLegL.Rotator();
    LowerLeg_R_Rot = Cur_LowerLegR.Rotator();
    Foot_L_Rot     = Cur_FootL.Rotator();
    Foot_R_Rot     = Cur_FootR.Rotator();
}

// ──────────────────────────────────────────────────────────────
//  SLERP 헬퍼 (Speed==0 이면 즉각 반영)
// ──────────────────────────────────────────────────────────────
FQuat UPoseRetargetingAnimInstance::SmoothQuat(
    const FQuat& Cur, const FQuat& Target, float Speed, float DeltaTime)
{
    if (Speed <= 0.f || DeltaTime <= 0.f) return Target;
    float Alpha = FMath::Clamp(Speed * DeltaTime, 0.f, 1.f);
    return FQuat::Slerp(Cur, Target, Alpha);
}
