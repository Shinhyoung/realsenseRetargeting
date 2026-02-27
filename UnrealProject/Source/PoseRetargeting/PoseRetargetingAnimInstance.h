#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "OSCPoseReceiver.h"
#include "PoseRetargetingAnimInstance.generated.h"

/**
 * PoseRetargetingAnimInstance — 전신 12 뼈대
 *
 * [Anim Graph 설정]
 *  각 뼈마다 "Transform (Modify) Bone" 노드를 추가하고
 *  Rotation 핀에 아래 FRotator UPROPERTY 를 연결하세요.
 *  Rotation Mode: Replace Component  /  Transform Space: Component Space
 *
 *  뼈 이름 → UPROPERTY 매핑 (UE5 Manny 기본값):
 *    neck_01    → Neck_Rot
 *    spine_03   → Spine_Rot
 *    upperarm_l → UpperArm_L_Rot   upperarm_r → UpperArm_R_Rot
 *    lowerarm_l → LowerArm_L_Rot   lowerarm_r → LowerArm_R_Rot
 *    thigh_l    → UpperLeg_L_Rot   thigh_r    → UpperLeg_R_Rot
 *    calf_l     → LowerLeg_L_Rot   calf_r     → LowerLeg_R_Rot
 *    foot_l     → Foot_L_Rot       foot_r     → Foot_R_Rot
 */
UCLASS(BlueprintType, Blueprintable)
class POSERETARGETING_API UPoseRetargetingAnimInstance : public UAnimInstance
{
    GENERATED_BODY()

public:
    // ── OSCPoseReceiver 참조 ─────────────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose")
    AOSCPoseReceiver* OSCReceiver = nullptr;

    /**
     * 이 AnimInstance 가 추적할 사람 번호 (0-based)
     * 여러 아바타를 배치하고 각각 0, 1, 2 로 설정하면
     * 각 아바타가 다른 사람의 포즈를 따라감
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose",
              meta = (ClampMin = "0", ClampMax = "2"))
    int32 PersonIndex = 0;

    // ── 뼈 이름 ─────────────────────────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName Neck_Bone      = TEXT("neck_01");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName Spine_Bone     = TEXT("spine_03");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName UpperArmL_Bone = TEXT("upperarm_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName UpperArmR_Bone = TEXT("upperarm_r");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName LowerArmL_Bone = TEXT("lowerarm_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName LowerArmR_Bone = TEXT("lowerarm_r");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName UpperLegL_Bone = TEXT("thigh_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName UpperLegR_Bone = TEXT("thigh_r");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName LowerLegL_Bone = TEXT("calf_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName LowerLegR_Bone = TEXT("calf_r");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName FootL_Bone     = TEXT("foot_l");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BoneNames")
    FName FootR_Bone     = TEXT("foot_r");

    // ── Anim Graph 에 노출되는 회전값 (12 뼈대) ─────────────
    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator Neck_Rot       = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator Spine_Rot      = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator UpperArm_L_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator UpperArm_R_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator LowerArm_L_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator LowerArm_R_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator UpperLeg_L_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator UpperLeg_R_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator LowerLeg_L_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator LowerLeg_R_Rot = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator Foot_L_Rot     = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Rotations")
    FRotator Foot_R_Rot     = FRotator::ZeroRotator;

    UPROPERTY(BlueprintReadOnly, Category = "Pose")
    bool bPoseValid = false;

    // ── 스무딩 ──────────────────────────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|Smoothing",
              meta = (ClampMin="0.0", ClampMax="30.0"))
    float SlerpSpeed = 10.0f;

protected:
    virtual void NativeInitializeAnimation() override;
    virtual void NativeUpdateAnimation(float DeltaSeconds) override;

private:
    FQuat Cur_Neck      = FQuat::Identity;
    FQuat Cur_Spine     = FQuat::Identity;
    FQuat Cur_UpperArmL = FQuat::Identity;
    FQuat Cur_UpperArmR = FQuat::Identity;
    FQuat Cur_LowerArmL = FQuat::Identity;
    FQuat Cur_LowerArmR = FQuat::Identity;
    FQuat Cur_UpperLegL = FQuat::Identity;
    FQuat Cur_UpperLegR = FQuat::Identity;
    FQuat Cur_LowerLegL = FQuat::Identity;
    FQuat Cur_LowerLegR = FQuat::Identity;
    FQuat Cur_FootL     = FQuat::Identity;
    FQuat Cur_FootR     = FQuat::Identity;

    static FQuat SmoothQuat(const FQuat& Cur, const FQuat& Target,
                             float Speed, float DeltaTime);
};
