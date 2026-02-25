#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "OSCServer.h"
#include "OSCMessage.h"
#include "OSCPoseReceiver.generated.h"

// ──────────────────────────────────────────
//  단일 관절 위치 데이터
// ──────────────────────────────────────────
USTRUCT(BlueprintType)
struct FJointData
{
    GENERATED_BODY()

    /** Unreal 월드 좌표 (cm) */
    UPROPERTY(BlueprintReadOnly, Category = "Pose")
    FVector Position = FVector::ZeroVector;

    /** 유효한 깊이값이 있을 때만 true */
    UPROPERTY(BlueprintReadOnly, Category = "Pose")
    bool bValid = false;
};

// ──────────────────────────────────────────
//  전신 뼈대 쿼터니언 결과 (11 뼈대)
// ──────────────────────────────────────────
USTRUCT(BlueprintType)
struct FPoseBoneResult
{
    GENERATED_BODY()

    // ── 머리/목 ──────────────────────────
    /** 목  (어깨 중심 → 코) */
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Neck        = FQuat::Identity;

    // ── 척추 ─────────────────────────────
    /** 척추 (골반 중심 → 어깨 중심) */
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Spine       = FQuat::Identity;

    // ── 팔 ───────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperArm_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperArm_R  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerArm_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerArm_R  = FQuat::Identity;

    // ── 다리 ─────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperLeg_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperLeg_R  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerLeg_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerLeg_R  = FQuat::Identity;

    // ── 발 ───────────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Foot_L      = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Foot_R      = FQuat::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Pose") bool bHasData     = false;
};

// ──────────────────────────────────────────
//  OSC 수신 + 전신 쿼터니언 계산 Actor
// ──────────────────────────────────────────
UCLASS(BlueprintType, Blueprintable)
class POSERETARGETING_API AOSCPoseReceiver : public AActor
{
    GENERATED_BODY()

public:
    AOSCPoseReceiver();

    // ── OSC 설정 ──────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSC")
    FString ListenAddress = TEXT("0.0.0.0");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSC")
    int32 ListenPort = 9000;

    // ── 전신 관절 데이터 (TMap, 27개) ─────
    /**
     * 키: Python 에서 송신하는 관절 이름
     *   머리: Nose, L_Eye, R_Eye, L_Ear, R_Ear
     *   상체: L_Shoulder, R_Shoulder, L_Elbow, R_Elbow, L_Wrist, R_Wrist
     *   손:   L_Thumb, R_Thumb, L_Index, R_Index, L_Pinky, R_Pinky
     *   하체: L_Hip, R_Hip, L_Knee, R_Knee, L_Ankle, R_Ankle
     *   발:   L_Heel, R_Heel, L_Foot, R_Foot
     */
    UPROPERTY(BlueprintReadOnly, Category = "Pose|Joints")
    TMap<FString, FJointData> Joints;

    // ── 바인드 포즈 기준 방향 (UE5 Manny T-포즈 기준) ──
    // 캐릭터가 +X를 향할 때 컴포넌트 공간 방향
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Neck       = FVector(0.f,  0.f,  1.f);  // 위(+Z)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Spine      = FVector(0.f,  0.f,  1.f);  // 위(+Z)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperArm_L = FVector(0.f, -1.f,  0.f);  // 좌(-Y)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperArm_R = FVector(0.f,  1.f,  0.f);  // 우(+Y)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerArm_L = FVector(0.f, -1.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerArm_R = FVector(0.f,  1.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperLeg_L = FVector(0.f,  0.f, -1.f);  // 아래(-Z)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperLeg_R = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerLeg_L = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerLeg_R = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Foot_L     = FVector(1.f,  0.f,  0.f);  // 전방(+X)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Foot_R     = FVector(1.f,  0.f,  0.f);

    // ── 계산 결과 ─────────────────────────
    UPROPERTY(BlueprintReadOnly, Category = "Pose")
    FPoseBoneResult BoneResult;

    // ── 디버그 ────────────────────────────
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|Debug")
    bool bShowDebugText = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|Debug")
    bool bShowDebugSpheres = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|Debug")
    float DebugSphereRadius = 4.f;

    UPROPERTY(BlueprintReadOnly, Category = "Pose|Debug")
    int32 TotalMessagesReceived = 0;

    // ── Blueprint 유틸리티 ────────────────
    /** RealSense 카메라 좌표 → Unreal 좌표 변환 */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Pose")
    static FVector ConvertCamToUnreal(float RS_X, float RS_Y, float RS_Z);

    /**
     * 두 관절 벡터 → 뼈대 쿼터니언
     * Q = FindBetweenVectors(BindDir, normalize(ToPos - FromPos))
     */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Pose")
    static FQuat ComputeBoneQuat(const FVector& FromPos, const FVector& ToPos,
                                 const FVector& BindDir);

    /** 관절 위치 조회 (Blueprint 에서 사용) */
    UFUNCTION(BlueprintCallable, Category = "Pose")
    bool GetJoint(const FString& Name, FVector& OutPosition, bool& OutValid) const;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;

private:
    UPROPERTY()
    UOSCServer* OSCServer = nullptr;

    FCriticalSection JointMutex;
    TMap<FString, FJointData> PendingJoints;
    TAtomic<int32> MessageCounter{ 0 };

    UFUNCTION()
    void HandleOSCMessage(const FOSCMessage& Message,
                          const FString& IPAddress, int32 Port);

    void FlushAndComputeRotations();
    void StoreJoint(const FString& Name,
                    float RS_X, float RS_Y, float RS_Z, float DepthM);
    void DrawDebugVisualization();

    /** TMap 에서 관절을 안전하게 가져오는 내부 헬퍼 */
    const FJointData* FindJoint(const FString& Name) const;

    /** 두 관절이 모두 유효할 때만 쿼터니언 계산 후 저장 */
    void TryComputeBone(FQuat& OutQuat,
                        const FString& FromName, const FString& ToName,
                        const FVector& BindDir);

    /** 두 관절의 중간점 계산 (척추·골반 중심 등에 사용) */
    static bool MidPoint(const FJointData* A, const FJointData* B, FVector& OutMid);
};
