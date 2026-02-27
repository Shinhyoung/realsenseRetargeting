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
//  한 사람의 관절 맵 래퍼 (TArray<TMap> 는 UPROPERTY 불가라서 래핑)
// ──────────────────────────────────────────
USTRUCT(BlueprintType)
struct FPersonJointMap
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Pose")
    TMap<FString, FJointData> Joints;
};

// ──────────────────────────────────────────
//  전신 뼈대 쿼터니언 결과 (12 뼈대)
// ──────────────────────────────────────────
USTRUCT(BlueprintType)
struct FPoseBoneResult
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Neck        = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Spine       = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperArm_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperArm_R  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerArm_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerArm_R  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperLeg_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat UpperLeg_R  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerLeg_L  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat LowerLeg_R  = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Foot_L      = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") FQuat Foot_R      = FQuat::Identity;
    UPROPERTY(BlueprintReadOnly, Category = "Pose") bool  bHasData    = false;
};

// ──────────────────────────────────────────
//  OSC 수신 + 전신 쿼터니언 계산 Actor (다인원 지원)
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

    // ── 다인원 설정 ───────────────────────
    /** Python MAX_PERSONS 와 동일하게 설정 (1~3) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|MultiPerson",
              meta = (ClampMin = "1", ClampMax = "3"))
    int32 MaxPersons = 3;

    // ── 전신 관절 데이터 (PersonId → 관절이름 → 위치) ──────
    /**
     * PersonJoints[0] = 첫 번째 사람의 27개 관절
     * PersonJoints[1] = 두 번째 사람의 27개 관절
     * ...
     */
    UPROPERTY(BlueprintReadOnly, Category = "Pose|Joints")
    TArray<FPersonJointMap> PersonJoints;

    // ── 전신 쿼터니언 결과 (PersonId 인덱스) ─────────────
    UPROPERTY(BlueprintReadOnly, Category = "Pose")
    TArray<FPoseBoneResult> PersonBoneResults;

    // ── 바인드 포즈 기준 방향 (UE5 Manny T-포즈 기준) ──
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Neck       = FVector(0.f,  0.f,  1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Spine      = FVector(0.f,  0.f,  1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperArm_L = FVector(0.f, -1.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperArm_R = FVector(0.f,  1.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerArm_L = FVector(0.f, -1.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerArm_R = FVector(0.f,  1.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperLeg_L = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_UpperLeg_R = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerLeg_L = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_LowerLeg_R = FVector(0.f,  0.f, -1.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Foot_L     = FVector(1.f,  0.f,  0.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pose|BindPose")
    FVector BindDir_Foot_R     = FVector(1.f,  0.f,  0.f);

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
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Pose")
    static FVector ConvertCamToUnreal(float RS_X, float RS_Y, float RS_Z);

    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Pose")
    static FQuat ComputeBoneQuat(const FVector& FromPos, const FVector& ToPos,
                                 const FVector& BindDir);

    /** 특정 사람의 관절 위치 조회 */
    UFUNCTION(BlueprintCallable, Category = "Pose")
    bool GetJointForPerson(int32 PersonId, const FString& Name,
                           FVector& OutPosition, bool& OutValid) const;

    /** 현재 수신 중인 사람 수 */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Pose")
    int32 GetActivePersonCount() const;

    /** 특정 사람의 관절 맵 직접 접근 헬퍼 */
    const TMap<FString, FJointData>* GetJointsMapForPerson(int32 PersonId) const
    {
        if (PersonJoints.IsValidIndex(PersonId))
            return &PersonJoints[PersonId].Joints;
        return nullptr;
    }

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;

private:
    UPROPERTY()
    UOSCServer* OSCServer = nullptr;

    FCriticalSection JointMutex;
    // 버퍼: OSC 스레드 → 게임 스레드
    TArray<TMap<FString, FJointData>> PendingPersonJoints;
    TAtomic<int32> MessageCounter{ 0 };

    UFUNCTION()
    void HandleOSCMessage(const FOSCMessage& Message,
                          const FString& IPAddress, int32 Port);

    void FlushAndComputeRotations();
    void ComputeBonesForPerson(int32 PersonId);
    void DrawDebugVisualization();

    /** 특정 사람의 TMap 에서 관절을 안전하게 가져오는 헬퍼 */
    static const FJointData* FindJointInMap(
        const TMap<FString, FJointData>& JointsMap, const FString& Name);

    /** 특정 사람의 TMap 에서 두 관절이 유효하면 쿼터니언 계산 */
    void TryComputeBoneInMap(FQuat& OutQuat,
                             const TMap<FString, FJointData>& JointsMap,
                             const FString& FromName, const FString& ToName,
                             const FVector& BindDir);

    /** 두 관절의 중간점 계산 */
    static bool MidPoint(const FJointData* A, const FJointData* B, FVector& OutMid);
};
