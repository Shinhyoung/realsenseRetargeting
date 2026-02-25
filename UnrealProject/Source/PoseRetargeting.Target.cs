using UnrealBuildTool;

public class PoseRetargetingTarget : TargetRules
{
    public PoseRetargetingTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.Latest;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
        ExtraModuleNames.Add("PoseRetargeting");
    }
}
