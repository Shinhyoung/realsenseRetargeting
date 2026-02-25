using UnrealBuildTool;

public class PoseRetargetingEditorTarget : TargetRules
{
    public PoseRetargetingEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.Latest;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
        ExtraModuleNames.Add("PoseRetargeting");
    }
}
