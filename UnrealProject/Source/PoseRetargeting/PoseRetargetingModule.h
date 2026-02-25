#pragma once
#include "Modules/ModuleManager.h"

class FPoseRetargetingModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
