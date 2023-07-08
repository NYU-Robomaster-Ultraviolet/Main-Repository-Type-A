#include "unjam_feeder_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace feeder
{
UnjamFeederCommand::UnjamFeederCommand(FeederSubsystem *const feeder, 
    src::Drivers *drivers)
    : feeder(feeder),
      drivers(drivers)
{
    if (feeder == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(feeder));
}

void  UnjamFeederCommand::initialize() {
    feeder->setTargetRPM(0);
    keyPressTimeout.restart(10);
}

void  UnjamFeederCommand::execute()
{ 
    if(drivers->control_interface.getFPressed()){
        feeder->setTargetRPM(-LEVEL_ONE_FEEDER_RPM / 2);
        keyPressTimeout.restart(1000);
    }
    else(feeder->setTargetRPM(0));
}

void  UnjamFeederCommand::end(bool) { feeder->setTargetRPM(0); }

bool  UnjamFeederCommand::isFinished() const { return false; }

bool UnjamFeederCommand::checkBarrelHeatLimit() const {
    if(drivers->ref_interface.refDataValid()){
        #if defined(Target_STANDARD)
            std::pair<uint16_t, uint16_t> heatLimits = drivers->ref_interface.getShooterHeat();
            if(heatLimits.first > heatLimits.second - 30) return true;
        #elif defined (TARGET_HERO)
            std::pair<uint16_t, uint16_t> heatLimits = drivers->ref_interface.getShooterHeat();
                if(heatLimits.first > heatLimits.second - 100) return true;
        #elif defined (TARGET_SENTRY)
            std::vector<uint16_t> heatLimits = drivers->ref_interface.getShooterHeat();
            if((heatLimits[0] > heatLimits[1] - 30) || (heatLimits[2] > heatLimits[3] - 30))
                return true;
        #endif
    }
    return false;
}
}  // namespace feeder