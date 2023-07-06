#include "feeder_movement_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace feeder
{
FeederMovementCommand::FeederMovementCommand(
    FeederSubsystem *const feeder,
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

void  FeederMovementCommand::initialize() {
    feeder->setTargetRPM(0);
    if(!checkBarrelHeatLimit()){
        uint8_t level = drivers->ref_interface.getLevel();
        if(level == 3)
            feeder->setTargetRPM(LEVEL_THREE_FEEDER_RPM); //3000
        else if(level == 2)
            feeder->setTargetRPM(LEVEL_TWO_FEEDER_RPM);
        else
            feeder->setTargetRPM(LEVEL_THREE_FEEDER_RPM);
        burstFireTimeout.restart(2000); //1000
    }
}

void  FeederMovementCommand::execute()
{

    //17mm = 10 barrel heat, 42mm = 100 barrel heat
    // if(burstFireTimeout.isExpired() || checkBarrelHeatLimit()){
    //     feeder->setTargetRPM(0);
    // }
    if(checkBarrelHeatLimit()){
        feeder->setTargetRPM(0);
    }
}

void  FeederMovementCommand::end(bool) { feeder->setTargetRPM(0); }

bool  FeederMovementCommand::isFinished() const { return false; }

bool FeederMovementCommand::checkBarrelHeatLimit() const {
    if(drivers->ref_interface.revDataValid()){
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