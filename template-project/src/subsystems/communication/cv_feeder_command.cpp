#include "cv_feeder_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace feeder
{
CVFeeder::CVFeeder(
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

void  CVFeeder::initialize() {
    feeder->setTargetRPM(0);
    burstFireCooldown.restart(10);
    burstFireTimeout.restart(10);
    }

void  CVFeeder::execute()
{
    if(checkBarrelHeatLimit() || (burstFireTimeout.isExpired() && !drivers->cv_com.foundTarget())) 
        feeder->setTargetRPM(0);
    else if(drivers->cv_com.foundTarget()){
        feeder->setTargetRPM(LEVEL_ONE_FEEDER_RPM);
        burstFireTimeout.restart(1000);
    }
#ifdef TARGET_SENTRY
    // if(burstFireTimeout.isExpired() || checkBarrelHeatLimit()){
    //     feeder->setTargetRPM(0);
    // }
    // else if(drivers->cv_com.foundTarget() && burstFireCooldown.isExpired()){
    //         feeder->setTargetRPM(3000);
    //         burstFireTimeout.restart(1000);
    //         burstFireCooldown.restart(6000);
    // }
#endif
}

void  CVFeeder::end(bool) { feeder->setTargetRPM(0); }

bool  CVFeeder::isFinished() const { return false; }

bool CVFeeder::checkBarrelHeatLimit() const {
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