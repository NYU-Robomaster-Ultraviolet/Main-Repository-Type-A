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

void  CVFeeder::initialize() {feeder->setTargetRPM(0);}

void  CVFeeder::execute()
{

#ifdef TARGET_SENTRY
    if(drivers->cv_com.foundTarget()){
        feeder->setTargetRPM(1500);
    }
    else{
        feeder->setTargetRPM(0);
    }
#endif
}

void  CVFeeder::end(bool) { feeder->setTargetRPM(0); }

bool  CVFeeder::isFinished() const { return false; }
}  // namespace feeder