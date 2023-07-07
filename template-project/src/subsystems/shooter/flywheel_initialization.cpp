#include "flywheel_initialization.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace shooter
{
FlywheelInitialization::FlywheelInitialization(
    ShooterSubsystem *const shooter,
    src::Drivers *drivers)
    : shooter(shooter),
      drivers(drivers)
{
    if (shooter == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(shooter));
}

void  FlywheelInitialization::initialize() {shooter->changeOnFlag();}

void  FlywheelInitialization::execute()
{   
    if(initializeTimeout.isExpired()){
        initializeTimeout.restart(1000);
        shooter->initializeFlywheel();
    }
    
}

void  FlywheelInitialization::end(bool) {shooter->changeOnFlag();}

bool  FlywheelInitialization::isFinished() const { return false; }
}  // namespace shooter