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

void  FlywheelInitialization::initialize() {initializeTimeout.restart(1);}

void  FlywheelInitialization::execute()
{   
    if(!initializeTimeout.isExpired()){
        shooter->initializeFlywheel();
    }
    if(drivers->ref_interface.getShooterPowerStatus() && !prevShooterPowerStatus){
        shooter->initializeFlywheel();
        initializeTimeout.restart(2000);
    }
        
    prevShooterPowerStatus = drivers->ref_interface.getShooterPowerStatus();
}

void  FlywheelInitialization::end(bool) {}

bool  FlywheelInitialization::isFinished() const { return false; }
}  // namespace shooter