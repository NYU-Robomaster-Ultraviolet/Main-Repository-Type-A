#include "hero_shooter_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace shooter
{
HeroShooterCommand::HeroShooterCommand(
    HeroShooterSubsystem *const shooter,
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

void  HeroShooterCommand::initialize() {}

void  HeroShooterCommand::execute()
{   
    if(flag) flag = false;
    else flag = true;
    drivers->leds.set(drivers->leds.G, flag);
    shooter->setDesiredOutput(16000);
}

void  HeroShooterCommand::end(bool) { shooter->setDesiredOutput(0); }

bool  HeroShooterCommand::isFinished() const { return false; }
}  // namespace shooter