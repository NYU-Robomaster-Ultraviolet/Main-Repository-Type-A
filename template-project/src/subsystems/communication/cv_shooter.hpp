#ifndef CV_SHOOTER_COMMAND_HPP_
#define CV_SHOOTER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/shooter/shooter_subsystem.hpp"
#include "drivers.hpp"
namespace shooter{

class CVShooterCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ShooterSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] shooter a pointer to the shooter to be passed in that this
     *      Command will interact with.
     */
    CVShooterCommand(ShooterSubsystem *const shooter, src::Drivers *drivers);

    CVShooterCommand(const CVShooterCommand &other) = delete;

    CVShooterCommand &operator=(const CVShooterCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "shooter command"; }

    // Will turn on the flywheels the first time a target is found, and never turn them back off
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    ShooterSubsystem *const shooter;

    src::Drivers *drivers;

    bool flyWheelsOn = false;
}; //class ShooterUserCommand : public tap::control::Command
} //namespace shooter
#endif  // SHOOT_MOVEMENT_COMMAND_HPP_