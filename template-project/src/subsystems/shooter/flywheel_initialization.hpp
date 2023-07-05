#ifndef FLYWHEEL_INITIALIZATION_HPP_
#define FLYWHEEL_INITIALIZATION_HPP_

#include "tap/control/command.hpp"

#include "shooter_subsystem.hpp"
#include "drivers.hpp"
namespace shooter{

class FlywheelInitialization : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ShooterSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] shooter a pointer to the shooter to be passed in that this
     *      Command will interact with.
     */
    FlywheelInitialization(ShooterSubsystem *const shooter, src::Drivers *drivers);

    FlywheelInitialization(const FlywheelInitialization &other) = delete;

    FlywheelInitialization &operator=(const FlywheelInitialization &other) = delete;

    void initialize() override;

    const char *getName() const { return "shooter command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    ShooterSubsystem *const shooter;

    //tracks if shooter has power, false for no power, true for has power
    bool prevShooterPowerStatus = false;

    src::Drivers *drivers;

    tap::arch::MilliTimeout initializeTimeout;
}; //class ShooterUserCommand : public tap::control::Command
} //namespace shooter
#endif  // SHOOT_MOVEMENT_COMMAND_HPP_