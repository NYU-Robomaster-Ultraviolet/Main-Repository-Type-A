#ifndef SHOOT_MOVEMENT_COMMAND_HPP_
#define SHOOT_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "shooter_subsystem.hpp"
#include "drivers.hpp"
namespace shooter{

class ShootUserCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ShooterSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] shooter a pointer to the shooter to be passed in that this
     *      Command will interact with.
     */
    ShootUserCommand(ShooterSubsystem *const shooter, src::Drivers *drivers);

    ShootUserCommand(const ShootUserCommand &other) = delete;

    ShootUserCommand &operator=(const ShootUserCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "shooter command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    ShooterSubsystem *const shooter;

    src::Drivers *drivers;
}; //class ShooterUserCommand : public tap::control::Command
} //namespace shooter
#endif  // SHOOT_MOVEMENT_COMMAND_HPP_