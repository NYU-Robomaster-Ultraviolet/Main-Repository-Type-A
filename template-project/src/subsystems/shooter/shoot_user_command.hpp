#ifndef SHOOT_MOVEMENT_COMMAND_HPP_
#define SHOOT_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "shooter_subsystem.hpp"
#include "drivers.hpp"
#include "tap/architecture/timeout.hpp"

namespace shooter{

class ShooterCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ShooterSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] shooter a pointer to the shooter to be passed in that this
     *      Command will interact with.
     */
    ShooterCommand(ShooterSubsystem *const shooter, src::Drivers *drivers);

    ShooterCommand(const ShooterCommand &other) = delete;

    ShooterCommand &operator=(const ShooterCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "shooter command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    ShooterSubsystem *const shooter;

    src::Drivers *drivers;

    bool wasOffline;

    tap::arch::MilliTimeout initTimeout;


}; //class ShooterUserCommand : public tap::control::Command
} //namespace shooter
#endif  // SHOOT_MOVEMENT_COMMAND_HPP_