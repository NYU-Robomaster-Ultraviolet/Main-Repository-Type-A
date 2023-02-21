#ifndef GIMBAL_MOVEMENT_COMMAND_HPP_
#define GIMBAL_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "gimbal_subsystem.hpp"
#include "drivers.hpp"
namespace gimbal{
class GimbalMovementCommand : public tap::control::Command {
public:
    GimbalMovementCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    GimbalMovementCommand(const GimbalMovementCommand &other) = delete;

    GimbalMovementCommand &operator=(const GimbalMovementCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "gimbal movement command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    float findRotation(const float& destination) const;
private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
}; //GimbalMovementCommand


}// namespace gimbal

#endif