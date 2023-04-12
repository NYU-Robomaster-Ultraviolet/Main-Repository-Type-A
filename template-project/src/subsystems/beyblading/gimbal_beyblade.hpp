#ifndef GIMBAL_BEYBLADE_HPP_
#define GIMBAL_BEYBLADE_HPP_

#include "tap/control/command.hpp"

#include "subsystems/gimbal/gimbal_subsystem.hpp"
#include "drivers.hpp"
namespace gimbal{
class GimbalBeybladeCommand : public tap::control::Command {
public:
    GimbalBeybladeCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    GimbalBeybladeCommand(const GimbalBeybladeCommand &other) = delete;

    GimbalBeybladeCommand &operator=(const GimbalBeybladeCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "gimbal movement command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    float findRotation(const float& destination) const;
private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
    bool noTurn;
    float offset = 1.7;
    float rotation = (0.2 / offset);

}; //GimbalBeybladeCommand


}// namespace gimbal

#endif