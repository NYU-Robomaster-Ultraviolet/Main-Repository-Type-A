#ifndef GIMBAL_BEYBLADE_HPP_
#define GIMBAL_BEYBLADE_HPP_

#include "tap/control/command.hpp"

#include "subsystems/gimbal/gimbal_subsystem.hpp"
#include "drivers.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

namespace gimbal{
class GimbalBeybladeCommand : public tap::control::Command {
public:
    GimbalBeybladeCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers);

    GimbalBeybladeCommand(const GimbalBeybladeCommand &other) = delete;

    GimbalBeybladeCommand &operator=(const GimbalBeybladeCommand &other) = delete;

    void initialize() override;

    const char *getName() const { return "gimbal beyblade command"; }

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;
private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
    bool noTurn;
    float offset = 1.7;
    float rotation = (BEYBLADE_INPUT / offset);

}; //GimbalBeybladeCommand


}// namespace gimbal

#endif