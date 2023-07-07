#ifndef CV_COMMAND_HPP_
#define CV_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_subsystem.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"
#include "tap/architecture/timeout.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

namespace gimbal{

class CVGimbal : public tap::control::Command {
public:
    CVGimbal(GimbalSubsystem *const gimbal, src::Drivers *drivers, GimbalInterface* gimbalInter);

    CVGimbal(const CVGimbal &other) = delete;

    CVGimbal &operator=(const CVGimbal &other) = delete;

    //brings gimbal to set position and tells jetson to start sending information
    void initialize() override;

    const char *getName() const { return "cv command"; }

    /**
     * @brief Handles CV given offset inputs
     * 
     * @return ** void 
     */
    void execute() override;

    //turns tells jetson to stop sending information
    void end(bool) override;

    bool isFinished() const override;


private:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
    GimbalInterface* gimbalInterface;
    tap::arch::MilliTimeout targetFoundCooldown;
    tap::arch::MilliTimeout targetFoundAdjustmentWindow;
    bool beyblading;
    bool previousTargetFound = false;

}; //CVGimbal


}// namespace cv

#endif