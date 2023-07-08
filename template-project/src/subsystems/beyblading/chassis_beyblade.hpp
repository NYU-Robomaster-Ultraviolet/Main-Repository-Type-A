#ifndef CHASSIS_BEYBLADE_HPP_
#define CHASSIS_BEYBLADE_HPP_

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis_subsystem.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"

#if defined (TARGET_STANDARD)
#include "controls/standard/standard_constants.hpp"
#elif defined (TARGET_SENTRY)
#include "controls/sentry/sentry_constants.hpp"
#elif defined (TARGET_HERO)
#include "controls/hero/hero_constants.hpp"
#endif

namespace chassis{

class ChassisBeybladeCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisBeybladeCommand(ChassisSubsystem *const chassis, src::Drivers *drivers, gimbal::GimbalInterface* gimbal);

    //copy controls
    ChassisBeybladeCommand(const ChassisBeybladeCommand &other) = delete;

    ChassisBeybladeCommand &operator=(const ChassisBeybladeCommand &other) = delete;

    /**
     * Called when the Command starts
     * Gives initial inputs to chassis subsystem
     * Should make it so that wheels stop moving
     */
    void initialize() override;

    //reading the code explains the code
    const char *getName() const { return "chassis drive command"; }

    /**
     * Called continously while the Command is running
     * Uses the control interface in drivers to get chassis inputs and gives them to the 
     * Chassis Subsystem to move the robot
     */
    void execute() override;

    /**
     * Called when Command ends
     * Should make it so that wheels stop moving
     */
    void end(bool) override;

    /**
     * Tells you if the command ends naturally,
     * which should never be the case for this command
     */
    bool isFinished() const override;

    //Will check power consumpion, limiting power if approaches limit, otherwise remove limits if not nearby
    bool checkPowerLimits();

    //updates the level of the chassis subsystem and changes beyblade speeds
    bool updateChassisLevel();
private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;

    gimbal::GimbalInterface* gimbalInterface;

    float limitValueRange = 1;

    float rotation = BEYBLADE_INPUT;

    uint8_t robotLevel = 1;

    tap::arch::MilliTimeout checkPowerTimeout;

    #if defined (TARGET_SENTRY)
    tap::arch::MilliTimeout switchTurnTimeout;
    bool turningRight = true;
    float inputVal = .35;
    uint32_t timeoutInterval = 3000;
    bool spent_already = false;
    #endif
}; //class ChassisBeybladeCommand : public tap::control::Command
} //namespace Chassis
#endif //CHASSIS_MOVEMENT_COMMAND_HPP_ 