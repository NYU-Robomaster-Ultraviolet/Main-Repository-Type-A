#ifndef CHASSIS_MOVEMENT_COMMAND_HPP_
#define CHASSIS_MOVEMENT_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "chassis_subsystem.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"
namespace chassis{

class ChassisMovementCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    ChassisMovementCommand(ChassisSubsystem *const chassis, src::Drivers *drivers, gimbal::GimbalInterface* gimbal);

    //copy controls
    ChassisMovementCommand(const ChassisMovementCommand &other) = delete;

    ChassisMovementCommand &operator=(const ChassisMovementCommand &other) = delete;

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
private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;

    gimbal::GimbalInterface* gimbalInterface;
}; //class ChassisMovementCommand : public tap::control::Command
} //namespace Chassis
#endif //CHASSIS_MOVEMENT_COMMAND_HPP_ 