#ifndef CV_CHASSIS_COMMAND_HPP_
#define CV_CHASSIS_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis_subsystem.hpp"
#include "drivers.hpp"
#include "subsystems/gimbal/gimbal_motor_interface.hpp"
#include "tap/util_macros.hpp"
namespace chassis{

class CVChassisCommand : public tap::control::Command{
public:
    /**
     * Initializes the command with the passed in ChassisSubsystem.  Must not
     * be nullptr.
     *
     * @param[in] chassis a pointer to the chassis to be passed in that this
     *      Command will interact with.
     */
    CVChassisCommand(ChassisSubsystem *const chassis, src::Drivers *drivers, gimbal::GimbalInterface* gimbal);

    //copy controls
    CVChassisCommand(const CVChassisCommand &other) = delete;

    CVChassisCommand &operator=(const CVChassisCommand &other) = delete;

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

    /**
     * @brief This function will check the power usage against the limit, then speeds
     * if approaching the limit
     * @return true : speed was limited
     * @return false : speed was not limited
     */
    bool checkPowerLimit();

    //updates the level of the chassis and beyblade output
    bool updateChassisLevel();
private:
    ChassisSubsystem *const chassis;

    src::Drivers *drivers;

    gimbal::GimbalInterface* gimbalInterface;

    //previous input, used to prevent overiding cv commands
    float prevX = 0;
    float prevY = 0;
    float prevR = 0;

    //beyblade input
    float beybladeInput = BEYBLADE_INPUT;

    //timeout to phase out cv commands
    tap::arch::MilliTimeout timeout;

    //the limit of inputs, this can decrease from power limiting
    float limitValueRange = 1;

    //used to delay power management check
    tap::arch::MicroTimeout checkPowerTimeout;
}; //class CVChassisCommand : public tap::control::Command
} //namespace Chassis
#endif //CHASSIS_MOVEMENT_COMMAND_HPP_ 