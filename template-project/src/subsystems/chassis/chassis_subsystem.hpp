#ifndef CHASSIS_SUBSYSTEM_HPP_
#define CHASSIS_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "controls/standard/standard_constants.hpp"
#include "drivers.hpp"

namespace chassis
{
/**
 * A bare bones Subsystem for interacting with a 4 wheeled chassis.
 */

class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Constructs a new ChassisSubsystem with default parameters specified in
     * the private section of this class.
     */
    ChassisSubsystem(src::Drivers *drivers);

    //updates the string with the encoder values of the motors
    void updateWheelvalues();

    //sends a copy of the string detailing motor encoder information
    std::string getUartOutput(){return outputString;}

    //copy controls
    ChassisSubsystem(const ChassisSubsystem &other) = delete;

    ChassisSubsystem &operator=(const ChassisSubsystem &other) = delete;

    //default deconstructor
    ~ChassisSubsystem() = default;

    /**
     * Called when Subsystem is registered
     * initializes motors 
     */
    void initialize() override;

    /**
     * Called continously as long as robot is running
     * Updates the Pid calculator with desired outputs
     */
    void refresh() override;

    /**
     * @param x : right and left inputs from controller
     * @param y : foward and backward inputs from controller
     * @param r : rotation from left or right inputs from controller 
     * calculate the desired rpm of each wheel to have the desired movement
     */
    void setDesiredOutput(float x, float y, float r);

    /**
     * @param pid : the pid calculator for the given motor
     * @param motor : the motor that needs to be updated
     * @param desiredRpm : the desired rpm set for tht motor
     * using pid calculator to calculate a rpm to set to given motor to after feeding the calculator
     * the desiredRpm
     */
    void updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm);

    //checks if every motor is online
    bool motorOnline() {return frontLeftMotor.isMotorOnline() && frontRightMotor.isMotorOnline() && 
    backLeftMotor.isMotorOnline() && backRightMotor.isMotorOnline();}

    //converts the motor absolute encoder value to radians
    inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
        return (M_TWOPI * static_cast<float>(encoderValue)) / tap::motor::DjiMotor::ENC_RESOLUTION;
    }

    //getters for each motor
    const tap::motor::DjiMotor &getFrontLeftMotor() const { return frontLeftMotor; }
    const tap::motor::DjiMotor &getFrontRightMotor() const { return frontRightMotor; }
    const tap::motor::DjiMotor &getBackLeftMotor() const { return backLeftMotor; }
    const tap::motor::DjiMotor &getBackRightMotor() const { return backRightMotor; }

private:
    //all constants used in this subsystem
    CHASSIS_CONSTANTS constants;

    ///< Motors.  Use these to interact with any dji style motors.
    tap::motor::DjiMotor frontLeftMotor;
    tap::motor::DjiMotor frontRightMotor;
    tap::motor::DjiMotor backLeftMotor;
    tap::motor::DjiMotor backRightMotor;

    // PID controllers for RPM feedback from wheels
    modm::Pid<float> frontLeftPid;
    modm::Pid<float> frontRightPid;
    modm::Pid<float> backLeftPid;
    modm::Pid<float> backRightPid;

    ///< Any user input is translated into desired RPM for each motor.
    float frontLeftDesiredRpm;
    float frontRightDesiredRpm;
    float backLeftDesiredRpm;
    float backRightDesiredRpm;
    //
    std::string outputString;
};  // class ChassisSubsystem

}  // namespace chassis


#endif  // CHASSIS_SUBSYSTEM_HPP_