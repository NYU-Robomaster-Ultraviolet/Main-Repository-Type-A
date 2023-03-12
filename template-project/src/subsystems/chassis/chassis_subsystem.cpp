#include "chassis_subsystem.hpp"

#include "controls/standard/standard_constants.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace chassis
{

ChassisSubsystem::ChassisSubsystem(src::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          frontLeftMotor(drivers, constants.FRONT_LEFT_MOTOR_ID, constants.CAN_BUS_MOTORS, false, "front left motor"),
          frontRightMotor(drivers, constants.FRONT_RIGHT_MOTOR_ID, constants.CAN_BUS_MOTORS, true, "front right motor"),
          backLeftMotor(drivers, constants.BACK_LEFT_MOTOR_ID, constants.CAN_BUS_MOTORS, false, "back left motor"),
          backRightMotor(drivers, constants.BACK_RIGHT_MOTOR_ID, constants.CAN_BUS_MOTORS, true, "back right motor"),
          frontLeftPid(constants.CHASSIS_MOTOR_KP,constants.CHASSIS_MOTOR_KI,constants.CHASSIS_MOTOR_KD,
          constants.CHASSIS_MOTOR_MAX_IOUT,constants.CHASSIS_MOTOR_MAX_OUT),
          frontRightPid(constants.CHASSIS_MOTOR_KP,constants.CHASSIS_MOTOR_KI,constants.CHASSIS_MOTOR_KD,
          constants.CHASSIS_MOTOR_MAX_IOUT,constants.CHASSIS_MOTOR_MAX_OUT),
          backLeftPid(constants.CHASSIS_MOTOR_KP,constants.CHASSIS_MOTOR_KI,constants.CHASSIS_MOTOR_KD,
          constants.CHASSIS_MOTOR_MAX_IOUT,constants.CHASSIS_MOTOR_MAX_OUT),
          backRightPid(constants.CHASSIS_MOTOR_KP,constants.CHASSIS_MOTOR_KI,constants.CHASSIS_MOTOR_KD,
          constants.CHASSIS_MOTOR_MAX_IOUT,constants.CHASSIS_MOTOR_MAX_OUT),
          frontLeftDesiredRpm(0),
          frontRightDesiredRpm(0),
          backLeftDesiredRpm(0),
          backRightDesiredRpm(0)
    {}

void ChassisSubsystem::updateWheelvalues(){
    float FRRPM = frontRightMotor.getShaftRPM();
    float FLRPM = frontLeftMotor.getShaftRPM();
    float BRRPM = backRightMotor.getShaftRPM();
    float BLRPM = backLeftMotor.getShaftRPM();
    float FRPos = wrappedEncoderValueToRadians(frontRightMotor.getEncoderUnwrapped());
    float FLPos = wrappedEncoderValueToRadians(frontLeftMotor.getEncoderUnwrapped());
    float BRPos = wrappedEncoderValueToRadians(backRightMotor.getEncoderUnwrapped());
    float BLPos = wrappedEncoderValueToRadians(backLeftMotor.getEncoderUnwrapped());
    //format: name,RPM,EncoderPosition(rad);
    outputString = "FR," + std::to_string(FRRPM) + ',' +  std::to_string(FRPos) + ';' + 
    "FL," + std::to_string(FLRPM) + ',' +  std::to_string(FLPos) + ';' + 
    "BR," + std::to_string(BRRPM) + ',' +  std::to_string(BRPos) + ';' + 
    "BL," + std::to_string(BLRPM) + ',' +  std::to_string(BLPos) + ';' ;
}

void ChassisSubsystem::initialize()
{   
    frontLeftMotor.initialize();
    frontRightMotor.initialize();
    backLeftMotor.initialize();
    backRightMotor.initialize();
}
void ChassisSubsystem::refresh() {
    updateRpmPid(&frontLeftPid, &frontLeftMotor, frontLeftDesiredRpm);
    updateRpmPid(&frontRightPid, &frontRightMotor, frontRightDesiredRpm);
    updateRpmPid(&backLeftPid, &backLeftMotor, backLeftDesiredRpm);
    updateRpmPid(&backRightPid, &backRightMotor, backRightDesiredRpm);
}
void ChassisSubsystem::updateRpmPid(modm::Pid<float>* pid, tap::motor::DjiMotor* const motor, float desiredRpm) {
    pid->update(desiredRpm - motor->getShaftRPM()); //updates pid
    motor->setDesiredOutput(pid->getValue()); //feeds pid output value to motor
}

/*
    Give desired setpoints for chassis movement. +x is forward, +y is right, +r is clockwise (turning right).
*/
void ChassisSubsystem::setDesiredOutput(float x, float y, float r) 
{ 
    frontLeftDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR * (x+y+r)), -constants.MAX_CURRENT_OUTPUT, constants.MAX_CURRENT_OUTPUT);
    frontRightDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR* (-x+y-r)), -constants.MAX_CURRENT_OUTPUT, constants.MAX_CURRENT_OUTPUT);
    backLeftDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR* (-x+y+r)), -constants.MAX_CURRENT_OUTPUT, constants.MAX_CURRENT_OUTPUT);
    backRightDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR* (x+y-r)), -constants.MAX_CURRENT_OUTPUT, constants.MAX_CURRENT_OUTPUT);
}
} //namespace chassis