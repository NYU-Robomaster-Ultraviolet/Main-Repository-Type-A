#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace chassis
{

ChassisSubsystem::ChassisSubsystem(src::Drivers *drivers, gimbal::GimbalInterface gimbal)
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
          backRightDesiredRpm(0),
          gimbalInterface(gimbal)
    {}

void ChassisSubsystem::updateWheelvalues(){
    //position of each wheel (based off of encoder)
    FRPos = wrappedEncoderValueToRadians(frontRightMotor.getEncoderUnwrapped());
    FLPos = wrappedEncoderValueToRadians(frontLeftMotor.getEncoderUnwrapped());
    BRPos = wrappedEncoderValueToRadians(backRightMotor.getEncoderUnwrapped());
    BLPos = wrappedEncoderValueToRadians(backLeftMotor.getEncoderUnwrapped());
    //get rpm of each wheel
    FRRPM = frontRightMotor.getShaftRPM();
    FLRPM = frontLeftMotor.getShaftRPM();
    BRRPM = backRightMotor.getShaftRPM();
    BLRPM = backLeftMotor.getShaftRPM();
    //get angular velocity of each wheel
    FRVelocity = FRRPM * M_TWOPI / 60;
    FLVelocity = FLRPM * M_TWOPI / 60;
    BRVelocity = BRRPM * M_TWOPI / 60;
    BLVelocity = BLRPM * M_TWOPI / 60;
    //calculates velocities
    longitudinalVelocity = (FLVelocity + FRVelocity + BLVelocity + BRVelocity) * constants.ROLLER_RADIUS / constants.NUM_WHEELS;
    transversalVelocity = (-FLVelocity + FRVelocity - BLVelocity + BRVelocity) * constants.ROLLER_RADIUS / constants.NUM_WHEELS;
    angularVelocity = (-FLVelocity + FRVelocity - BLVelocity + BRVelocity) *
         constants.ROLLER_RADIUS / (constants.NUM_WHEELS * (constants.FRONT_TO_BACK_WHEEL + constants.LEFT_TO_RIGHT_WHEEL));
    
    directionOfMovement = sqrtf(pow(longitudinalVelocity, 2) + pow(transversalVelocity, 2));
    resultantVelocity = atanf(transversalVelocity/longitudinalVelocity);
    //gimbalFrameVelocity = 
}

void ChassisSubsystem::initialize()
{
    frontLeftMotor.initialize();
    frontRightMotor.initialize();
    backLeftMotor.initialize();
    backRightMotor.initialize();
    updateWheelvalues();
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