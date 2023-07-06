#include "chassis_subsystem.hpp"

#include "tap/communication/serial/remote.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap;

namespace chassis
{

ChassisSubsystem::ChassisSubsystem(src::Drivers *drivers, gimbal::GimbalInterface * gimbal)
        : tap::control::Subsystem(drivers),
          frontLeftMotor(drivers, constants.FRONT_LEFT_MOTOR_ID, constants.CAN_BUS_MOTORS, constants.FRONT_LEFT_REVERSED, "front left motor"),
          frontRightMotor(drivers, constants.FRONT_RIGHT_MOTOR_ID, constants.CAN_BUS_MOTORS, constants.FRONT_RIGHT_REVERSED, "front right motor"),
          backLeftMotor(drivers, constants.BACK_LEFT_MOTOR_ID, constants.CAN_BUS_MOTORS, constants.BACK_LEFT_REVERSED, "back left motor"),
          backRightMotor(drivers, constants.BACK_RIGHT_MOTOR_ID, constants.CAN_BUS_MOTORS, constants.BACK_RIGHT_REVERSED, "back right motor"),
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


std::pair<float, float> ChassisSubsystem::transformVelocity(float fowardVelo, float rightVelo, float fowardOffset) const {
    //since y component is offset 90 degrees to the negatie, correct for this
    float rightOffset = fowardOffset + M_PI_2;
    //for the case of negative angles
    if(rightOffset > M_TWOPI) rightOffset -= M_TWOPI;
    //track if x or y components need to be negated
    uint8_t quadrentX = 1;
    uint8_t quadrentY;

    //convert into workable angles and negatives if needed for both (Unit circle)
    float chassisXVectorOffsetX;
    float chassisXVectorOffsetY;
    float chassisYVectorOffsetX;
    float chassisYVectoxrOffsetY;

    float newFrameForwardVelo = 0;
    float newFrameRightVelo = 0;
    
    //check positions of vectors in a unit circle
    if(fowardOffset > M_PI * 1.5 ){
        fowardOffset -= (M_PI * 1.5);
        quadrentX = 4;
    }
    else if(fowardOffset > M_PI){
        fowardOffset -= M_PI;
        quadrentX = 3;
    }
    else if(fowardOffset > M_PI / 2){
        fowardOffset -= (M_PI / 2);
       quadrentX = 2;
    }

    quadrentY = quadrentX + 1;
    if(quadrentY > 4) quadrentY = 1;

    //calcualte net x and y velocities from gimbal frame
    switch(quadrentX){
        case(1):{
            newFrameForwardVelo = cosf(fowardOffset) * fowardVelo;
            newFrameRightVelo = sinf(fowardOffset) * fowardVelo;
        }
        case(2):{
            newFrameForwardVelo = -sinf(fowardOffset) * fowardVelo;
            newFrameRightVelo = cosf(fowardOffset) * fowardVelo;
        }
        case(3):{
            newFrameForwardVelo = -cosf(fowardOffset) * fowardVelo;
            newFrameRightVelo = -sinf(fowardOffset) * fowardVelo;
        }
        case(4):{
            newFrameForwardVelo = sinf(fowardOffset) * fowardVelo;
            newFrameRightVelo = -cosf(fowardOffset) * fowardVelo;
        }
    }
    switch(quadrentY){
        case(1):{
            newFrameForwardVelo = cosf(rightOffset) * rightVelo;
            newFrameRightVelo = sinf(rightOffset) * rightVelo;
        }
        case(2):{
            newFrameForwardVelo = -sinf(rightOffset) * rightVelo;
            newFrameRightVelo = cosf(rightOffset) * rightVelo;
        }
        case(3):{
            newFrameForwardVelo = -cosf(rightOffset) * rightVelo;
            newFrameRightVelo = -sinf(rightOffset) * rightVelo;
        }
        case(4):{
            newFrameForwardVelo = sinf(rightOffset) * rightVelo;
            newFrameRightVelo = -cosf(rightOffset) * rightVelo;
        }
    }

    return std::pair<float, float>(newFrameForwardVelo, newFrameRightVelo);
}
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
    //X velocity foward
    longitudinalVelocity =  2.45 * (FLVelocity + FRVelocity + BLVelocity + BRVelocity) * constants.ROLLER_RADIUS / constants.NUM_WHEELS;
    //y velocity right
    transversalVelocity = 2.45 * -(-FLVelocity + FRVelocity + BLVelocity - BRVelocity) * constants.ROLLER_RADIUS / constants.NUM_WHEELS;
    //right is positive rotation, .8 is the estimated error from mecanum wheels
    angularVelocity = .8 * (-((-FLVelocity + FRVelocity - BLVelocity + BRVelocity) *
         constants.ROLLER_RADIUS) / (constants.NUM_WHEELS * (constants.FRONT_TO_BACK_WHEEL + constants.LEFT_TO_RIGHT_WHEEL)));
    
    directionOfMovement = sqrtf(pow(longitudinalVelocity, 2) + pow(transversalVelocity, 2));
    resultantVelocity = atanf(transversalVelocity/longitudinalVelocity);

    //find shift
    float gimbalFrameOffsetX = gimbalInterface->getYawEncoder();
    std::pair<float, float> transformedVelocities = 
        transformVelocity(longitudinalVelocity, transversalVelocity, gimbalInterface->getYawEncoder());
    gimbalFrameVelocityX = transformedVelocities.first;
    gimbalFrameVelocityY = transformedVelocities.second;

    //tracking time passed
    uint32_t time = tap::arch::clock::getTimeMilliseconds();
    timeError = time - prevTime;
    prevTime = time;
    float timeErrorSeconds = timeError / 1000;

    //calculate angular accelration
    angularAcceleration = (angularVelocity - prevAngularVelocity) / (timeErrorSeconds);

    // //calculate linear acceleration of x and y axis
    longitudinalAcceleration = (longitudinalVelocity - prevLongitudinalVelocity) / (timeErrorSeconds);
    transversalAcceleration = (transversalVelocity - prevTransversalVelocity) / (timeErrorSeconds);

    //calculate distances moved based off of acceleration, initial velocity, and time
    radiansTraveled = (prevAngularVelocity * timeError / 1000) ;
     //   + (.5 * (angularVelocity - prevAngularVelocity) * timeErrorSeconds);

    //distance moved by chassis foward and back in chassis frame
    chassisFrameDistanceTraveledX = (longitudinalVelocity * timeError / 1000);
    //    + (.5 * (longitudinalVelocity - prevLongitudinalVelocity) * timeErrorSeconds);

    //distance moved by chassis left and right in chassis frame
    chassisFrameDistanceTraveledY = (prevTransversalVelocity * timeError / 1000);
        // + (.5 * (transversalVelocity - prevTransversalVelocity) * timeErrorSeconds);
    
    //distance moved by chassis foward and back in gimbal frame
    gimbalFrameDistanceTraveledX = gimbalFrameVelocityX * timeError / 1000;

    //distance moved by chassis left and right in gimbal frame
    gimbalFrameDistanceTraveledY = gimbalFrameVelocityY * timeError / 1000;

    //toldal distance traveled fowards in gimbal frame
    gimbalFrameDistanceTraveled = gimbalFrameOffsetX * timeErrorSeconds;

    //tracks previous velocities
    prevAngularVelocity = angularVelocity;
    prevLongitudinalVelocity = longitudinalVelocity;
    prevTransversalVelocity = transversalVelocity;
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
    //changes maximum wheel ouput based on level
    if(robotLevel == 3)
        maximumPower = constants.POWER_LIMIT_THREE;
    else if(robotLevel == 2)
        maximumPower = constants.POWER_LIMIT_TWO;
    else 
        maximumPower = constants.POWER_LIMIT_ONE;

    //runs pid and sets output for wheels
    updateWheelvalues();
    
    //is set to drive at given velocity, change velocity
    if(velocityMoveFlag) {
        setVelocityOutput();
    }

    //for driving a set amount of distance
    if(targetRadians || targetDistance) {
        distanceReached = false; //tracks if finished movement
        if(fabsf(targetRadians) < constants.MIN_RADIANS){ //deadzone to prevent overshoots
            targetRadians = 0;
        }
        else {
            targetRadians -= radiansTraveled; //removes from target radians
            //chances velocity to match the given rotational velocity against measured velocity from encoder value
            if(angularVelocity != 0 && fabsf(angularVelocity - targetRotationVelocity) > .017){
                currRotationSampleInput += (angularVelocity < targetRotationVelocity) ? .002 : -.002;
            }
        }
        if(fabsf(targetDistance) < constants.MIN_DISTANCE){ //deadzone to prevent overshoots
            targetDistance = 0;
        }
        else {
            targetDistance -= chassisFrameDistanceTraveledX; //removes from target distance
            //chances velocity to match the given velocity against measured velocity from encoder value
            if(longitudinalVelocity != 0 && fabsf(longitudinalVelocity - targetFowardVelocity) > .02){
                currFowardSampleInput += (longitudinalVelocity < targetFowardVelocity) ? .002 : -.002;
            }
        }
        //stops movement if reached targets
        setDesiredOutput(
            0,
            targetDistance ? limitVal<float>(currFowardSampleInput, -1, 1) : 0,
            targetRadians ? limitVal<float>(currRotationSampleInput, -1, 1) : 0
        );
    }
    else distanceReached = true;

    //flag to stop moving
    if(stopFlag) setDesiredOutput(0, 0, 0);

    //updates pid controllers with current rpms
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
    if(beybladeMode){
        x = limitVal<float> (x, -.2, .2);
        y = limitVal<float> (y, -.2, .2);
    }
    frontLeftDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR * (x+y+r)), -maximumPower, maximumPower);
    frontRightDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR* (-x+y-r)), -maximumPower, maximumPower);
    backLeftDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR* (-x+y+r)), -maximumPower, maximumPower);
    backRightDesiredRpm = tap::algorithms::limitVal<float>(
        (constants.RPM_SCALE_FACTOR* (x+y-r)), -maximumPower, maximumPower);
}

void ChassisSubsystem::setTargetVelocity(float x, float y){
    targetFowardVelocity = x;
    currFowardSampleInput = .2;
    targetRightVelocity = y;
    currFowardSampleInput = .2;
}
void ChassisSubsystem::setRotationVelocity(float r){
    targetRotationVelocity = r;
    currRotationSampleInput = .2;
}
void ChassisSubsystem::setVelocityOutput()
{   
    float transformAngle = M_TWOPI - gimbalInterface->getYawEncoder();
    float gimbalFrameFowardError = targetFowardVelocity - gimbalFrameVelocityX;
    float gimbalFrameRightError = targetRightVelocity - gimbalFrameVelocityY;
    std::pair<float, float> transformedVelos = 
        transformVelocity(gimbalFrameFowardError, gimbalFrameRightError, gimbalInterface->getYawEncoder());
    float rotationError = targetRotationVelocity - angularVelocity;
    setDesiredOutput(transformedVelos.first, transformedVelos.second , rotationError);
}

void ChassisSubsystem::setRotationRadians(float r){
    targetRadians = r;
}

void ChassisSubsystem::setFowardMovement(float x){
    targetDistance = x;
}

void ChassisSubsystem::moveAllignWithGimbal(){
    float rotation = gimbalInterface->getYawEncoder();
    if(rotation > .005 && (rotation < M_TWOPI - .005 )){
        float rotation = gimbalInterface->getYawEncoder();
        if(rotation > M_TWOPI) rotation = -(rotation - M_TWOPI);
        else if(rotation < -M_TWOPI) rotation = -(rotation + M_TWOPI);
        targetRadians = rotation;
    }
    else targetRadians = 0;
    }

} //namespace chassis