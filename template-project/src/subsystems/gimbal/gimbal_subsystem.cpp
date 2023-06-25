#include "gimbal_subsystem.hpp"

#include "modm/math/geometry/angle.hpp"

namespace gimbal
{
// constructor
GimbalSubsystem::GimbalSubsystem(src::Drivers *drivers)
    : tap::control::Subsystem(drivers),
      yawMotor(
          drivers,
          constants.YAW_MOTOR_ID,
          constants.CAN_BUS_MOTORS_YAW,
          false,
          "Yaw Motor",
          tap::motor::DjiMotor::ENC_RESOLUTION / 2,
          constants.STARTING_YAW_ROT),
      pitchMotor(drivers, 
        constants.PITCH_MOTOR_ID, 
        constants.CAN_BUS_MOTORS_PITCH, false, 
        "Pitch Motor"),
      targetYaw(0.0f),
      targetPitch(0.0f),
      currentYawMotorSpeed(0.0f),
      currentPitchMotorSpeed(0.0f),
      yawMotorPid(constants.YAW_PID),
      pitchMotorPid(constants.PITCH_PID)
{
}

// initilaizes the gimbal motors and make them not get any power
void GimbalSubsystem::initialize()
{
    pastTime = tap::arch::clock::getTimeMilliseconds();
    setIMU(0, constants.STARTING_PITCH + constants.LEVEL_ANGLE);
    yawMotor.initialize();
    yawMotor.setDesiredOutput(0);
    pitchMotor.initialize();
    pitchMotor.setDesiredOutput(0);
    uint32_t currentPitchEncoder = pitchMotor.getEncoderWrapped();
    uint32_t currentYawEncoder = yawMotor.getEncoderWrapped();
    setEncoderPitchAngle(wrappedEncoderValueToRadians(currentPitchEncoder));
    setEncoderYawAngle(wrappedEncoderValueToRadians(currentYawEncoder));
    startingPitch = encoderPitch;
    startingYaw = encoderYaw;
    currentYaw = startingYaw;
    currentPitch = startingPitch;
    targetYaw = startingYaw;
    targetPitch = startingPitch;
}

void GimbalSubsystem::refresh()
{
    u_int32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    timeError = currentTime - pastTime;
    pastTime = currentTime;
    drivers->leds.set(drivers->leds.A, true);
    drivers->leds.set(drivers->leds.B, yawMotor.isMotorOnline());
    drivers->leds.set(drivers->leds.C,yawMotor.isMotorOnline());
    drivers->leds.set(drivers->leds.D, yawMotor.isMotorOnline());
    drivers->leds.set(drivers->leds.E, yawMotor.isMotorOnline());
    drivers->leds.set(drivers->leds.F, yawMotor.isMotorOnline());
    drivers->leds.set(drivers->leds.G, yawMotor.isMotorOnline());
    drivers->leds.set(drivers->leds.H, false);
    if(isCalibrated() && imuStatesCalibrated()){
        setPitchImu();
        setYawImu();
    }
    if (yawMotor.isMotorOnline())
        {
        currentYawMotorSpeed = getYawMotorRPM();  // gets the rotational speed from motor
        setEncoderYawAngle(wrappedEncoderValueToRadians(yawMotor.getEncoderWrapped()));
        currentYaw = encoderYaw;
    }
    if (pitchMotor.isMotorOnline())
        {
        currentPitchMotorSpeed = getPitchMotorRPM();  // gets the rotational speed from motor
        setEncoderPitchAngle(wrappedEncoderValueToRadians(pitchMotor.getEncoderWrapped()));
        currentPitch = encoderPitch;
    }
    // if no inputs, lock gimbal
    if (inputsFound)
    {
        if (yawMotor.isMotorOnline())
        {
            // if(isCalibrated() && imuStatesCalibrated()){
            //     currentYaw = imuYaw;
            //     if(firstSetYaw) {
            //         targetYaw = currentYaw;
            //         firstSetYaw = false;
            //     }
            // }
            // else currentYaw = encoderYaw;
            updateYawPid();
        }
        if (pitchMotor.isMotorOnline())
        {
            // if(isCalibrated() && imuStatesCalibrated()){
            //     currentPitch = imuPitch;
            //     if(firstSetPitch) {
            //         targetPitch = currentPitch;
            //         firstSetPitch = false;
            //     }
            // }
            // else currentPitch = encoderPitch;
            updatePitchPid();
        }
    }
    else
    {
        targetPitch = currentPitch;
        targetYaw = currentYaw;
    }
}

// Takes in a encoded wrapped motor value and turns it into radians
inline float GimbalSubsystem::wrappedEncoderValueToRadians(int64_t encoderValue)
{
    return (M_TWOPI * static_cast<float>(encoderValue)) / tap::motor::DjiMotor::ENC_RESOLUTION;
}

// this function will use the angel pid to determine the angel the robot will need to move to, then
// use a motor speed pid to change the motor speed accordingly
void GimbalSubsystem::updateYawPid()
{
    // find rotation
    yawError = targetYaw - currentYaw;
    //Limits the error to keep rotations consistant
    if (yawError > constants.MAX_YAW_ERROR)
        yawError -= M_TWOPI;
    else if (yawError < -constants.MAX_YAW_ERROR)
        yawError += M_TWOPI;
    // Keeps within range of radians
    if (-(constants.YAW_MINIMUM_RADS) < yawError && yawError < constants.YAW_MINIMUM_RADS)
    {
        yawMotor.setDesiredOutput(0.0f);
    }
    // else if(isCalibrated() && imuStatesCalibrated() && -(constants.YAW_MINIMUM_IMU_RADS) < yawError &&
    //  yawError < constants.YAW_MINIMUM_IMU_RADS){
    //     yawMotor.setDesiredOutput(0.0f);
    // }
    else
    {
        yawMotorPid.runController( yawError, // * constants.MOTOR_SPEED_FACTOR,
            getYawMotorRPM(), timeError);
        yawMotorOutput = limitVal<float>( yawMotorPid.getOutput(), -constants.MAX_YAW_SPEED,
            constants.MAX_YAW_SPEED);
        if (-constants.MIN_YAW_SPEED < yawMotorOutput && yawMotorOutput < constants.MIN_YAW_SPEED)
            yawMotorOutput = 0;
        else yawMotor.setDesiredOutput(yawMotorOutput);
    }
}

// updates the pitch angle
void GimbalSubsystem::updatePitchPid()
{
    pitchError = (targetPitch - currentPitch);
    pitchMotorPid.runController(
        pitchError * constants.MOTOR_SPEED_FACTOR,
        getPitchMotorRPM(),
        timeError);
    if (-(constants.PITCH_MINIMUM_RADS) < pitchError && pitchError < constants.PITCH_MINIMUM_RADS)
    {
        pitchMotorOutput = 0;
    }
    else
    {
        pitchMotorOutput = limitVal<float>( pitchMotorPid.getOutput(), -constants.MAX_PITCH_SPEED,
            constants.MAX_PITCH_SPEED);
    }
    pitchMotorOutput += gravityCompensation();
    if (-constants.MIN_PITCH_SPEED < pitchMotorOutput && pitchMotorOutput < constants.MIN_PITCH_SPEED)
        pitchMotorOutput = 0;
    else
        pitchMotor.setDesiredOutput(pitchMotorOutput);
}

// this method will check if the motor is turning, and will return a new output to try to reach
// stable position
float GimbalSubsystem::gravityCompensation()
{
    float limitAngle = M_PI / 2;
    limitAngle = limitVal<float>(currentPitch - PITCH_ENCODER_OFFSET - (constants.LEVEL_ANGLE), -M_PI, M_PI);
    return constants.GRAVITY_COMPENSATION_SCALAR * cosf(limitAngle);
}

// this is the function that is called through the remote control input
void GimbalSubsystem::controllerInput(float yawInput, float pitchInput)
{
    setYawAngle(targetYaw + (yawInput * constants.YAW_SCALE));
    setPitchAngle(targetPitch + (pitchInput * constants.PITCH_SCALE));
    inputsFound = true;
}

/**
 * @brief This is the function that is called when CV team is sending offset angles through UART
 *
 * @param yawInput  new yaw value; should be between -2 PI and 2 PI, exclusive
 * @param pitchInput  new pitch value; should be between -2 PI and 2 PI, exclusive
 */
void GimbalSubsystem::cvInput(float yawInput, float pitchInput)
{
    yawInput = limitVal<float>(yawInput, -M_TWOPI, M_TWOPI);
    pitchInput = limitVal<float>(pitchInput, -M_TWOPI, M_TWOPI);
    // adds inputted angle to current angle
    setYawAngle(currentYaw + yawInput);
    setPitchAngle(currentPitch + pitchInput);
    inputsFound = true;
}

void GimbalSubsystem::noInputs()
{
    inputsFound = false;
    pitchMotor.setDesiredOutput(0);
    yawMotor.setDesiredOutput(0);
}

void GimbalSubsystem::setIMU(float yaw, float pitch)
{
    imuYaw = yaw;
    imuPitch = pitch;
}

void GimbalSubsystem::findVelocityImu(uint32_t time){
    //gets acelerations
    imuAx = drivers->mpu6500.getAx();
    imuAy = drivers->mpu6500.getAy();
    imuAz = drivers->mpu6500.getAz();
    //if aceleration = 0, make velocity
    imuVx += imuAx * time * 0.001;
    imuVy += imuAy * time * 0.001;
    imuVz += imuAz * time * 0.001;
}
}  // namespace gimbal