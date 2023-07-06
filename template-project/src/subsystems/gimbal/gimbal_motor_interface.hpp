#ifndef GIMBAL_MOTOR_INTERFACE_HPP_
#define GIMBAL_MOTOR_INTERFACE_HPP_

#include "gimbal_subsystem.hpp"
namespace gimbal{
class GimbalInterface{
public:
    GimbalInterface(GimbalSubsystem* gimbal) : gimbal(gimbal){}
    
    //gets the yaw angle wrapped around 0 - 2pi, with the foward position treated as 0 rads
    float getYawEncoder() const {return gimbal->wrapAngle(gimbal->getYawEncoder() - YAW_ENCODER_OFFSET);}

    //gets the pitch encoder value
    float getPitchEncoder() const {return gimbal->getPitchEncoder();}

    //gets yaw velocity based on encoder rpm measurements
    float getYawVelocity() const {return gimbal->getYawVelocity();}

    //not used since no imu
    float getChassisBeybladeInput() const {return gimbal->getChassisBeybladeSpeed();}

    //this allows the gimbal to check if it should be beyblading or not (from cv inputs mostly)
    bool getBeybladeMote() const {return beybladeOn;}
    void setBeyblade(bool on) {beybladeOn = on;}
private:
    GimbalSubsystem* gimbal;
    bool beybladeOn = false;
};//class gimbalInterface

}//namespace gimbal

#endif