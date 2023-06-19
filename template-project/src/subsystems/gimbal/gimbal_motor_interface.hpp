#ifndef GIMBAL_MOTOR_INTERFACE_HPP_
#define GIMBAL_MOTOR_INTERFACE_HPP_

#include "gimbal_subsystem.hpp"
namespace gimbal{
class GimbalInterface{
public:
    GimbalInterface(GimbalSubsystem* gimbal) : gimbal(gimbal){}
    
    float getYawEncoder() const {return gimbal->getYawEncoder() - YAW_ENCODER_OFFSET;}
    float getPitchEncoder() const {return gimbal->getPitchEncoder();}
private:
    GimbalSubsystem* gimbal;
};//class gimbalInterface

}//namespace gimbal

#endif