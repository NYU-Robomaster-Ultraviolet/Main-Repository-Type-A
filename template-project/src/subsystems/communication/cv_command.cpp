#include "cv_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/control_interface.hpp"

namespace gimbal
{
CVGimbal::CVGimbal(GimbalSubsystem *const gimbal, src::Drivers *drivers, GimbalInterface * gimbalInter)
: gimbal(gimbal), drivers(drivers), gimbalInterface(gimbalInter)
{
     if (gimbal == nullptr)
    {
        return;
    }
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem *>(gimbal));
}
void  CVGimbal::initialize() {
    gimbal->changeCVMode(true);
    targetFoundCooldown.restart(10);
    targetFoundAdjustmentWindow.restart(10);
    gimbal->allignGimbal();
    drivers->cv_com.changeCV(true);
    #if defined (TARGET_SENTRY)
    //gimbal->setBeybladeMode(3);
    #else
    gimbal->setBeybladeMode(0);
    #endif
}

void  CVGimbal::execute() {

    drivers->cv_com.setAngles(modm::toDegree(gimbal->getPitchEncoder() - PITCH_ENCODER_OFFSET) - 90, 
        modm::toDegree(gimbal->wrapAngle(gimbal->getYawEncoder() - YAW_ENCODER_OFFSET)));
    // #if defined (TARGET_SENTRY)
    // //unsigned char beyblade = drivers->cv_com.getBeybladeMode();
    // #elif defined (TARGET_HERO) || defined (TARGET_STANDARD)
    // unsigned char beyblade = gimbalInterface->getBeybladeMote();
    // gimbal->setBeybladeMode(beyblade);
    // #endif
    #if defined (TARGET_HERO) || defined (TARGET_STANDARD)
    unsigned char beyblade = gimbalInterface->getBeybladeMote();
    gimbal->setBeybladeMode(beyblade);
    #endif
    float yawInput = 0;
    float pitchInput = 0;
    if(drivers->cv_com.validReading()){
        if(gimbalInterface->getBeybladeMote()){
            //adjustment phase
            if(!targetFoundAdjustmentWindow.isExpired()){
                gimbal->setBeybladeMode(4);
            }
            //sets to adjustment phase
            else if(drivers->cv_com.foundTarget()){
                gimbal->changeSlowBeyblade(.5);
                gimbal->setBeybladeMode(4);
                targetFoundAdjustmentWindow.restart(1000);
                targetFoundCooldown.restart(5000);
            }
            // //compensate for cv
            // else if(drivers->cv_com.foundTarget()){
            //     gimbal->setBeybladeMode(5);
            //     beyblading = true;
                
            //     targetFoundCooldown.restart(10000);
            // }
            #if defined TARGET_SENTRY //return to scouting mode
                else if(targetFoundCooldown.isExpired()){
                    gimbal->changeSlowBeyblade(1);
                    gimbal->setBeybladeMode(3);
                    beyblading = false;
                }
            #endif
        }
        yawInput = drivers->cv_com.getYaw();
        pitchInput = drivers->cv_com.getPitch();
        if(beyblading){
            if(yawInput < 0) yawInput *= 2;
            else if(yawInput > 0) yawInput *= .5;
        }
        drivers->cv_com.invalidateAngle();
        gimbal->cvInput(yawInput, pitchInput);
    }
    // else if(drivers->cv_com.getGimbalReadFlag()){
    //     float xInput = drivers->cv_com.getGimbalX();
    //     float yInput = drivers->cv_com.getGimbalY();
    //     gimbal->controllerInput(xInput, yInput);
    //     drivers->cv_com.resetGimbalReadFlag();
    // }
    //beyblade inputs
}

void  CVGimbal::end(bool) {
    gimbal->setBeybladeMode(0);
    drivers->cv_com.changeCV(0);
    gimbal->changeCVMode(false);
    gimbal->cvInput(0, 0);
    gimbal->noInputs();
    beyblading = false;
    gimbal->changeSlowBeyblade(1);
}

bool  CVGimbal::isFinished() const { return false; }



}//namespace cv