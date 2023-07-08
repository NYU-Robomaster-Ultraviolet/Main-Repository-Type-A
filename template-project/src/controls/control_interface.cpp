#include "control_interface.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::communication::serial;
/**
 * @brief Gets the current X input from the operator.
 * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
 *
 * @return float The current X input from the operator.
 */
namespace src::control{
    float ControlInterface::getChassisXInput() {
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    float keyboardX = drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A);

    #if defined(TARGET_HERO)
        if(ctrMode) keyboardX *= .1;
        else if(!shiftMode) keyboardX *= .2;
        else keyboardX *= .4;
        #else
        if(ctrMode) keyboardX *= .3;
        else if(!shiftMode) keyboardX *= .6;
        #endif
    
    if (prevUpdateCounterX != updateCounter) {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) + keyboardX, currTime);
        prevUpdateCounterX = updateCounter;
    }

    //float finalX = limitVal<float>((chassisXInput.getInterpolatedValue(currTime)) * X_SENSITIVITY, -X_SENSITIVITY, X_SENSITIVITY);
    float finalX = limitVal<float>((chassisXInput.getInterpolatedValue(currTime)), -1, 1);
    return finalX;
    }
    /**
     * @brief Gets the current Y input from the operator.
     * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
     *
     * @return float The current Y input from the operator.
     */
    float ControlInterface::getChassisYInput() {
        uint32_t updateCounter = drivers->remote.getUpdateCounter();
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        //this is for mouse commands
        float keyboardY = drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

        #if defined(TARGET_HERO)
        if(ctrMode) keyboardY *= .1;
        else if(!shiftMode) keyboardY *= .2;
        else keyboardY *= .4;
        #else
        if(ctrMode) keyboardY *= .3;
        else if(!shiftMode) keyboardY *= .6;
        #endif

        //check if reading a different remote input
        if (prevUpdateCounterY != updateCounter) {
            chassisYInput.update(drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) + keyboardY, currTime);
            prevUpdateCounterY = updateCounter;
        }


        //float finalY = limitVal<float>((chassisYInput.getInterpolatedValue(currTime))*Y_SENSITIVITY, -Y_SENSITIVITY, Y_SENSITIVITY);
        float finalY = limitVal<float>((chassisYInput.getInterpolatedValue(currTime)), -1, 1);

        return finalY;
    }
    /**
     * @brief Gets the current rotation input from the operator.
     * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
     *
     * @return float The current rotation input from the operator.
     */
    float ControlInterface::getChassisRotationInput() {
        uint32_t updateCounter = drivers->remote.getUpdateCounter();
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

        if (prevUpdateCounterRotation != updateCounter) {
            chassisRotationInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), currTime);
            prevUpdateCounterRotation = updateCounter;
        }

        float keyBoardR = drivers->remote.keyPressed(Remote::Key::X) - drivers->remote.keyPressed(Remote::Key::Z);  
        if(ctrMode) keyBoardR *= .3;
        else if(!shiftMode) keyBoardR *= .6;
        //float finalRotation = limitVal<float>((chassisRotationInput.getInterpolatedValue(currTime)) * Y_SENSITIVITY, -Y_SENSITIVITY, Y_SENSITIVITY);
        #if defined TARGET_HERO
        float mouseInput = 0;
        if(mouseReturnValX > .1) mouseInput = .1;
        else if(mouseReturnValX < -.1) mouseInput = -.1;
        if(mouseReturnValX > .4) mouseInput = .2;
        else if(mouseReturnValX < -.4) mouseInput = -.2;
        if(mouseReturnValX > .7) mouseInput = .3;
        else if(mouseReturnValX < -.7) mouseInput = -.3;
        float finalRotation = limitVal<float>((chassisRotationInput.getInterpolatedValue(currTime) + keyBoardR + (mouseInput)), -1, 1);
        #else
        float finalRotation = limitVal<float>((chassisRotationInput.getInterpolatedValue(currTime) + keyBoardR), -1, 1);
        #endif
        return finalRotation;
    }

    float ControlInterface::getGimbalYawInput() {
        float MouseX = drivers->remote.getMouseX(); 
        return limitVal<float>(-drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) - MouseX * 0.1, -1, 1);
    }

    float ControlInterface::getGimbalPitchInput() {
        float MouseY = drivers->remote.getMouseY();
        #if defined(TARGET_HERO)
        return limitVal<float>(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) - MouseY * 0.002, -1, 1);
        #else
        return limitVal<float>(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) - MouseY * 0.1, -1, 1);
        #endif
    }

    void ControlInterface::init(){
        shiftCheckTimeout.restart(10);
        ctrChecKTimeout.restart(10);
        fChecKTimeout.restart(10);
        MouseXCheckTimeout.restart(10);
    }

    void ControlInterface::checkKeyPresses() {
        if(shiftCheckTimeout.isExpired()){
            if(drivers->remote.keyPressed(Remote::Key::SHIFT)){
                shiftMode = !shiftMode;
                ctrMode = false;
                shiftCheckTimeout.restart(1000);
            }
        }

        if(ctrChecKTimeout.isExpired()){
            if(drivers->remote.keyPressed(Remote::Key::CTRL)){
                ctrMode = !ctrMode;
                shiftMode = false;
                ctrChecKTimeout.restart(1000);
            }
        }
        if(fChecKTimeout.isExpired()){
            if(drivers->remote.keyPressed(Remote::Key::F)){
                fPressed = true;
                fChecKTimeout.restart(1000);
            }
            else
                fPressed = false;
        }
        //gets avereage mouse input in time period
        if(MouseXCheckTimeout.isExpired()){
            uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
            uint32_t timeError = currTime - lastMouseCheckTime;
            mouseReturnValX = limitVal<float>(mouseValX / timeError, -1, 1);
            lastMouseCheckTime = currTime;
            mouseValX = 0;
            MouseXCheckTimeout.restart(200);
        }
        mouseValX += drivers->remote.getMouseX() * .0003; 
        
    }
}