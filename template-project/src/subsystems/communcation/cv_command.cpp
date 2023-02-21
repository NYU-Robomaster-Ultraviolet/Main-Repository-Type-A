#include "cv_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/errors/create_errors.hpp"

#include "controls/standard/control_interface.hpp"
#include "controls/standard/standard_constants.hpp"

namespace gimbal
{
CvCommand::CvCommand(GimbalSubsystem *const gimbal, src::Drivers *drivers) 
: gimbal(gimbal), drivers(drivers) 
{
}
void  CvCommand::initialize() {
    gimbal->cvInput(findRotation(YAW_ENCODER_OFFSET), LEVEL_ANGLE - gimbal->getPitchEncoder());
}

void  CvCommand::execute() {
    char *buffer = new char[1024];
    int bytes_read = readFromUart(drivers, buffer);
    if (bytes_read > 0) {
            char *temp = new char[bytes_read];
            UnPackMsgs(drivers, buffer);
            writeToUart(drivers, buffer, bytes_read);
            sendAutoAimMsg(drivers, 1, 2, 1);
    }
}

void  CvCommand::end(bool) { 

    }

bool  CvCommand::isFinished() const { return false; }

float CvCommand::findRotation(const float& destination) const {
    float rotation = destination - gimbal->getYawEncoder();
    if(rotation > M_TWOPI) rotation = -(rotation - M_TWOPI);
    else if(rotation < -M_TWOPI) rotation = -(rotation + M_TWOPI);
    return rotation;
}

int CvCommand::writeToUart(src::Drivers *drivers, std::string s)
{
    int n = s.length();
    // declaring character array
    char char_array[n + 1];
    strncpy(char_array, s.c_str(), n);
    int res = drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart6, //Uart7
        (uint8_t *)char_array,
        n);
    modm::delay_ms(100);
    return res;
}

int CvCommand::writeToUart(src::Drivers *drivers, char *s, int n)
{
    // declaring character array
    int res =
        drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart6, (uint8_t *)s, n); //Uart7
    modm::delay_ms(100);
    return res;
}

int CvCommand::readFromUart(src::Drivers *drivers, char *buffer)
{
    // declaring character array
    int bytes_read = 0;
    bool read_flag = 1;
    while (read_flag)
    {
        int res = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart6, //Uart7
            (uint8_t *)(buffer + bytes_read));
        // writeToUart(drivers,"bytes read = "+ to_string(res)+"\n");
        if (res > 0)
            bytes_read += res;
        else
            read_flag = 0;
    }
    modm::delay_ms(100);
    return bytes_read;
}

void CvCommand::UnPackMsgs(src::Drivers *drivers, char *buffer)
{
    Header headerStruct = (Header)buffer;

    //  type 1 for autoaim
    if (headerStruct->msg_type == 1)
    {
        AutoAimStruct lpsData = (AutoAimStruct)buffer;
        // writeToUart(drivers, to_string(lpsData->header) + " ");
        // writeToUart(drivers, to_string(lpsData->length) + " ");
        // writeToUart(drivers, to_string(lpsData->empty1) + " ");
        // writeToUart(drivers, to_string(lpsData->empty2) + " ");
        // writeToUart(drivers, to_string(lpsData->msg_type) + " ");
        // writeToUart(drivers, to_string(lpsData->pitch) + " ");
        // writeToUart(drivers, to_string(lpsData->yaw) + " ");
        // writeToUart(drivers, to_string(lpsData->hasTarget) + " ");
        
        updateGimbalError(lpsData->pitch, lpsData->yaw);
    }
}

void CvCommand::sendAutoAimMsg(src::Drivers *drivers, int pitch, int yaw, int hasTarget)
{
    AutoAimStructObj autoAimStruct = AutoAimStructObj();
    autoAimStruct.pitch = pitch * 100;
    autoAimStruct.yaw = yaw * 100;
    autoAimStruct.empty1 = 0;
    autoAimStruct.empty2 = 0;
    autoAimStruct.footer = 0;
    autoAimStruct.header = 5;
    autoAimStruct.hasTarget = hasTarget;
    autoAimStruct.msg_type = 1;
    autoAimStruct.length = 5;

    char str[sizeof(autoAimStruct)];
    memcpy(str, &autoAimStruct, sizeof(autoAimStruct));
    writeToUart(drivers, str, sizeof(autoAimStruct) - 1);
}

void CvCommand::updateGimbalError(int pitch, int yaw){
    float pitchF = static_cast<float>(pitch / 100);
    float yawF = static_cast<float>(yaw / 100);
    if(yawF > M_PI) yawF -= M_TWOPI;
    gimbal->cvInput(yawF, pitchF);
}

}//namespace cv