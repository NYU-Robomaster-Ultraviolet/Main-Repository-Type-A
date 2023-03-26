#include "cv_com.hpp"
#include "drivers.hpp"

CVCom::CVCom(src::Drivers *drivers) : drivers(drivers){}

int CVCom::writeToUart(const std::string& s)
{
    const int n = s.length();
    // declaring character array
    char char_array[n + 1];
    strncpy(char_array, s.c_str(), n);
    int res = drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7, //Uart7
        (uint8_t *)char_array,
        n);
    //delayMS(100); //delay for .1 sec
    return res;
}

int CVCom::writeToUart(char *s, int n)
{
    // declaring character array
    int res =
        drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart7, (uint8_t *)s, n); //Uart7
    //delayMS(100); //delay for .1 sec
    return res;
}

int CVCom::readFromUart(char *buffer)
{
    // declaring character array
    int bytes_read = 0;
    bool read_flag = 1;
    while (read_flag)
    {
        int res = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart7, //Uart7
            (uint8_t *)(buffer + bytes_read));
        // writeToUart(drivers,"bytes read = "+ to_string(res)+"\n");
        if (res > 0)
            bytes_read += res;
        else
            read_flag = 0;
    }
    //delayMS(100); //delay for .1 sec
    return bytes_read;
}

void CVCom::UnPackMsgs(char *buffer)
{
    Header headerStruct = (Header)buffer;
    std::vector<float> returnVec;
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
        float yaw = static_cast<float>(lpsData->yaw) / 100;
        float pitch = static_cast<float>(lpsData->pitch) / 100;
        std::pair<float, float> temp(yaw, pitch);
        angle = temp;
        validAngle = true;
    }
    
}

void CVCom::sendAutoAimMsg(int pitch, int yaw, int hasTarget)
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
    writeToUart(str, sizeof(autoAimStruct) - 1);
}

void CVCom::update(){
    char *buffer = new char[1024];
    int bytes_read = readFromUart(buffer);
    if (bytes_read > 0) {
            char *temp = new char[bytes_read];
            UnPackMsgs( buffer);
            writeToUart(buffer, bytes_read);
            delete[] temp;
    }
    sendAutoAimMsg(1, 2, 1);
    delete[] buffer;
}