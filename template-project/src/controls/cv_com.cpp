#include "cv_com.hpp"

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

CVCom::CVCom(src::Drivers *drivers) : drivers(drivers), byteIndex(0), buffer(new char[buffer_size])
{
}

CVCom::~CVCom() { delete[] buffer; }

void CVCom::init()
{
    setColor(!drivers->refSerial.isBlueTeam);
    sendColorMsg();
    receivingTimeout.restart(RECEIVING_TIME);  // sets up timer
    sendingTimeout.restart(100);
    // thread for CVCom::update()
}
int CVCom::writeToUart(const std::string &s)
{
    const int n = s.length();
    // declaring character array
    char char_array[n + 1];
    strncpy(char_array, s.c_str(), n);
    int res = drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
        (uint8_t *)char_array,
        n);
    return res;
}

int CVCom::writeToUart(char *s, int n)
{
    // declaring character array
    int res =
        drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart7, (uint8_t *)s, n);
    return res;
}

bool CVCom::sendingLoop()
{
    if (!sendingTimeout.isExpired()) return 0;
    sendingTimeout.restart(SENDING_TIME);
    sendAutoAimMsg(
        int(modm::toRadian(drivers->mpu6500.getPitch()) * 100),
        int(modm::toRadian(drivers->mpu6500.getYaw()) * 100));
    sendColorMsg();
    sendEnableMsg(cv_on);
    return 1;
}

int CVCom::readFromUart()
{
    // declaring character array
    size_t bytes_read = 0;
    bool read_flag = 1;
    size_t i = 0;
    size_t msg_len = 0;
    size_t msg_type = 0;
    header headerStruct{};
    int res = 0;
    // wait for timeout
    // if timeout not reached, return
    if (!receivingTimeout.isExpired())
    {
        return 0;
    }
    receivingTimeout.restart(RECEIVING_TIME);

    //  get first header, unpack "B" 0xE7
    readingState = WAITING_FOR_HEADER;

    switch (readingState)
    {
        case WAITING_FOR_HEADER:
        {
            byteIndex = 0;
            // read the next byte
            res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(&buffer[byteIndex]),
                1);
            // if (res != 1) return 0;
            //  looking for header
            short count = 0;
            while (buffer[byteIndex] != 0xE7 && count < 50)
            {
                res = drivers->uart.read(
                    tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                    (uint8_t *)(buffer),
                    1);
                count++;
            }
            // failed to find header return
            if (buffer[byteIndex] != 0xE7)
            {
                receivingTimeout.restart(RECEIVING_TIME);
                return 1;
            }
            bytes_read += 1;
            readingState = READING_HEADER;
            byteIndex++;
        }
        case READING_HEADER:
        {
            // Header_length=5
            // Read the next 4 bytes
            int res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(buffer + 1),
                4);

            if (res <= 0)
            {
                receivingTimeout.restart(RECEIVING_TIME);
                return bytes_read;
            }

            headerStruct = *reinterpret_cast<header *>(buffer);
            msg_len = headerStruct.length;
            msg_type = headerStruct.msg_type;

            // bytes_read += 4;
            readingState = READING_DATA;
        }
        case READING_DATA:
        {
            // Read the next msg_len bytes
            short count = 0;
            do
            {
                res = drivers->uart.read(
                    tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                    (uint8_t *)(buffer + 5),
                    msg_len);
                count++;
            } while (res <= 0 && count < 50);

            if (res <= 0)
            {
                receivingTimeout.restart(READING_DATA);
                return bytes_read;
            }
            drivers->leds.set(drivers->leds.D, flag);
            flag = !flag;

            switch (headerStruct.msg_type)
            {
                // 0: autoaim, 1: align_request, 3:align_finish
                case 1:
                {
                    // Autoaim
                    autoAimStruct a = *reinterpret_cast<autoAimStruct *>(buffer);
                    // convert *100 pitch int values to float without truncating
                    pitch = (a.pitch) / 100.0;
                    yaw = (a.yaw) / 100.0;

                    // cap each value to 1 or -1
                    int cap = 1;
                    if (pitch > cap)
                        pitch = cap;
                    else if (pitch < -cap)
                        pitch = -cap;
                    if (yaw > cap)
                        yaw = cap;
                    else if (yaw < -cap)
                        yaw = -cap;

                    hasTarget = a.hasTarget;
                    validAngle = true;

                    writeToUart(buffer, 1);
                    readingState = WAITING_FOR_HEADER;
                    byteIndex = 0;

                    break;
                }
                case 2:
                {
                    // chassisMovement
                    chassisMoveStruct c = *reinterpret_cast<chassisMoveStruct *>(buffer);
                    chassisX = c.chassisX / 1000.0;
                    chassisY = c.chassisY / 1000.0;
                    chassisR = c.chassisR / 1000.0;
                    chassisReadFlag = true;

                    mode = c.mode;
                    break;
                }
                case 3:
                {
                    // Align finish
                    alignFinishStruct a = *reinterpret_cast<alignFinishStruct *>(buffer);
                    gimbalReadFlag = true;
                    validAngle = true;
                    chassisReadFlag = true;
                    yaw = 0;
                    pitch = 0;
                    gimbalX = 0;
                    gimbalY = 0;
                    chassisX = 0;
                    chassisY = 0;
                    chassisR = 0;
                    break;
                }
                case 4:
                {
                    gimbalMoveStruct g = *reinterpret_cast<gimbalMoveStruct *>(buffer);
                    gimbalX = g.gimbalX / 100.0;
                    gimbalY = g.gimbalY / 100.0;
                    gimbalReadFlag = true;
                    mode = g.mode;
                    break;
                }
                break;
            }
        }
    }
    receivingTimeout.restart(RECEIVING_TIME);

    return byteIndex;
}

void CVCom::UnPackMsgs(char *buffer) {}

void CVCom::sendAutoAimMsg(int p, int y)
{
    SendingAngleStructObj autoAimStruct = SendingAngleStructObj();
    autoAimStruct.pitch = p;
    autoAimStruct.yaw = y;
    autoAimStruct.empty1 = 0;
    autoAimStruct.empty2 = 0;
    autoAimStruct.footer = 0;
    autoAimStruct.header = 0xE7;
    autoAimStruct.msg_type = 5;
    autoAimStruct.length = 8;

    char str[sizeof(autoAimStruct)];
    memcpy(str, &autoAimStruct, sizeof(autoAimStruct));
    writeToUart(str, sizeof(autoAimStruct) - 1);
}

void CVCom::sendColorMsg()
{
    ColorStructObj sending = ColorStructObj();
    sending.header = 0xE7;
    sending.color = color;
    sending.msg_type = 2;
    sending.length = 1;
    sending.empty1 = 0;
    sending.empty2 = 0;
    sending.footer = 0;
    char str[sizeof(sending)];
    memcpy(str, &sending, sizeof(sending));
    drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
        (uint8_t *)(str),
        sizeof(sending));
    // writeToUart(str, sizeof(sending) - 1);
}

void CVCom::sendEnableMsg(bool start)
{
    ColorStructObj sending = ColorStructObj();
    sending.header = 0xE7;
    sending.color = start;
    sending.msg_type = 1;
    sending.length = 1;
    sending.empty1 = 0;
    sending.empty2 = 0;
    sending.footer = 0;
    char str[sizeof(sending)];
    memcpy(str, &sending, sizeof(sending));
    drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
        (uint8_t *)(str),
        sizeof(sending));
    // writeToUart(str, sizeof(sending) - 1);
}

void CVCom::update()
{
    // set the gimbal subsystem to use the cvcom
    int bytes_read = readFromUart();
    sendingLoop();
    // if (bytes_read > 0)
    // {

    // }
    // else
    //     drivers->leds.set(drivers->leds.C, true);
}
