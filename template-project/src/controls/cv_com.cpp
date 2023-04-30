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
    timeout.restart(100); //sets up timer
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
    if (!timeout.isExpired())
    {
        return 0;
    }
    timeout.restart(100);

    //for debugging imu
    sendAutoAimMsg(imuPitch, imuYaw, 0);
    
    //reads single byte
    res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(&buffer[byteIndex]),
                1);
    if (res != 1) return 0; //maybe this is not sufficient
    // write back
    writeToUart(buffer, byteIndex + 1);
    //  get first header, unpack "B" 0xE7
    readingState = WAITING_FOR_HEADER;

    switch (readingState)
    {
        case WAITING_FOR_HEADER:
        {
            byteIndex = 0;
            // read the next byte
            // drivers->leds.set(drivers->leds.C, false);
            res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(&buffer[byteIndex]),
                1);
            // if (res != 1) return 0;
            //  looking for header
            short count = 0;
            while (buffer[byteIndex] != 0xE7 && count < 10)
            {
                // drivers->leds.set(drivers->leds.C, true);
                res = drivers->uart.read(
                    tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                    (uint8_t *)(buffer),
                    1);
                count++;
            }
            //failed to find header return
            if (buffer[byteIndex] != 0xE7)
            {
                timeout.restart(100);
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
                timeout.restart(100);
                return bytes_read;
            }
            
            headerStruct = *reinterpret_cast<header *>(buffer);
            msg_len = headerStruct.length;
            msg_type = headerStruct.msg_type;
            // write back

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
            } while (res <= 0 && count < 10);

            if (res <= 0)
            {
                timeout.restart(100);
                return bytes_read;
            }

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
                    hasTarget = a.hasTarget;
                    // yaw = autoAimStruct->yaw;

                    //drivers->leds.set(drivers->leds.C, false);
                    validAngle = true;

                    writeToUart(buffer, 1);
                    readingState = WAITING_FOR_HEADER;
                    byteIndex = 0;

                    break;
                }
                case 2:
                {
                    // Align request
                    // if (byteIndex == sizeof(alignRequestStruct))
                    // {
                    //     AlignRequestStruct alignRequestStruct = (AlignRequestStruct)(buffer);
                    //     readingState = WAITING_FOR_HEADER;
                    // }
                    break;
                }
                case 3:
                {
                    // Align finish
                    // if (byteIndex == sizeof(alignFinishStruct))
                    // {
                    //     AlignFinishStruct alignFinishStruct = (AlignFinishStruct)(buffer);
                    //     readingState = WAITING_FOR_HEADER;
                    // }
                    break;
                }
                break;
            }
        }
    }
    timeout.restart(100);

    return byteIndex;
}

void CVCom::UnPackMsgs(char *buffer) {}

void CVCom::sendAutoAimMsg(int p, int y, int hasTarget)
{
    AutoAimStructObj autoAimStruct = AutoAimStructObj();
    autoAimStruct.pitch = p;
    autoAimStruct.yaw = y;
    autoAimStruct.empty1 = 0;
    autoAimStruct.empty2 = 0;
    autoAimStruct.footer = 0;
    autoAimStruct.header = 0xE7;
    autoAimStruct.msg_type = 5;
    autoAimStruct.length = 11;

    char str[sizeof(autoAimStruct)];
    memcpy(str, &autoAimStruct, sizeof(autoAimStruct));
    writeToUart(str, sizeof(autoAimStruct) - 1);
}

void CVCom::sendColorMsg(){
    ColorStructObj sending = ColorStructObj();
    sending.color = color;
    sending.msg_type = 2;
    sending.header = 0xE7;
    sending.length = 4;
    char str[sizeof(sending)];
    memcpy(str, &sending, sizeof(sending));
    writeToUart(str, sizeof(sending) - 1);
}

void CVCom::update()
{
    // set the gimbal subsystem to use the cvcom
    int bytes_read = readFromUart();
    // if (bytes_read > 0)
    // {

    // }
    // else
    //     drivers->leds.set(drivers->leds.C, true);
}
