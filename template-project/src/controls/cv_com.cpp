#include "cv_com.hpp"

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

CVCom::CVCom(src::Drivers *drivers) : drivers(drivers), byteIndex(0), buffer(new char[buffer_size]) {}

CVCom::~CVCom(){delete[] buffer;}

void CVCom::init()
{
    timeout.restart(100);
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
    int res = drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7,
        (uint8_t *)s,
        n);
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
    Header headerStruct{};
    int res = 0;
    // wait for timeout
    // if timeout not reached, return
    if (!timeout.isExpired())
    {
        return 0;
    }
    timeout.restart(100);

    //reads single byte
    res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(&buffer[byteIndex]),
                1);
    if (res != 1) return 0; //maybe this is not sufficient
    // write back
    writeToUart(buffer, byteIndex + 1);
    //  get first header, unpack "B" 0xE7

    switch (readingState)
    {
        case WAITING_FOR_HEADER:
        {
            // read the next byte
            // drivers->leds.set(drivers->leds.C, false);
            // res = drivers->uart.read(
            //     tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
            //     (uint8_t *)(&buffer[byteIndex]),
            //     1);
            // if (res != 1) return 0; //maybe this is not sufficient
            // // write back
            // writeToUart(buffer, byteIndex + 1);
            //looking for header
            if(buffer[byteIndex] != 0xE7) {
                bytes_read += 1;
                readingState = READING_HEADER;
            }
            else byteIndex++;
            break;
            // while (*reinterpret_cast<uint8_t *>(buffer + bytes_read) != 0xE7) //breaks if Jetson not connected
            // {
            //     // read the next byte
            //     res = drivers->uart.read(
            //         tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
            //         (uint8_t *)(buffer),
            //         1);
            //     if (res != 1) return 0;
            //     writeToUart(buffer, bytes_read);
            // }
            // bytes_read += 1;
            // readingState = READING_HEADER;
        }
        case READING_HEADER:
        {  
            byteIndex++;
            // Header_length=5
            // Read the next 4 bytes
            // int res = drivers->uart.read(
            //     tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
            //     (uint8_t *)(buffer + 1),
            //     4);

            // if (res <= 0)
            // {
            //     timeout.restart(100);
            //     return bytes_read;
            // }

            // unpack the header
            if(byteIndex == 4) {
                headerStruct = *reinterpret_cast<Header *>(buffer);
                msg_len = headerStruct->length;
                msg_type = headerStruct->msg_type;
                //bytes_read += 4;
                readingState = READING_DATA;
            }
            break;
        }
        case READING_DATA:
        {
            byteIndex++;
            // Read the next msg_len bytes
            // do
            // {
            //     res = drivers->uart.read(
            //         tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
            //         (uint8_t *)(buffer + 5),
            //         msg_len);
            // } while (res <= 0);

            // if (res <= 0)
            // {
            //     // timeout.restart(10);
            //     return bytes_read;
            // }
            switch (headerStruct->msg_type)
            {
                // 0: autoaim, 1: align_request, 3:align_finish
                case 0:
                {   
                    // Autoaim
                    if(byteIndex == sizeof(AutoAimStruct)){
                        AutoAimStruct autoAimStruct = *reinterpret_cast<AutoAimStruct *>(buffer);
                        pitch = autoAimStruct->pitch;
                        // yaw = autoAimStruct->yaw;
                        if (pitch > 0.45 && pitch < 0.55)
                        {
                            drivers->leds.set(drivers->leds.C, false);
                            validAngle = true;
                        }
                        readingState = WAITING_FOR_HEADER;
                    }
                    break;
                }
                case 1:
                {
                    // Align request
                    if(byteIndex == sizeof(alignRequestStruct)){
                        AlignRequestStruct alignRequestStruct = (AlignRequestStruct)(buffer);
                        readingState = WAITING_FOR_HEADER;
                    }        
                    break;
                }
                case 2:
                {   
                    // Align finish
                    if(byteIndex == sizeof(alignFinishStruct)){
                    AlignFinishStruct alignFinishStruct = (AlignFinishStruct)(buffer);
                    readingState = WAITING_FOR_HEADER;
                    }
                    break;
                }
                break;
            }

            // make blinking (rgb)

            // // unpack the header
            // bytes_read += msg_len;
            // timeout.restart(1000);
            // break;
        }
    }

    return byteIndex;
}

void CVCom::UnPackMsgs(char *buffer) {}

void CVCom::sendAutoAimMsg(int pitch, int yaw, int hasTarget)
{
    AutoAimStructObj autoAimStruct = AutoAimStructObj();
    autoAimStruct.pitch = pitch;
    autoAimStruct.yaw = yaw;
    autoAimStruct.empty1 = 0;
    autoAimStruct.empty2 = 0;
    autoAimStruct.footer = 0;
    autoAimStruct.header = 5;
    autoAimStruct.msg_type = 1;
    autoAimStruct.length = 5;

    char str[sizeof(autoAimStruct)];
    memcpy(str, &autoAimStruct, sizeof(autoAimStruct));
    writeToUart(str, sizeof(autoAimStruct) - 1);
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
