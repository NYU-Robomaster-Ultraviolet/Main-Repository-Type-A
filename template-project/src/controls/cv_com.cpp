#include "cv_com.hpp"

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

CVCom::CVCom(src::Drivers *drivers) : drivers(drivers) {}

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
    // delayMS(100); //delay for .1 sec
    // timeout.restart(1000);
    return res;
}

int CVCom::writeToUart(char *s, int n)
{
    // declaring character array
    int res = drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7,
        (uint8_t *)s,
        n);  // Uart7
    // delayMS(100); //delay for .1 sec
    // timeout.restart(1000);
    return res;
}

int CVCom::readFromUart(char *buffer)
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
    // writeToUart(buffer, bytes_read);
    //  get first header, unpack "B" 0xE7

    switch (readingState)
    {
        case WAITING_FOR_HEADER:
        {
            // read the next byte
            // drivers->leds.set(drivers->leds.C, false);
            res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(buffer + bytes_read),
                1);

            if (res != 1) return 0;
            // write back
            writeToUart(buffer, bytes_read);

            while (*reinterpret_cast<uint8_t *>(buffer + bytes_read) != 0xE7)
            {
                // read the next byte
                res = drivers->uart.read(
                    tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                    (uint8_t *)(buffer),
                    1);
                if (res != 1) return 0;
                writeToUart(buffer, bytes_read);
            }
            bytes_read += 1;
            readingState = READING_HEADER;
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

            // unpack the header
            headerStruct = *reinterpret_cast<Header *>(buffer);
            msg_len = headerStruct->length;
            msg_type = headerStruct->msg_type;
            bytes_read += 4;
            readingState = READING_DATA;
        }
        case READING_DATA:
        {
            // Read the next msg_len bytes
            do
            {
                res = drivers->uart.read(
                    tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                    (uint8_t *)(buffer + 5),
                    msg_len);
            } while (res <= 0);

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
                    AutoAimStruct autoAimStruct = *reinterpret_cast<AutoAimStruct *>(buffer);
                    pitch = autoAimStruct->pitch;
                    // yaw = autoAimStruct->yaw;
                    if (pitch > 0.45 && pitch < 0.55)
                    {
                        drivers->leds.set(drivers->leds.C, false);
                        validAngle = true;
                    }

                    break;
                }
                case 1:
                {
                    // Align request
                    AlignRequestStruct alignRequestStruct = (AlignRequestStruct)(buffer);
                    break;
                }
                case 2:
                {
                    // Align finish
                    AlignFinishStruct alignFinishStruct = (AlignFinishStruct)(buffer);
                    break;
                }
            }

            // make blinking (rgb)
            readingState = WAITING_FOR_HEADER;

            // unpack the header
            bytes_read += msg_len;
            timeout.restart(1000);
            break;
        }
    }

    return bytes_read;
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
    // set the gimbad subsystem to use the cvcom
    char *buffer = new char[buffer_size];
    int bytes_read = readFromUart(buffer);
    // if (bytes_read > 0)
    // {

    // }
    // else
    //     drivers->leds.set(drivers->leds.C, true);

    delete[] buffer;
}
