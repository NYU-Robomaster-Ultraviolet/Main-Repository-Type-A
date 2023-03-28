#include "cv_com.hpp"

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

CVCom::CVCom(src::Drivers *drivers) : drivers(drivers) {}

void CVCom::init() { timeout.restart(1000); }
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
    timeout.restart(1000);
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
    timeout.restart(1000);
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
    // get first header, unpack "B" 0xE7
    if (readingState == WAITING_FOR_HEADER)
    {
        // set led to off
        do
        {
            drivers->leds.set(drivers->leds.C, false);
            int res = drivers->uart.read(
                tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
                (uint8_t *)(buffer + bytes_read),
                1);
            if (res > 0)
            {
                bytes_read += 1;
                if (buffer[bytes_read] == 0xE7)
                {
                    readingState = READING_HEADER;
                    break;
                }
            }
        } while (res <= 0);
    }

    if (readingState == READING_HEADER)
    {
        // Header_length=5
        // Read the next 4 bytes
        int res = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
            (uint8_t *)(buffer + bytes_read),
            4);

        if (res <= 0)
        {
            return bytes_read;
        }
        if (bytes_read + 5 > buffer_size)
        {
            readingState = WAITING_FOR_HEADER;
            return bytes_read;
        }
        else
        {
            timeout.restart(1000);
            // unpack the header
            headerStruct = (Header)(buffer);
            msg_len = headerStruct->length;
            msg_type = headerStruct->msg_type;
            bytes_read += 5;
            readingState = READING_DATA;
        }
    }

    if (readingState == READING_DATA)
    {
        // read the rest of the message
        int res = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart7,  // Uart7
            (uint8_t *)(buffer + bytes_read + 1),
            msg_len);
        if (res <= 0)
        {
            return bytes_read;
        }
        bytes_read += msg_len + 1;
        // ensure headerStruct is not uninitialized
        if (headerStruct == nullptr)
        {
            readingState = WAITING_FOR_HEADER;
            return bytes_read;
        }
        timeout.restart(1000);
        switch (headerStruct->msg_type)
        {
            // 0: autoaim, 1: align_request, 3:align_finish
            case 0:
            {
                drivers->leds.set(drivers->leds.C, false);
                // Autoaim
                AutoAimStruct autoAimStruct = (AutoAimStruct)(buffer);
                pitch = autoAimStruct->pitch;
                yaw = autoAimStruct->yaw;
                validAngle = true;
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
    }

    timeout.restart(1000);
    // delayMS(100); //delay for .1 sec
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
    // need to call cvInput from the GimbalSunsystem in some way

    delete[] buffer;
}