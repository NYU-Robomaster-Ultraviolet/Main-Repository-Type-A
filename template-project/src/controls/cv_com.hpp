#ifndef CV_COM_HPP_
#define CV_COM_HPP_
#include "tap/architecture/timeout.hpp"

#include "modm/math/geometry/angle.hpp"

namespace src
{
class Drivers;
}

class CVCom
{
public:
    CVCom(src::Drivers *drivers);

    int writeToUart(const std::string &s);

    int writeToUart(char *s, int n);

    int readFromUart(char *buffer);

    void UnPackMsgs(char *buffer);

    void sendAutoAimMsg(int pitch, int yaw, int hasTarget);

    void update();

    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }

    void invalidateAngle() { validAngle = false; }

    bool validReading() const { return validAngle; }

    bool online() const { return !timeout.isExpired(); }

    void init();

    typedef struct autoAimStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        float pitch;
        float yaw;
        unsigned char hasTarget;
        unsigned short footer;
    } AutoAimStructObj, *AutoAimStruct;

    // message_type: 1
    typedef struct alignRequestStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        unsigned short x;
        unsigned short y;
        unsigned short r;
        unsigned short footer;
    } AlignRequestStructObj, *AlignRequestStruct;

    // message_type: 2
    typedef struct alignFinishStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        unsigned char x;
        unsigned short footer;
    } AlignFinishStructObj, *AlignFinishStruct;

    // Receive
    typedef struct enableStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        bool start;
        unsigned short footer;
    } EnableStructObj, *EnableStruct;

    // Send/recieve
    typedef struct header
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
    } *Header;

private:
    src::Drivers *drivers;

    // paw then pitch
    float yaw;
    float pitch;
    bool validAngle = false;
    tap::arch::MilliTimeout timeout;
    size_t buffer_size = 32;
    // reading state enum
    enum ReadingState
    {
        WAITING_FOR_HEADER = 0,
        READING_HEADER,
        READING_DATA
    };
    ReadingState readingState = WAITING_FOR_HEADER;
};  // class CVCom
#endif