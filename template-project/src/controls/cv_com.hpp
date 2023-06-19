#ifndef CV_COM_HPP_
#define CV_COM_HPP_
#include "tap/architecture/timeout.hpp"
#include "ref_interface.hpp"
#include "modm/math/geometry/angle.hpp"
#include "ref_interface.hpp"

namespace src
{
class Drivers;
}

class CVCom
{
public:
    CVCom(src::Drivers *drivers);

    ~CVCom();

    int writeToUart(const std::string &s);

    int writeToUart(char *s, int n);

    int readFromUart();

    void UnPackMsgs(char *buffer);

    void sendAutoAimMsg(int pitch, int yaw);

    void sendEnableMsg(bool start);

    void update();

    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }

    void invalidateAngle() { validAngle = false; }

    bool validReading() const { return validAngle; }

    bool foundTarget() const { return hasTarget; }

    bool online() const { return receivingTimeout.isExpired(); }

    bool sendingLoop();

    void init();

    void setAngles(int p, int y)
    {
        imuPitch = p;
        imuYaw = y;
    }

    void setColor(bool c) { color = c; }

    void updateHP(unsigned int h) { hp = h; }

    void sendColorMsg();

    void sendRefereeMsg();

    void changeCV(bool on) { cv_on = on; }

    bool getChassisReadFlag() const { return chassisReadFlag; }
    void resetChassisReadFlag() { chassisReadFlag = 0; }
    float getChassisX() const { return chassisX; }
    float getChassisY() const { return chassisY; }
    float getChassisR() const { return chassisR; }

    float getGimbalX() const { return gimbalX; }
    float getGimbalY() const { return gimbalY; }

    bool getGimbalReadFlag() const { return gimbalReadFlag; }
    void resetGimbalReadFlag() { gimbalReadFlag = 0; }
    void setEncoder(float yaw, float pitch) {
        encoderYaw = yaw * 100;
        encoderPitch = pitch * 100;
    }

    typedef struct autoAimStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        int pitch;
        int yaw;
        unsigned char hasTarget;
        unsigned short footer;
    } AutoAimStructObj, *AutoAimStruct;

    typedef struct sendingAngleStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        int pitch;
        int yaw;
        char footer;
    } SendingAngleStructObj, *SendingAngleStruct;

    typedef struct colorStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
        unsigned char color;
        char footer;
    } ColorStructObj, *ColorStruct;

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

    // message_type: 3
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

    // message_type: 2
    typedef struct chassisMoveStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;  // 2
        // value should be between -1000 and 1000;
        int chassisX;  // chassis x axis movement
        int chassisY;  // chassis y axis movement
        int chassisR;  // rotational movement
        // 0 : default movement
        unsigned char mode;  // used to switch command behavior
        unsigned short footer;
    } ChassisMoveStructObj, *ChassisMoveStruct;

    // message_type: 4
    typedef struct gimbalMoveStruct
    {
        unsigned char header;
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;  // 4
        // value should be between -1000 and 1000;
        int gimbalX;  // gimbal yaw movement
        int gimbalY;  // gimbal pitch movement
        // 0 : default movement, 1 : beyblade, 2 : beyblade backwards
        unsigned char mode;  // used to switch command behavior
        unsigned short footer;
    } GimbalMoveStructObj, *GimbalMoveStruct;

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
        // unsigned char
        unsigned char header;
        // unsigned short
        unsigned short length;
        unsigned char empty1;
        unsigned char empty2;
        unsigned short msg_type;
    } *Header;

    tap::arch::MilliTimeout receivingTimeout;
    tap::arch::MilliTimeout sendingTimeout;
    const unsigned int SENDING_TIME = 100;
    const unsigned int RECEIVING_TIME = 150;

private:
    src::Drivers *drivers;

    // chassisMoveValues
    float chassisX;
    float chassisY;
    float chassisR;
    bool chassisReadFlag = 0;

    // gimbalMoveValues
    float gimbalX;
    float gimbalY;
    bool gimbalReadFlag = 0;

    unsigned char mode = 0;

    // yaw then pitch
    float yaw;
    float pitch;
    int imuYaw = 0;
    int imuPitch = 0;
    int encoderYaw = 1;
    int encoderPitch = 2;
    int validAngle = false;
    bool hasTarget = false;
    size_t byteIndex = 0;
    size_t buffer_size = 100;
    char *buffer;
    bool color = 0;  // 0: red 1: blue for enemy color
    unsigned int hp;
    bool cv_on = 0;
    bool flag = 0;
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