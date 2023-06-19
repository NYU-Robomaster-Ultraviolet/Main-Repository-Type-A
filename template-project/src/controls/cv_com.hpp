#ifndef CV_COM_HPP_
#define CV_COM_HPP_
#include "tap/architecture/timeout.hpp"

#include "modm/math/geometry/angle.hpp"

#include "ref_interface.hpp"
#include "cv_com_structs.hpp"

namespace src
{
class Drivers;
}
namespace cv{
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
    void setImu(int x, int y, int z){
        imuVelocityX = x;
        imuVelocityY = y;
        imuVelocityZ = z;
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
    void setEncoder(float yaw, float pitch)
    {
        encoderYaw = yaw * 100;
        encoderPitch = pitch * 100;
    }


    tap::arch::MilliTimeout receivingTimeout;
    tap::arch::MilliTimeout sendingTimeout;
    const unsigned int SENDING_TIME = 100;
    const unsigned int RECEIVING_TIME = 5;

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
    int encoderYaw = 0;
    int encoderPitch = 0;
    int imuVelocityX = 0;
    int imuVelocityY = 0;
    int imuVelocityZ = 0; 
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
}; //namespace cv
#endif