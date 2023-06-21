#ifndef CV_COM_HPP_
#define CV_COM_HPP_
#include "tap/architecture/timeout.hpp"

#include "modm/math/geometry/angle.hpp"

#include "ref_interface.hpp"
#include "cv_com_receive_structs.hpp"
#include "cv_com_sending_structs.hpp"

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

    void setEncoder(float yaw, float pitch)
    {
        encoderYaw = yaw * 100;
        encoderPitch = pitch * 100;
    }


    //for chassis movement (mimic remote input)
    bool getChassisReadFlag() const { return chassisReadFlag; }
    void resetChassisReadFlag() { chassisReadFlag = 0; }
    float getChassisX() const { return chassisX; }
    float getChassisY() const { return chassisY; }
    float getChassisR() const { return chassisR; }

    //for chassis movment in velocity
    bool getChassisVeloFlag() const {return setChassisFlag;}
    void resetChassisVeloFlag() {setChassisFlag = 0;}
    bool getChassisSpinFlag() const {return chassisSpinFlag;}
    void resetChassisSpinFlag() {chassisSpinFlag = 0;}

    float getChassisFowardVelo() const {return xVelocity;}
    float getChassisRightVelo() const {return yVelocity;}
    float getChassisRotationVelo() const {return spinVelocity;}

    //for chassis movement in distance or radians
    float getChassisSpinRad() const {return modm::toRadian(spinAngle / 1000);}
    bool getChassisSpinRadFlag() const {return chassisSpinFlagRadians;}
    void resetChassisSpinRadFlag() {chassisSpinFlagRadians = 0;}

    float getChassisFowardMovement() const {return forwardDistance;}
    float getChassisFowardFlag() const {return chassisForwardFlag;}
    void resetChassisFowardFlag() {chassisForwardFlag = 0;}

    //for chassis stop
    bool getChassisStop() const {return stop;}


    //for gimbal movement
    float getGimbalX() const { return gimbalX; }
    float getGimbalY() const { return gimbalY; }

    bool getGimbalReadFlag() const { return gimbalReadFlag; }
    void resetGimbalReadFlag() { gimbalReadFlag = 0; }

    //for gimbal power limits
    float getYawPower() const {return xAnglePower;}
    float getPitchPower() const {return yAnglePower;}


    // Send/recieve
    typedef struct header
    {
        // unsigned char
        unsigned char header = 0xE7;
        // unsigned short
        unsigned short length;
        unsigned char empty1 = 0;
        unsigned char empty2 = 0;
        unsigned short msg_type;
    } *Header;

    tap::arch::MilliTimeout receivingTimeout;
    tap::arch::MilliTimeout sendingTimeout;
    const unsigned int SENDING_TIME = 100;
    const unsigned int RECEIVING_TIME = 5;

private:
    src::Drivers *drivers;

//DATA RECEIVED FROM JETSON

// yaw then pitch
    int validAngle = false;
    bool hasTarget = false;
    float yaw;
    float pitch;

    // chassisMoveValues
    float chassisX;
    float chassisY;
    float chassisR;
    bool chassisReadFlag = 0;

    // gimbalMoveValues
    float gimbalX;
    float gimbalY;
    bool gimbalReadFlag = 0;
    //beyblade or no beyblade
    unsigned char mode = 0;

    //chassis move straight
    int forwardDistance; //in mm
    float forwardVelocity; //m/s, divide by 1000 to get
    bool chassisForwardFlag = 0;

    //spin chassis 
    int spinAngle = 0; //in mm
    float spinVelocity = 0; //degree/s, divide by 1000 to get
    bool chassisSpinFlag = 0;
    bool chassisSpinFlagRadians = 0;

    //set power wrapped around 0 - 100
    float xLinearPower = 1;
    float yLinearPower = 1;
    float xAnglePower = 1;
    float yAnglePower = 1;

    //set velocity
    float xVelocity = 0;
    float yVelocity = 0;
    float yawVelocity = 0;
    float pitchVelocity = 0;  
    bool setChassisFlag = 0;
    bool setGimbalFlag = 0;

    //stop chassis
    bool stop;

//VALUES SENT TO JETSON

    int imuYaw = 0;
    int imuPitch = 0;
    int encoderYaw = 0;
    int encoderPitch = 0;
    int imuVelocityX = 0;
    int imuVelocityY = 0;
    int imuVelocityZ = 0; 
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