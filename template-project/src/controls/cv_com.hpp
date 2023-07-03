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

/**
 * @brief CVCom is used as an interface for handling information send or received through the UART ports designated with
 * communication with the external jetson computer for computer vision
 */
class CVCom
{
public:
    CVCom(src::Drivers *drivers);

    ~CVCom();

    /**
     * @brief Initializes cv communication, aka timers and starting signals
     * 
     * @return ** void 
     */
    void init();

    /**
     * @brief sends a string through the UART port used
     * 
     * @param s : a string that would be sent as a series of unsigned chars through UART
     * @return ** int: returns the number of bytes written
     */
    int writeToUart(const std::string &s);

    /**
     * @brief sends a character array through the UART port used
     * 
     * @param s : an array of characters that would be sent as a series of unsigned chars through UART
     * @return ** int: returns the number of bytes written
     */
    int writeToUart(char *s, int n);

    /**
     * @brief Reads from UART port and stores information as members if reading is valid
     * 
     * @return ** int: returns the bytes read
     */
    int readFromUart();

    /**
     * @brief Sends the pitch and yaw of the gimbal through UART, but can send anything
     * 
     * @param pitch : pitch angle in radians * 100
     * @param yaw  : yaw andle in radians * 100
     * @return ** void 
     */
    void sendAutoAimMsg(int pitch, int yaw);

    /**
     * @brief Sends the messaged used to tell the jetson to send data or not
     * 
     * @param start : boolean used as a signal to start or stop sending info, true for start sending
     * @return ** void 
     */
    void sendEnableMsg(bool start);

    /**
     * @brief The loop called in main to do all actions in a cycle AKA sending and receiving loop
     * 
     * @return ** void 
     */
    void update();

    /**
     * @brief the loop for sending through UART
     * 
     * @return true : if information was sent in this call
     * @return false : if the timer has yet to expire and nothing is sent
     */
    bool sendingLoop();

    //getters for angles used in CV auto aiming, the validity of the data, and if it has locked onto a target
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    void invalidateAngle() { validAngle = false; }
    bool validReading() const { return validAngle; }
    bool foundTarget() const { return hasTarget; }

    //checks if the timer for receiving has expired or not
    bool online() const { return receivingTimeout.isExpired(); }


    //sets angles that will be sent to the jetson
    void setAngles(int p, int y)
    {
        imuPitch = p;
        imuYaw = y;
    }

    //sets accelerometer data that would be sent to jetson
    void setImu(int x, int y, int z){
        imuVelocityX = x;
        imuVelocityY = y;
        imuVelocityZ = z;
    }

    //sets the color of enemy team
    void setColor(bool c) { color = c; }

    //updates current hp
    void updateHP(unsigned int h) { hp = h; }

    //sends the color of the enemy team
    void sendColorMsg();

    //sends the data received from the referee system
    void sendRefereeMsg();

    //changes if cv mode should be on or off
    void changeCV(bool on) { cv_on = on; }

    //sets encoder data, multiplying by 100 for easier processing
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

    //for chassis movement at a set velocity
    bool getChassisVeloFlag() const {return setChassisFlag;}
    void resetChassisVeloFlag() {setChassisFlag = 0;}
    bool getChassisSpinFlag() const {return chassisSpinFlag;}
    void resetChassisSpinFlag() {chassisSpinFlag = 0;}

    float getChassisFowardVelo() const {return xVelocity;}
    float getChassisRightVelo() const {return yVelocity;}
    float getChassisRotationVelo() const {return modm::toRadian( spinVelocity / 1000);}

    //for chassis movement of a set distance in distance or radians
    float getChassisSpinRad() const {return modm::toRadian(spinAngle / 1000);} //1.2 to adjust for mecanum error
    bool getChassisSpinRadFlag() const {return chassisSpinFlagRadians;}
    void resetChassisSpinRadFlag() {chassisSpinFlagRadians = 0;}

    float getChassisFowardMovement() const {return forwardDistance * 2.45 / 1000;} // 2.45 to correct for error in movement calc
    float getChassisFowardFlag() const {return chassisForwardFlag;}
    void resetChassisFowardFlag() {chassisForwardFlag = 0;}

    //for chassis stop
    bool getChassisStop() const {return stop;}


    //for gimbal movement
    float getGimbalX() const { return gimbalX; }
    float getGimbalY() const { return gimbalY; }

    bool getGimbalReadFlag() const { return gimbalReadFlag; }
    void resetGimbalReadFlag() { gimbalReadFlag = 0; }

    //for mimicing remote control inputs
    float getYawPower() const {return xAnglePower;}
    float getPitchPower() const {return yAnglePower;}
    float getXPower() const {return xLinearPower;}
    float getYPower() const {return yLinearPower;}\
    
    bool getGimbalPowerFlag() const {return setPowerGimbalFlag;}
    bool getChassisPowerFlag() const {return setPowerChassisFlag;}

    void resetGimbalPowerFlag() {setPowerGimbalFlag = false;}
    void resetChassisPowerFlag() {setPowerChassisFlag = false;}

    //returns the mode of beyblading 0: Off 1: clockwise 2: counterclockwise
    unsigned char getBeybladeMode() const {return beybladeMode;}


    // the header structure for every message type
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

    //timouts for receiving and sending data along with the time between
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
    unsigned char beybladeMode = 1;

    //chassis move straight
    int forwardDistance = 0; //in mm
    float forwardVelocity = 0; //m/s, divide by 1000 to get
    bool chassisForwardFlag = 0;

    //spin chassis 
    int spinAngle = 180000; //in degrees
    float spinVelocity = 100000; //degree/s, divide by 1000 to get
    bool chassisSpinFlag = 0;
    bool chassisSpinFlagRadians = 0;

    //set power wrapped around 0 - 100
    float xLinearPower = 0;
    float yLinearPower = 0;
    float xAnglePower = 0;
    float yAnglePower = 0;
    bool setPowerGimbalFlag = false;
    bool setPowerChassisFlag = false;

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