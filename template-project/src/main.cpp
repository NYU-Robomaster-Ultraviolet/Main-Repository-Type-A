/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of taproot-template-project.
 *
 * taproot-template-project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * taproot-template-project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with taproot-template-project.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "controls/robot_control.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);
// frequency for mpu6500
static constexpr float SAMPLE_FREQUENCY = 1000.0f;

static int writeToUart(src::Drivers *drivers, std::string s);
static int writeToUart(src::Drivers *drivers, char *s, int n);
static int readFromUart(src::Drivers *drivers, char *buffer);
void UnPackMsgs(src::Drivers *drivers, char *buffer);

typedef struct autoAimStruct
{
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
    unsigned short pitch;
    unsigned short yaw;
    bool hasTarget;
    unsigned short footer;
} AutoAimStructObj, *AutoAimStruct;

typedef struct header
{
    unsigned char header;
    unsigned short length;
    unsigned char empty1;
    unsigned char empty2;
    unsigned short msg_type;
} *Header;

void sendAutoAimMsg(src::Drivers *drivers, int pitch, int yaw, int hasTarget);

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();
    // Timer for mpu6500 periodicIMUUpdate (only in TYPE-C board)
    tap::arch::PeriodicMilliTimer mainLoopTimeout(1000.0f / SAMPLE_FREQUENCY);
    initializeIo(drivers);
    src::control::initializeSubsystemCommands(drivers);

#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        char *buffer = new char[1024];
        int bytes_read = readFromUart(drivers, buffer);

        // AutoAimStruct lpsData = (AutoAimStruct)buffer;
        if (bytes_read > 0)
        {
            char *temp = new char[bytes_read];
            UnPackMsgs(drivers, buffer);
            writeToUart(drivers, buffer, bytes_read);
            sendAutoAimMsg(drivers, 1, 2, 1);
        }

        //(only in TYPE-C board)
        if (mainLoopTimeout.execute())
        {
            drivers->mpu6500.periodicIMUUpdate();
        }
        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(src::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    // Added initialization of mpu6500 (only in TYPE-C board)
    drivers->mpu6500.init();
    drivers->mpu6500.requestCalibration();
    drivers->refSerial.initialize();
    // drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart7, 9600>();
}

float yaw, pitch, roll;
tap::communication::sensors::imu::ImuInterface::ImuState imuStatus;

static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
}

static int writeToUart(src::Drivers *drivers, std::string s)
{
    int n = s.length();
    // declaring character array
    char char_array[n + 1];
    strncpy(char_array, s.c_str(), n);
    int res = drivers->uart.write(
        tap::communication::serial::Uart::UartPort::Uart7,
        (uint8_t *)char_array,
        n);
    modm::delay_ms(100);
    return res;
}

static int writeToUart(src::Drivers *drivers, char *s, int n)
{
    // declaring character array
    int res =
        drivers->uart.write(tap::communication::serial::Uart::UartPort::Uart7, (uint8_t *)s, n);
    modm::delay_ms(100);
    return res;
}

static int readFromUart(src::Drivers *drivers, char *buffer)
{
    // declaring character array
    int bytes_read = 0;
    bool read_flag = 1;
    while (read_flag)
    {
        int res = drivers->uart.read(
            tap::communication::serial::Uart::UartPort::Uart7,
            (uint8_t *)(buffer + bytes_read));
        // writeToUart(drivers,"bytes read = "+ to_string(res)+"\n");
        if (res > 0)
            bytes_read += res;
        else
            read_flag = 0;
    }
    modm::delay_ms(100);
    return bytes_read;
}

void UnPackMsgs(src::Drivers *drivers, char *buffer)
{
    Header headerStruct = (Header)buffer;

    //  type 1 for autoaim
    if (headerStruct->msg_type == 1)
    {
        AutoAimStruct lpsData = (AutoAimStruct)buffer;
        // writeToUart(drivers, to_string(lpsData->header) + " ");
        // writeToUart(drivers, to_string(lpsData->length) + " ");
        // writeToUart(drivers, to_string(lpsData->empty1) + " ");
        // writeToUart(drivers, to_string(lpsData->empty2) + " ");
        // writeToUart(drivers, to_string(lpsData->msg_type) + " ");
        // writeToUart(drivers, to_string(lpsData->pitch) + " ");
        // writeToUart(drivers, to_string(lpsData->yaw) + " ");
        // writeToUart(drivers, to_string(lpsData->hasTarget) + " ");
    }
}

void sendAutoAimMsg(src::Drivers *drivers, int pitch, int yaw, int hasTarget)
{
    AutoAimStructObj autoAimStruct = AutoAimStructObj();
    autoAimStruct.pitch = pitch * 100;
    autoAimStruct.yaw = yaw * 100;
    autoAimStruct.empty1 = 0;
    autoAimStruct.empty2 = 0;
    autoAimStruct.footer = 0;
    autoAimStruct.header = 5;
    autoAimStruct.hasTarget = hasTarget;
    autoAimStruct.msg_type = 1;
    autoAimStruct.length = 5;

    char str[sizeof(autoAimStruct)];
    memcpy(str, &autoAimStruct, sizeof(autoAimStruct));
    writeToUart(drivers, str, sizeof(autoAimStruct) - 1);
}