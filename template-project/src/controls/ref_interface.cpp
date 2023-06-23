#include "ref_interface.hpp"
#include "drivers.hpp"

void RefInterface::updateData(){
   tap::communication::serial::RefSerialData::Rx::RobotData data = drivers->refSerial.getRobotData();
   tap::communication::serial::RefSerialData::Rx::GameData gameData = drivers->refSerial.getGameData();
   //general robot data
   returnData.robotID = 1;//static_cast<unsigned char>(data.robotId);
   returnData.maxHP = 2;// data.maxHp;
   returnData.currHP = 3; //data.currentHp;
   returnData.robotLevel = 4;//data.robotLevel;

   //match timing and stage
   returnData.stage = 5;//static_cast<uint8_t> (gameData.gameStage);
   returnData.timeLeft = 6;//gameData.stageTimeRemaining; 

   //All robots HP
   //redHP
   returnData.redHero1 = 7;//data.allRobotHp.red.hero1;
   // returnData.redEngineer2 = data.allRobotHp.red.engineer2;
   returnData.redStandard3 = 8;//data.allRobotHp.red.standard3;
   returnData.redStandard4 = 9;//data.allRobotHp.red.standard4;
   returnData.redStandard5 = 10;//data.allRobotHp.red.standard5;
   returnData.redSentry7 = 11;//data.allRobotHp.red.sentry7;
   // returnData.redOutpost = data.allRobotHp.red.outpost;
   returnData.redBase = 12;//data.allRobotHp.red.base;
   //blueHP
   returnData.blueHero1 = 13;//data.allRobotHp.blue.hero1;
   // returnData.blueEngineer2 = data.allRobotHp.blue.engineer2;
   returnData.blueStandard3 = 14;//data.allRobotHp.blue.standard3;
   returnData.blueStandard4 = 15;//data.allRobotHp.blue.standard4;
   returnData.blueStandard5 = 16;//data.allRobotHp.blue.standard5;
   returnData.blueSentry7 = 17;//data.allRobotHp.blue.sentry7;
   // returnData.blueOutpost = data.allRobotHp.blue.outpost;
   returnData.blueBase = 18;//data.allRobotHp.blue.base;

   //returnData.remainingCoins = data.remainingCoins;

   //RFID info (all bools)
   returnData.rfid = 19;//static_cast<u_int8_t>(data.rfidStatus.value);

   //armor
   returnData.damagedArmorId = 20;//static_cast<u_int8_t> (data.damagedArmorId);
   returnData.damageType = 21;//static_cast<u_int8_t> (data.damageType);

   returnData.warningLevel = 22;//data.refereeWarningData.level; //0: none 1: yellow card 2: red card 3: forfeture 
   returnData.warningRobotID = 23;//static_cast<u_int8_t>(data.refereeWarningData.foulRobotID);   //Robot ID 1-9 for red, 101-109 for blue

   returnData.receivedDpms = 24;//data.receivedDps * 1000; //converts dps to dpms to avoid floats
   returnData.lastReceivedWarningRobotTime = 25;//data.refereeWarningData.lastReceivedWarningRobotTime;  ///< Last time (in milliseconds) that a warning was received.



   // //chassis data
   powerUsage = data.chassis.power;
   powerLimit = data.chassis.powerConsumptionLimit;

   // //shooting data
   heat17ID1 = data.turret.heat17ID1;
   heat17ID2 = data.turret.heat17ID2;
   heat42 = data.turret.heat42;

   heatLimit17ID1 = data.turret.heatLimit17ID1;
   heatLimit17ID2 = data.turret.heatLimit17ID2;
   heatLimit42 = data.turret.heatLimit42;
}