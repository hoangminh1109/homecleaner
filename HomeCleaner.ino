#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "NetworkConfigs.h"

/***************** PIN CONFIGURATION *******************/
// Digital pin
#define BRC_PIN 2         // Baud Rate Change

// Virtual pin
#define VPIN_STARTSTOP  V0
#define VPIN_CLEAN      V1
#define VPIN_SPOT       V2
#define VPIN_DOCK       V3

#define VPIN_CHARGE_STATE V4
#define VPIN_VOLTAGE      V5
#define VPIN_CURRENT      V6
#define VPIN_TEMP         V7
#define VPIN_BAT_CHARGE   V8
#define VPIN_BAT_CAPACITY V9

#define VPIN_LEFTMOTOR_CURR   V10
#define VPIN_RIGHTMOTOR_CURR  V11
#define VPIN_MAINBRUSH_CURR   V12
#define VPIN_SIDEBRUSH_CURR   V13
#define VPIN_STASIS_TOGGLING  V14
#define VPIN_STASIS_DISABLED  V15

#define VPIN_ROBOT_RUNNING    V16
#define VPIN_ROBOT_OIMODE     V17

#define VPIN_STOP       V50
#define VPIN_FORWARD    V51
#define VPIN_BACKWARD   V52
#define VPIN_ROTATECW   V53
#define VPIN_ROTATECCW  V54

#define VPIN_TEST             V100
#define VPIN_LASTCMD          V99
#define VPIN_LASTRESP         V98
#define VPIN_CLEARLASTCMD     V97

/*************** Widgets ****************/

/********************* Wake up Robot *******************/
void wakeupRobot()
{
    digitalWrite(BRC_PIN, LOW);
    delay(250);
    digitalWrite(BRC_PIN, HIGH);
}

void clearLastCmd()
{
  Blynk.virtualWrite(VPIN_LASTCMD, " ");
  Blynk.virtualWrite(VPIN_LASTRESP, " ");
}

/********************* Request sensor ************************/
int requestSensor(byte req[], int reqSize, byte resp[], int respSize, uint32_t tmo)
{
  uint32_t startWaitingTime;
  boolean rcvd = false;
  int rcv_size = 0;

  //TODO: for debug
  int i;
  String cmdStr = "", respStr = "";
  int byteSent = 0;
  
  if (req == NULL || reqSize == 0) return -1;
  if (resp == NULL || respSize == 0) return -1;

  //NOTE: for debug
  clearLastCmd();

  for (i = 0; i < reqSize; i++)
  {
    cmdStr += String(req[i]) + " ";
  }

  // flush remaining data
  while(Serial.available()>0)
  {
    Serial.read();
    yield();
  }

  // send request to Roomba
  byteSent = Serial.write(req, reqSize);
  Serial.flush();
  //delay((reqSize*80)/1152 + 1); // 1000(millis) * reqSize(bytes) * 8bit / 115200 (baudrate) + 1 (buffer time)

  Blynk.virtualWrite(VPIN_LASTCMD, String(byteSent) + " - " + cmdStr);
  
  startWaitingTime = millis();

  while(millis() - startWaitingTime <= tmo)
  {
    if (Serial.available() >= respSize)
    {
      rcvd = true;
      break;
    }
    yield();
  }

  if (rcvd)
  {
    rcv_size = 0;
    while(rcv_size < respSize)
    {
      resp[rcv_size++] = Serial.read();
    }

    //NOTE: for debug
    for (i = 0; i < respSize; i++)
    {
      respStr += (resp[i] < 0x10 ? "0" : "") + String(resp[i], HEX) + " ";
    }
    Blynk.virtualWrite(VPIN_LASTRESP, String(respSize) + " - " + respStr);
      
    return respSize;
  }
  else
  {
    //NOTE: for debug
    int rcv_cnt = Serial.available();
    if (rcv_cnt > 0)
    {
      while(Serial.available() > 0)
      {
        respStr += String(Serial.read(), HEX) + " ";
      }
      Blynk.virtualWrite(VPIN_LASTRESP, "Timeout " + String(rcv_cnt) + " - " + respStr);
    }
    else
    {
      Blynk.virtualWrite(VPIN_LASTRESP, "Timeout 0");
    }
    
    return 0; // timeout
  }
}

/***************** TIMERS DEFINITINO *******************/
BlynkTimer timerGetSensor;
BlynkTimer timerShowSensorData;
BlynkTimer timerRobotState;

/***************** INTERNAL VARIABLES *****************/

/* CONTROLLING */
boolean isControlled;

/* ROBOT STATE */
boolean isRunning;
uint8_t cntRobotStop;

/* state of robot
 *  1: auto, 2: manual
 *  Robot execute CLEANING, SPOT and DOCK in Auto mode.
 *  It must enter Manual mode to have manual movement,
 *     but should also return Auto mode after 5sec of no manual control
 */
uint8_t stateRobot;
uint8_t cntRobotNotDrive;

/* BATTERY */
uint8_t   batChargeState;
uint16_t  batVoltage;
int16_t   batCurrent;
int8_t    batTemp;
uint16_t  batCharged;
uint16_t  batCapacity;

String batChargeStateDesc[6] = {
  "Not charging",
  "Reconditioning Charging",
  "Full Charging",
  "Trickle Charging",
  "Waiting",
  "Charging Fault Condition"
};

String getBatChargeStateDesc(uint8_t state)
{
  if (state > 5) return "Unknown State";
  return batChargeStateDesc[state];
}

/* MOTORS */
int16_t leftMotorCurrent;
int16_t rightMotorCurrent;
int16_t mainBrushCurrent;
int16_t sideBrushCurrent;
uint8_t stasis;

/* OI MODE */
uint8_t OIMode;

/* get sensor timeout count */
uint8_t tmoCntGetSensor;

#define MAX_TMO_GETSENSOR 10

void resetSensorData()
{
  // battery    
  batChargeState  = 0;
  batVoltage      = 0;
  batCurrent      = 0;
  batTemp         = 0;
  batCharged      = 0;
  batCapacity     = 0;
  // motors
  leftMotorCurrent  = 0;
  rightMotorCurrent = 0;
  mainBrushCurrent  = 0;
  sideBrushCurrent  = 0;
  stasis            = 0;  
}

String OIModeDesc[4] = {
  "Off",
  "Passive",
  "Safe",
  "Full"
};

String getOIModeDesc(uint8_t mode)
{
  if (mode > 3) return "Unknown Mode";
  return OIModeDesc[mode];
}


/* get sensor data */
void getSensorData()
{
    byte cmd[5] = {149, 3, 3, 107, 35};
    byte resp[20];

    // Skip if Robot is not controlled
    if (!isControlled) return;

    // battery
    if (requestSensor(cmd, sizeof(cmd), resp, sizeof(resp), 1000) == sizeof(resp))
    {
      // battery    
      batChargeState  = (uint8_t)   (resp[0]);
      batVoltage      = (uint16_t)  ((uint16_t)(resp[1] << 8) | (uint16_t)(resp[2]));
      batCurrent      = (int16_t)   ((uint16_t)(resp[3] << 8) | (uint16_t)(resp[4]));
      batTemp         = (int8_t)    (resp[5]);
      batCharged      = (uint16_t)  ((uint16_t)(resp[6] << 8) | (uint16_t)(resp[7]));
      batCapacity     = (uint16_t)  ((uint16_t)(resp[8] << 8) | (uint16_t)(resp[9]));
      // motors
      leftMotorCurrent  = (int16_t)   ((uint16_t)(resp[10] << 8) | (uint16_t)(resp[11]));
      rightMotorCurrent = (int16_t)   ((uint16_t)(resp[12] << 8) | (uint16_t)(resp[13]));
      mainBrushCurrent  = (int16_t)   ((uint16_t)(resp[14] << 8) | (uint16_t)(resp[15]));
      sideBrushCurrent  = (int16_t)   ((uint16_t)(resp[16] << 8) | (uint16_t)(resp[17]));
      stasis            = (uint8_t)   (resp[18]);
      // OI mode
      OIMode            = (uint8_t)   (resp[19]);

      //tmo counter
      tmoCntGetSensor = 0;
      
    }
    else
    {
      tmoCntGetSensor++;
      if (tmoCntGetSensor > MAX_TMO_GETSENSOR) /* consecutive timeout, the may be problem with serial communcation or OI */
      {
        /* try wake up the robot and open OI */
        wakeupRobot();
        Serial.write(128);
        resetSensorData();
        tmoCntGetSensor = 0;
      }
    }
}

/* show sensor data */
void showSensorData()
{
    if (isControlled)
    {
      /* battery */
      Blynk.virtualWrite(VPIN_CHARGE_STATE, getBatChargeStateDesc(batChargeState) );
      Blynk.virtualWrite(VPIN_VOLTAGE, batVoltage);
      Blynk.virtualWrite(VPIN_CURRENT, batCurrent);
      Blynk.virtualWrite(VPIN_TEMP, batTemp);
      Blynk.virtualWrite(VPIN_BAT_CHARGE, batCharged);
      Blynk.virtualWrite(VPIN_BAT_CAPACITY, batCapacity);    
      /* motors */
      Blynk.virtualWrite(VPIN_LEFTMOTOR_CURR, leftMotorCurrent);
      Blynk.virtualWrite(VPIN_RIGHTMOTOR_CURR, rightMotorCurrent);
      Blynk.virtualWrite(VPIN_MAINBRUSH_CURR, mainBrushCurrent);
      Blynk.virtualWrite(VPIN_SIDEBRUSH_CURR, sideBrushCurrent);
      
      Blynk.virtualWrite(VPIN_STASIS_TOGGLING, ((stasis & 0x01) == 0x01) ? "Yes" : "No" );
      Blynk.virtualWrite(VPIN_STASIS_DISABLED, ((stasis & 0x02) == 0x02) ? "Yes" : "No" );

      Blynk.virtualWrite(VPIN_ROBOT_OIMODE, getOIModeDesc(OIMode));
   
    }
    else
    {
      /* battery */
      Blynk.virtualWrite(VPIN_CHARGE_STATE, "N/A");
      Blynk.virtualWrite(VPIN_VOLTAGE, "N/A");
      Blynk.virtualWrite(VPIN_CURRENT, "N/A");
      Blynk.virtualWrite(VPIN_TEMP, "N/A");
      Blynk.virtualWrite(VPIN_BAT_CHARGE, "N/A");
      Blynk.virtualWrite(VPIN_BAT_CAPACITY, "N/A");    
      /* motors */
      Blynk.virtualWrite(VPIN_LEFTMOTOR_CURR, "N/A");
      Blynk.virtualWrite(VPIN_RIGHTMOTOR_CURR, "N/A");
      Blynk.virtualWrite(VPIN_MAINBRUSH_CURR, "N/A");
      Blynk.virtualWrite(VPIN_SIDEBRUSH_CURR, "N/A");
      Blynk.virtualWrite(VPIN_STASIS_TOGGLING, "N/A" );
      Blynk.virtualWrite(VPIN_STASIS_DISABLED, "N/A" );
      /* robot running */
      Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "N/A");
      Blynk.virtualWrite(VPIN_ROBOT_OIMODE, "N/A");
    }
}

#define TIME_ROBOTSTOP  1 // time (counts) to treat as robot stop after no motor motion
#define TIME_MANUALCONTROL 50 // time (counts) that Robot should return Auto mode if there's no manual control

boolean isRobotMoving()
{
  if (leftMotorCurrent > 10 || leftMotorCurrent < -10) return true;
  if (rightMotorCurrent > 10 || rightMotorCurrent < -10) return true;
  return false;
}

/* update robot running */
void updateRobotState()
{
  // Skip if Robot is not controlled
  if (!isControlled) return;

  switch (stateRobot)
  {
    case 1: // auto mode
      if (isRobotMoving())
      {
        if (!isRunning)
        {
          isRunning = true;
          cntRobotStop = 0;
          Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Running");
          Blynk.notify("Robot is running!");
        }
        cntRobotStop = 0;
      }
      else
      {
        if (isRunning)
        {
          cntRobotStop++;
          if (cntRobotStop >= TIME_ROBOTSTOP) // no motion after 3sec -> robot stop
          {
            isRunning = false;
            cntRobotStop = 0;
            Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Staying");
            Blynk.notify("Robot is stopped!");
          }
        }
      }    
      break;
    case 2: // manual mode
      cntRobotNotDrive++;
      if (cntRobotNotDrive >= TIME_MANUALCONTROL)
      {
        stateRobot = 1;
        Serial.write(128);
        delay(10);
        cntRobotNotDrive = 0;
        isRunning = false;
        Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Staying");
      }    
    break;
  }
  
}

#define INTERVAL_GETSENSOR    500L
#define INTERVAL_SHOWSENSOR   1000L
#define INTERVAL_ROBOTRUNNING 500L

void setup() {
  // setup serial
  Serial.begin(115200, SERIAL_8N1);

  // put your setup code here, to run once:
  Blynk.begin(auth, ssid, pass, server, port);

  // setup Baudrate Change Pin
  pinMode(BRC_PIN, OUTPUT);
  digitalWrite(BRC_PIN, HIGH);

  /* startup not controlled */
  isControlled = false;
  isRunning = false;
  cntRobotStop = 0;
  stateRobot = 1; //Auto

  /* timeout count */
  tmoCntGetSensor = 0;

  // setup timers
  timerGetSensor.setInterval(INTERVAL_GETSENSOR, getSensorData);
  timerShowSensorData.setInterval(INTERVAL_SHOWSENSOR, showSensorData);
  timerRobotState.setInterval(INTERVAL_ROBOTRUNNING, updateRobotState);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  timerGetSensor.run();
  timerShowSensorData.run();
  timerRobotState.run();
}

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

BLYNK_WRITE(VPIN_STARTSTOP) //START
{
  int pinData = param.asInt();
  
  if (pinData == 1) // wake the robot up and start OI
  {
    // wake up Roomba if necessary
    wakeupRobot();
    // open Open Interface in Auto mode
    Serial.write(128);
    delay(100);

    isControlled = true;
    stateRobot = 1; //always Auto when start controlling
    Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Staying");
    
    // get initial sensor data
    getSensorData();
    showSensorData();  
    
  }
  else // stop OI
  {
    isControlled = false;

    // return Auto mode
    stateRobot = 1;
    Serial.write(128);
    delay(100);
    
    // close Open Interface
    Serial.write(173);
    delay(100);      
  }
}

BLYNK_WRITE(VPIN_CLEAN) //CLEAN
{
  int pinData = param.asInt();
  
  if (pinData == 1) // send CLEAN command
  {
    // change back to Auto mode if necessary
    if (stateRobot != 1)
    {
      stateRobot = 1;
      Serial.write(128);
      delay(10);      
    }
    
    // send CLEAN command
    Serial.write(135);
    delay(100);
  }
}

BLYNK_WRITE(VPIN_SPOT) //SPOT 
{
  int pinData = param.asInt();
  
  if (pinData == 1) // send SPOT command
  {
    // change back to Auto mode if necessary
    if (stateRobot != 1)
    {
      stateRobot = 1;
      Serial.write(128);
      delay(10);      
    }
        
    // send SPOT command
    Serial.write(134);
    delay(100);
  }
}

BLYNK_WRITE(VPIN_DOCK) //DOCK
{
  int pinData = param.asInt();
  
  if (pinData == 1) // send DOCK command
  {
    // change back to Auto mode if necessary
    if (stateRobot != 1)
    {
      stateRobot = 1;
      Serial.write(128);
      delay(10);      
    }
        
    // send SPOT command
    Serial.write(143);
    delay(100);
  }
}

void driveMotor(uint8_t mode)
{
  byte cmdStop[5] = {137, 0x00, 0x00, 0x00, 0x00}; // stop
  byte cmdForward[5] = {137, 0x00, 0x64, 0x00, 0x00}; // forward
  byte cmdBackward[5] = {137, 0xff, 0x9C, 0x00, 0x00}; // backward
  byte cmdRotateCW[5] = {137, 0x00, 0xC8, 0xff, 0xff}; // rotate cw
  byte cmdRotateCCW[5] = {137, 0x00, 0xC8, 0x00, 0x01}; // rotate ccw
  switch (mode)
  {
    case 0: // stop;
      Serial.write(cmdStop, sizeof(cmdStop));
      delay(10);
      isRunning = false;
      Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Staying");
      break;
    case 1: // forward;
      Serial.write(cmdForward, sizeof(cmdForward));
      delay(10);
      isRunning = true;
      Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Running");
      break;
    case 2: // backward;
      Serial.write(cmdBackward, sizeof(cmdBackward));
      delay(10);    
      isRunning = true;
      Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Running");
      break;
    case 3: // rotate cw;
      Serial.write(cmdRotateCW, sizeof(cmdRotateCW));
      delay(10);    
      isRunning = true;
      Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Running");
      break;
    case 4: // rotate ccw;
      Serial.write(cmdRotateCCW, sizeof(cmdRotateCCW));
      delay(10);    
      isRunning = true;
      Blynk.virtualWrite(VPIN_ROBOT_RUNNING, "Running");
      break;
    default:
      break;
  }
  //stop
  //Serial.write(cmdStop, sizeof(cmdStop));
  //delay(10);    
  
}

BLYNK_WRITE(VPIN_STOP) //STOP
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    // change back to Safe mode if necessary
    if (stateRobot != 2)
    {
      stateRobot = 2;
      Serial.write(131);
      delay(10);      
    }
    
    cntRobotNotDrive = 0;
        
    // send Drive command
    driveMotor(0);
  }
}


BLYNK_WRITE(VPIN_FORWARD) //FORWARD
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    // change back to Safe mode if necessary
    if (stateRobot != 2)
    {
      stateRobot = 2;
      Serial.write(131);
      delay(10);      
    }
    
    cntRobotNotDrive = 0;
        
    // send Drive command
    driveMotor(1);
  }
}

BLYNK_WRITE(VPIN_BACKWARD) //BACKWARD
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    // change back to Safe mode if necessary
    if (stateRobot != 2)
    {
      stateRobot = 2;
      Serial.write(131);
      delay(10);      
    }

    cntRobotNotDrive = 0;
        
    // send Drive command
    driveMotor(2);
  }
}

BLYNK_WRITE(VPIN_ROTATECW) //Rotate CW
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    // change back to Safe mode if necessary
    if (stateRobot != 2)
    {
      stateRobot = 2;
      Serial.write(131);
      delay(10);      
    }

    cntRobotNotDrive = 0;
        
    // send Drive command
    driveMotor(3);
  }
}



BLYNK_WRITE(VPIN_ROTATECCW) //Rotate CCW
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    // change back to Safe mode if necessary
    if (stateRobot != 2)
    {
      stateRobot = 2;
      Serial.write(131);
      delay(10);      
    }

    cntRobotNotDrive = 0;
        
    // send Drive command
    driveMotor(4);
  }
}

BLYNK_WRITE(VPIN_TEST) //Anything for test
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    getSensorData();
  }
}

BLYNK_WRITE(VPIN_CLEARLASTCMD)
{
  int pinData = param.asInt();
  
  if (pinData == 1)
  {
    clearLastCmd();
  }  
}
