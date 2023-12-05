// #include <Adafruit_BNO08x.h>

/****************************************************************
Code for euHy board

OnBoard Accelerometer
External capacitance Sensor
Serial Connected to ESP32 board

*****************************************************************/
#include <i2c_t3.h>
#include "bottle_config.h"

#define TS 10 //Every 10 samples (0.1s)
#define TIMERINT 60000000
#define ACCTHRESHOLD 0.25

//declare on-board accelerometer
MMA8653FC onBoardAcc;

void accInit();
void readAcc(bool serialDebug);
void calibratePitch();
void getFinalVolOffBoard(float minAngle);

float getDynamicAcc();
float getCapacitanceVolume();

String espMessage;

float fAccX;
float fAccY;
float fAccZ;

int16_t AccX;
int16_t AccY;
int16_t AccZ;

float calibrationConstantOn;
float angOnBoardMin;
float angOnBoard;
float angOnBoardInit;

float activity;
float timeLastActivity;
float finalVolOffBoard;
float finalVolOnBoard;

float fullCupVol;
float lastSipVol;
float lastVol;

float dynamicAcc;
float prevDynamicAcc;
float baselineDynamicAcc;

int sampleCount;
int nzc;

long int tActInit;
long int tActEnd;

bool isEvent;

float tremorFreq;

float vStart;
float vEnd;
float touch;

void addFloatToMessage(int idx, float data);

//different states the system can be in
enum State {INIT,
            WAIT_ACT, // monitors activity and changes state when threshold exceeded
            DATA_ACT,
            ACT_VERIFY, // gathers event data
            SEND_DAT // sends data to server
          }; // experiment of accelerometers and volume
State state;
State newState;

//measurements 
const int N = 120;

//timer
IntervalTimer secondTimer;

void timerOn();

void setup() {
  //Initialize Serial Communication
  USB_Serial.begin(115200);
  ESP_Serial.begin(115200);

  USB_Serial.println("Serial Started!");

  //wait for ESP to be connected
  while (!ESP_Serial)
    delay(10);

  USB_Serial.println("Initializing I2C Communication to MMA8653FC ");

  // I2C setup for Master mode, pins 18/19, internal pullups, 400kHz
  setupI2C();
  onBoardAcc.accInit();

  state = WAIT_ACT;

  activity = 0.0;
  sampleCount = 0;
  nzc = 0;

  calibratePitch();

  calibrationConstantOn = 90.0 - angOnBoardInit; // greater than 0
  angOnBoardMin = angOnBoardInit;

  USB_Serial.print("Initial angle on board: ");
  USB_Serial.println(angOnBoardMin);

  USB_Serial.print("Calibration constant on-board: ");
  USB_Serial.println(calibrationConstantOn);

  espMessage = "";

  fullCupVol = 400;
  lastVol = fullCupVol;

  baselineDynamicAcc = getDynamicAcc();

  delay(10);
}

void loop() {

  switch (state){
    case INIT:
      
      break;

    case WAIT_ACT:
    {
      //USB_Serial.println("WAIT_ACT");
      nzc = 0;

      readAcc(false);
      dynamicAcc = getDynamicAcc();
      activity += abs(dynamicAcc);
      sampleCount++;
      
      if(sampleCount == TS){
        sampleCount = 0;
        //USB_Serial.println(activity);
        if(activity > ACCTHRESHOLD){
          newState = DATA_ACT;
          tActInit = millis();
          prevDynamicAcc = dynamicAcc;
          vStart = getCapacitanceVolume();
        }
        else newState = WAIT_ACT;

        activity = 0;
      }     

      else newState = WAIT_ACT;

      touch = touchRead(CAP_S0); //may need to change cpacitance pin
      break;
    }
    
    case DATA_ACT:
    {
      //get readings
      readAcc(false);
      prevDynamicAcc = dynamicAcc;
      dynamicAcc = getDynamicAcc();
      activity += abs(dynamicAcc);
      sampleCount++;
      touch = touchRead(CAP_S0); //may need to change cpacitance pin

      if(sampleCount == TS){
        sampleCount = 0;
        // USB_Serial.println(activity);
        // USB_Serial.println(angOnBoard);
        
        if(activity < ACCTHRESHOLD){
          
          tActEnd = millis();
          timeLastActivity = ((float)tActEnd - (float)tActInit)/1000;
          activity = 0;

          if(timeLastActivity < 1.0){
            //not an event so disregard
            newState = WAIT_ACT;
          }

          else{
            newState = ACT_VERIFY;
            activity = 0;
            getFinalVolOnBoard(90 - angOnBoardMin + calibrationConstantOn);
            lastSipVol = lastVol - finalVolOnBoard;
            lastVol = finalVolOnBoard != 0 ? lastVol : fullCupVol;
            vEnd = getCapacitanceVolume();
          }

        }

        else{
          activity = 0;
        }

      }

      else newState = DATA_ACT;

      if((dynamicAcc > baselineDynamicAcc) && (prevDynamicAcc <= baselineDynamicAcc)) nzc++;
      if(angOnBoard < angOnBoardMin) angOnBoardMin = angOnBoard;

      //bound angle between 90 and 0
      if(angOnBoardMin > 90 - calibrationConstantOn) angOnBoardMin = 90.0;
      else if (angOnBoardMin < -calibrationConstantOn) angOnBoardMin = 0.0;
      break;
    }

  case ACT_VERIFY:
    readAcc(false);
    dynamicAcc = getDynamicAcc();
    activity += abs(dynamicAcc);
    sampleCount++;
    secondTimer.begin(timerOn, 1000);

    if(sampleCount == TS){
      sampleCount = 0;
      
      if(activity < ACCTHRESHOLD){
        activity = 0;
      }

      else if(!isEvent){
        newState = DATA_ACT;
        secondTimer.end();
      }

    }

    else newState = ACT_VERIFY;
    
    break;
  

  case SEND_DAT:
  {
    USB_Serial.println("SEND_ACT");
    USB_Serial.print("Time over threshold(s): ");
    USB_Serial.println(timeLastActivity);

    USB_Serial.print("NZC: ");
    USB_Serial.println(nzc);

    tremorFreq = (float)nzc/timeLastActivity;
    USB_Serial.print("Tremor Frequency: ");
    USB_Serial.println(tremorFreq);    
    
    ESP_Serial.print('t');
    ESP_Serial.flush();
    ESP_Serial.print(timeLastActivity);
    ESP_Serial.flush(); //wait for transmission to finish

    ESP_Serial.print('f');
    ESP_Serial.flush();
    ESP_Serial.print(tremorFreq);
    ESP_Serial.flush();

    ESP_Serial.print('v');
    ESP_Serial.flush();
    ESP_Serial.print(lastSipVol);
    ESP_Serial.flush(); //wait for transmission to finish


    ESP_Serial.print('s');
    ESP_Serial.flush();
    ESP_Serial.print(vStart);
    ESP_Serial.flush(); //wait for transmission to finish

    ESP_Serial.print('e');
    ESP_Serial.flush();
    ESP_Serial.print(vEnd);
    ESP_Serial.flush(); //wait for transmission to finish
    
    ESP_Serial.print('d');
    ESP_Serial.flush();
    ESP_Serial.print(vEnd - vStart);
    ESP_Serial.flush(); //wait for transmission to finish

    angOnBoardMin = angOnBoardInit;
    nzc = 0;
    newState = WAIT_ACT;
  }
} 
  

  delay(10);
  state = newState;
}

void readAcc(bool serialDebug) {
  Wire.beginTransmission(I2C_ADDRESS_ACC);  //= ST + (Device Adress+W(0)) + wait for ACK
  Wire.write(ctrl_reg_address);             // store the register to read in the buffer of the wire library
  Wire.endTransmission(I2C_NOSTOP, 1000);   // actually send the data on the bus -note: returns 0 if transmission OK-
  delayMicroseconds(2);                     //
  Wire.requestFrom(I2C_ADDRESS_ACC, 7);     // read a number of byte and store them in wire.read (note: by nature, this is called an "auto-increment register adress")

  Wire.read();  // discard first byte (status)

  // read six bytes
  // note:  2G is 32768, -2G is -32768
  AccX = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2);  // MSB first
  AccY = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2);  // MSB first
  AccZ = ((int16_t)(((unsigned char)Wire.read() << 8) | (unsigned char)Wire.read()) * 1000) / (32768 / 2);  // MSB first
  fAccX = (float)(AccX) / (1000);
  fAccY = (float)(AccY) / (1000);
  fAccZ = (float)(AccZ) / (1000);

  angOnBoard = atan(fAccZ/sqrt(fAccX*fAccX+fAccY*fAccY))*180/PI;

  //console log - defaults to false
  if (serialDebug) {
    USB_Serial.print("X: ");
    USB_Serial.print(fAccX);
    USB_Serial.print(" Y: ");
    USB_Serial.print(fAccY);
    USB_Serial.print(" Z: ");
    USB_Serial.println(fAccZ);
  }
}

//helper function to verify correct communication between ESP and Teensy
void debugSerial() {
  if (Serial1.available() > 0) {
    USB_Serial.print("Data is available: ");
    USB_Serial.println(ESP_Serial.readStringUntil('\n'));
    ESP_Serial.flush();
  } 
}

float getDynamicAcc(){
    return sqrt(fAccX*fAccX + fAccY*fAccY + fAccZ*fAccZ) - 1;
}

void calibratePitch(){
  readAcc(false);
  angOnBoardInit = angOnBoard;
  
}

void getFinalVolOnBoard(float minAngle){
  float intersect = 61.0799;
  USB_Serial.print("Min angle");
  USB_Serial.println(minAngle);
  if(minAngle < intersect){
    finalVolOnBoard = -0.03106*minAngle*minAngle - 1.018*minAngle + 463;
  }
  else{
    finalVolOnBoard = 0.004996*pow(minAngle, 3) - 1.13*pow(minAngle, 2) + 74.3*minAngle - 1176;
  }

  finalVolOnBoard = finalVolOnBoard < 0 ? 0.0 : finalVolOnBoard;
}

void timerOn(){
  isEvent = true;
  newState = SEND_DAT;

  secondTimer.end();
}

float getCapacitanceVolume(){
  return A_CAP*touch + B_CAP;
}
