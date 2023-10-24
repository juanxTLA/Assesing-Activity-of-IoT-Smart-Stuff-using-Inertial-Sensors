// #include <Adafruit_BNO08x.h>

/****************************************************************
Code for euHy board

OnBoard Accelerometer
External 9-axis accelerometer
External capacitance Sensor
Serial Connected to ESP32 board

*****************************************************************/
#include <i2c_t3.h>
#include "Adafruit_BNO08x_RVC.h"
#include "bottle_config.h"

#define CAP_S0 22
#define CAP_S1 23

#define PI 3.14

#define TS 100 //Hz
#define TIMERINT 60000000
#define ACCTHRESHOLD 1.7
#define MSG_LENGTH_ACC 38

//declare Adafruit aceclerometer
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
BNO08x_RVC_Data rvcData;

//declare on-board accelerometer
MMA8653FC onBoardAcc;

void accInit();
void readAcc(bool serialDebug);
void readImu();
void calibratePitch();
void getFinalVolOffBoard(float minAngle);

void debugSerial();

float getDynamicAcc(int a);


String espMessage;

float fAccX;
float fAccY;
float fAccZ;

int16_t AccX;
int16_t AccY;
int16_t AccZ;

float yaw;
float pitch;
float roll;

float accXRvc;
float accYRvc;
float accZRvc;

float minPitch;
float pitchInit; //hold it for when variable resets
float calibrationConstant;

float minAngleOnBoard;
float angOnBoard;
float angOnBoardInit;

float activity;
float timeLastActivity;
float finalVolOffBoard;
float finalVolOnBoard;

float fullCupVol;
float lastSipVol;
float lastVol;

byte messageAccSerial[MSG_LENGTH_ACC];

int sampleCount;

long int tActInit;
long int tActEnd;

bool isEvent;

void addFloatToMessage(int idx, float data);

//different states the system can be in
enum State {IDLE,
            ESP_SERIAL_DEBUG, // debug serial communication with ESP board
            IMU_SERIAL_APP,
            RUN
          }; // experiment of accelerometers and volume
State state;
char newState;

//measurements 
const int N = 120;

//timer
IntervalTimer myTimer;

void setup() {
  //Initialize Serial Communication
  /*
    Serial  -> Debugging console
    Serial1 -> ESP32 communication
    Serial3 -> Adafruit Accelerometer
  */
  USB_Serial.begin(115200);
  ESP_Serial.begin(115200);
  IMU_Serial.begin(115200);

  USB_Serial.println("Serial Started!");

  //wait for ESP to be connected
  while (!ESP_Serial)
    delay(10);

  while (!IMU_Serial)
    delay(10);


  USB_Serial.println("Initializing I2C Communication to MMA8653FC ");

  if (!rvc.begin(&IMU_Serial)) {  // connect to the sensor over hardware serial
    Serial.println("Could not find BNO08x!");
    while (1)
      delay(10);
  }

  // I2C setup for Master mode, pins 18/19, internal pullups, 400kHz
  setupI2C();
  onBoardAcc.accInit();

  //prepare messages for serial app
  messageAccSerial[0] = 0x55;

  state = RUN;

  activity = 0.0;
  sampleCount = 0;
  calibratePitch();

  calibrationConstant = 90.0 - angOnBoardInit; // greater than 0
  USB_Serial.print("Calib: ");
  USB_Serial.println(calibrationConstant);
  minPitch = pitchInit;
  minAngleOnBoard = angOnBoardInit;

  USB_Serial.print("Initial angle: ");
  USB_Serial.println(minPitch);

  USB_Serial.print("Initial angle on board: ");
  USB_Serial.println(minAngleOnBoard);

  espMessage = "";

  finalVolOffBoard = 0.0;
  fullCupVol = 400;
  lastSipVol = 0;

  delay(10);
}

void loop() {

  if(USB_Serial.available() > 0){
    newState = USB_Serial.read();
    while(USB_Serial.available() > 0)
    USB_Serial.read();
    
  }

  else{
    if(newState == 'i'){
      state = IDLE;
      USB_Serial.println("Entering IDLE Mode");
    }

    else if(newState == 'r'){
      state = RUN;
      USB_Serial.println("Entering RUN state");
    }

    else if(newState == 's'){
      state = ESP_SERIAL_DEBUG;
      USB_Serial.println("Entering SERIAL DEBUG state");
    }

    else if(newState == 'a'){
      state = IMU_SERIAL_APP;
      USB_Serial.println("Entering IMU SERIAL APP state");
    }
    newState = '\0';
  }
   
  switch (state){
    case IDLE:
      
      break;

    case ESP_SERIAL_DEBUG:
    {
      debugSerial();
      break;
    }
    
    case IMU_SERIAL_APP:
    {
      readAcc(false); // get inertial measurement of on board sensor
      readImu();

      float dataArray[9] = {
        fAccX,
        fAccY,
        fAccZ,
        yaw,
        pitch,
        roll,
        accXRvc,
        accYRvc,
        accZRvc
      };
      int dataIdx = 0;

      //prepare message for UAH serial App
      for(int i = 1; i < MSG_LENGTH_ACC - 1; i += 4){
        addFloatToMessage(i, dataArray[dataIdx]);
        dataIdx++;
      }

      //checksum
      messageAccSerial[MSG_LENGTH_ACC - 1] = messageAccSerial[1];
      for(int i = 2; i < MSG_LENGTH_ACC - 1; i++){
        messageAccSerial[MSG_LENGTH_ACC - 1] = (byte)(messageAccSerial[MSG_LENGTH_ACC - 1] + messageAccSerial[i]);
      }

      USB_Serial.write(messageAccSerial, MSG_LENGTH_ACC);
      //delay(20);
      break;
  }
    
    case RUN:
    {
      //get readings
      readAcc(false);
      //readImu();

      if(sampleCount == TS){
        sampleCount = 0;
        USB_Serial.println(activity);
        USB_Serial.println(pitch);
        
        if(activity < ACCTHRESHOLD){
          
          //if it was an event then process information and send to ESP
          if(isEvent){
            tActEnd = millis();
            isEvent = false;
            timeLastActivity = ((float)tActEnd - (float)tActInit)/1000;

            //event only happens if the activity is more than 2 seconds
            if(timeLastActivity > 2.00){
              USB_Serial.print("Time over threshold(ms): ");
              USB_Serial.println(timeLastActivity);
              USB_Serial.print("Min Angle (deg): ");
              USB_Serial.println(minAngleOnBoard);
              USB_Serial.print("VolFinal: ");
              

              //getFinalVolOffBoard(pitchInit - minPitch + calibrationConstant);
              //USB_Serial.println(finalVolOffBoard);
              getFinalVolOnBoard(angOnBoardInit - minAngleOnBoard +calibrationConstant);
              USB_Serial.println(finalVolOnBoard);

              lastSipVol = lastVol - finalVolOnBoard;
              
              ESP_Serial.print('t');
              ESP_Serial.flush();
              ESP_Serial.print(timeLastActivity);
              ESP_Serial.flush(); //wait for transmission to finish

              ESP_Serial.print('d');
              ESP_Serial.flush();
              ESP_Serial.print(minPitch);
              ESP_Serial.flush();

              ESP_Serial.print('v');
              ESP_Serial.flush();
              ESP_Serial.print(lastSipVol);
              ESP_Serial.flush(); //wait for transmission to finish
              
              minPitch = pitchInit;
              minAngleOnBoard = angOnBoardInit;
              lastVol = finalVolOnBoard != 0 ? lastVol : fullCupVol;
            }
          }
        }
        else{
          if(!isEvent) tActInit = millis();
          
          isEvent = true;
        }

        activity = 0;
      }

      else{
        activity += getDynamicAcc(1);
        sampleCount++;
      }

      //if(pitch < minPitch & minPitch > -15) minPitch = pitch;
      if(angOnBoard < minAngleOnBoard) minAngleOnBoard = angOnBoard;
    }
  }

  delay(10);
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

//function to turn float into 4 byte array and append it to array
void addFloatToMessage(int idx, float data){
  byte *byte2Float = (byte *) &data;
  messageAccSerial[idx]   = byte2Float[0];
  messageAccSerial[idx+1] = byte2Float[1];
  messageAccSerial[idx+2] = byte2Float[2];
  messageAccSerial[idx+3] = byte2Float[3];
}

void readImu(){
    while(!rvc.read(&rvcData)) {};

    yaw   = rvcData.yaw;
    pitch = rvcData.pitch;
    roll  = rvcData.roll;

    accXRvc = rvcData.x_accel/9.81;
    accYRvc = rvcData.y_accel/9.81;
    accZRvc = rvcData.z_accel/9.81;
}

float getDynamicAcc(int a){
  if(a == 1){
    return abs(sqrt(fAccX*fAccX + fAccY*fAccY + fAccZ*fAccZ) - 1);
  }

  else{
    return abs(sqrt(accXRvc*accXRvc + accYRvc*accYRvc + accZRvc*accZRvc) - 1);
  }
}

void calibratePitch(){
  readImu();
  pitchInit = pitch;

  readAcc(false);
  angOnBoardInit = angOnBoard;
  
}

void getFinalVolOffBoard(float minAngle){
  float intersect = 62.6072;
  if(minAngle < intersect){
    finalVolOffBoard = -0.02626*minAngle*minAngle - 1.039*minAngle + 466.1;
  }
  else{
    finalVolOffBoard = 0.005094*pow(minAngle, 3) - 1.205*pow(minAngle, 2) + 82.79*minAngle - 1412;
  }

  finalVolOffBoard = finalVolOffBoard < -calibrationConstant ? 0.0 : finalVolOffBoard;
}

void getFinalVolOnBoard(float minAngle){
  float intersect = 61.0799;
  if(minAngle < intersect){
    finalVolOnBoard = -0.03106*minAngle*minAngle - 1.018*minAngle + 463;
  }
  else{
    finalVolOnBoard = 0.004996*pow(minAngle, 3) - 1.13*pow(minAngle, 2) + 74.3*minAngle - 1176;
  }

  finalVolOnBoard = finalVolOnBoard < -calibrationConstant ? 0.0 : finalVolOnBoard;
}
