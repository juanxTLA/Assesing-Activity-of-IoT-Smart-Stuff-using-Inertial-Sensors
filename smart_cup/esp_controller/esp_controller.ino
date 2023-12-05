#include "config.h"

WiFiClient CLIENT;
PubSubClient client(CLIENT);

float timeEvent;
float frequency;
float vStart;
float vEnd;
float volumeAcc;
float volumeCap;

bool newData;

char option;

String JSONmessage;

const long gmtOffSet = -6 * 60 * 60;

void connectToWifi();
void transmitData();
void clientConnect();
void reconnect();

enum State { INIT,        //get data from ntp server and sent it to Teensy
             WAIT_DATA,   //wait for data from Teensy
             UPLOAD_DATA  //upload data
};
State state;
State newState;

struct tm timeInfo;

void setup() {
  // // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  connectToWifi();
  client.setServer(MQTT_SERVER, 1883);


  while (!client.connected()) {
    clientConnect();
  }

  client.loop();

  option = '\0';

  newData = false;

  //empty buffer
  while (Serial.available() > 0) {
    Serial.read();
  }

  state = INIT;
}

void loop() {

  //access NTP server
  switch (state) {
    case INIT:
    {
      // configTime(gmtOffSet, gmtOffSet, NTPSERVER);
      // while (!getLocalTime(&timeInfo)) {}
      // Serial.println(&timeInfo, "%A, %B %d %Y %H:%M:%S");

      newState = WAIT_DATA;
      break;
    }

    case WAIT_DATA:
    {
      while(Serial.available() > 0){
        option = Serial.read();

        if(option == 't'){
          timeEvent = Serial.parseFloat();
        }
        else if(option == 'f'){
          frequency = Serial.parseFloat();
        }
        else if(option == 'v'){
          volumeAcc = Serial.parseFloat();
        }
        else if(option == 's'){
          vStart = Serial.parseFloat();
        }
        else if(option == 'e'){
          vEnd = Serial.parseFloat();
        }
        else if(option == 'd'){
          volumeCap = Serial.parseFloat();
        }

        newData = true;
      }

      newState = newData ? UPLOAD_DATA : WAIT_DATA;
      break;
    }

    case UPLOAD_DATA:
    {
      newData = false;
      if(!client.connected()){
        reconnect();
      }

      transmitData();
      newState = WAIT_DATA;
      break;
    }  
  }

  state = newState;
  delay(100);
}

void connectToWifi() {
  WiFi.mode(WIFI_STA);
  //Serial.print("Connecting to ");

  WiFi.begin(SSID, PASS);
  Serial.println(SSID);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  //Serial.println("Connected");
}


void clientConnect() {
  while (!client.connected()) {
    String clientId = "smartcup";

    if (client.connect(clientId.c_str(), USERNAME, PASSWORD)) {
      client.publish("SMARTCUP/status", "booted");
      client.subscribe("inTopic");
    } else {
      delay(100);
      Serial.println("Unable to connect");
    }
  }
}

void transmitData() {
  char msg[200];
  StaticJsonDocument<200> doc;

  doc["id"] = CUP_ID;
  doc["start_time"] = timeEvent;
  doc["frequency"] = frequency;
  doc["vStart"] = vStart;
  doc["vEnd"] = vEnd;
  doc["dv"] = volumeAcc;
  doc["dvc"] = volumeCap;

  JSONmessage = "";

  serializeJson(doc, JSONmessage);
  JSONmessage.toCharArray(msg, JSONmessage.length() + 1);

  client.publish("SMARTCUP/values", msg, false);  //need retain flag to true
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "smartcup";

    if (client.connect(clientId.c_str(), USERNAME, PASSWORD)) {
      Serial.println("Reconnected");
      client.publish("SMARTCUP/status", "reconnected");
    } else {
      delay(100);
      Serial.println("Unable to connect");
    }
  }
}