#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

#define LED 2

const char *MQTT_SERVER = "juantarrat.duckdns.org";
const char *SSID = "MyWifi";
const char *PASS = "406WarnerSt!";
//const char *PASS = "Go Chargers!";

WiFiClient CLIENT;
PubSubClient client(CLIENT);

float timeEvent;
float volume;
float angle;

bool newData;

char option;

String JSONmessage;

HardwareSerial MySerial0(0);

void connectToWifi();
void transmitData();
void clientConnect();
void reconnect();

void setup() {
  // // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Serial begun");
  // connectToWifi();
  // client.setServer(MQTT_SERVER, 1883);

  MySerial0.begin(115200, SERIAL_8N1, 6, 7);

  // while(!client.connected()){
  //   clientConnect();
  // }

  // client.loop();

  // timeEvent = 0;
  // volume = 0.0;
  // angle = 0.0;

  // option = '\0';

  // newData = false;
}

void loop() {
  
  // //first we process the data from Teensy
  // while(MySerial0.available() > 0){
    
  //   option = MySerial0.read();

  //   Serial.print("Option");
  //   Serial.println(option);
    
  //   if(option == 't'){
  //     timeEvent = MySerial0.parseFloat();
  //   }

  //   else if(option == 'd'){
  //     angle = MySerial0.parseFloat();
  //   }

  //   else if(option == 'v'){
  //     volume = MySerial0.parseFloat();
  //   }


  //   newData = true;
  // }

  // MySerial0.flush();

  // if(newData){
  //   if(!client.connected()){
  //     reconnect();
  //   }

  //   client.loop();
    
  //   byte *byte2Float = (byte *) &volume;

  //   transmitData();
  //   client.publish("SMARTCUP/volume", byte2Float, true); //need retain flag to true
  //   delay(3000);
  //   Serial.println("done");

  //   newData = false;
  // }
  MySerial0.println("Hello");
  MySerial0.flush();
  delay(1000);
  Serial.println("now");
}

void connectToWifi(){
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");

  WiFi.begin(SSID, PASS);
  Serial.println(SSID);

  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }

  Serial.println("Connected");
}


void clientConnect(){
  while(!client.connected()){
    String clientId = "smartcup";

    if(client.connect(clientId.c_str(), "juantarrat", "juantarrat")){
      client.publish("SMARTCUP/status", "booted");
      client.subscribe("inTopic");
    }
    else{
      delay(100);
      Serial.println("Unable to connect");
    }
  }
}

void transmitData(){
    char msg[200];
    StaticJsonDocument<200> doc;
  
    doc["time"] = timeEvent;
    doc["angle"] = angle;

    JSONmessage = "";

    serializeJson(doc, JSONmessage);
    JSONmessage.toCharArray(msg, JSONmessage.length()+1);

    client.publish("SMARTCUP/values", msg, true); //need retain flag to true
}

void reconnect(){
  while(!client.connected()){
    String clientId = "smartcup";

    if(client.connect(clientId.c_str(), "juantarrat", "juantarrat")){
      Serial.println("Reconnected");
      client.publish("SMARTCUP/status", "reconnected");
    }
    else{
      delay(100);
      Serial.println("Unable to connect");
    }
  }
}