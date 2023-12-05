#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "time.h"

#define USERNAME "emil"
#define PASSWORD "emil"
#define MQTT_SERVER "136.53.99.251:8123"

#define SSID "WIFI_ID"
#define PASS "WIFI_PASS"

#define CUP_ID "T0001"

#define LED 2