#include <Arduino.h>
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#include <WiFiClientSecure.h>
//#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <secrets.h>
#include <bsec.h>

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt"
};


WiFiClient wificlient;
//MQTTClient mqttclient(256);
AsyncMqttClient amqttclient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t bmegetDataTimer;

Bsec bme_dev;

String MACasString() {
  byte mac[6];
  WiFi.macAddress(mac);
  String ID = "";
  for (int i = 0; i < 6;i++) {
    ID += String(mac[i],HEX);
    if (i != 5) ID+= ":";
  }
  return ID;
}

void errLeds(void)
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(100);
}

// Helper function definitions
void checkBmeSensorStatus(void)
{
  String output="";
  if (bme_dev.status != BSEC_OK) {
    if (bme_dev.status < BSEC_OK) {
      output = "BSEC error code : " + String(bme_dev.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(bme_dev.status);
      Serial.println(output);
    }
  }

  if (bme_dev.bme680Status != BME680_OK) {
    if (bme_dev.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(bme_dev.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(bme_dev.bme680Status);
      Serial.println(output);
    }
  }
}


void connectToMqtt() {
  amqttclient.connect();
}

void publishMQTT() {
  if (bme_dev.run()) {
    StaticJsonDocument<128> doc;
    char output[256];
    doc["sensor"] = "BME680";
    doc["time"] = millis();

    JsonObject data = doc.createNestedObject("data");
    data["temperature"] = bme_dev.temperature;
    data["humidity"] = bme_dev.humidity;
    data["pressure"] = bme_dev.pressure;
    data["iaq"] = bme_dev.iaq;


    size_t n= serializeJson(doc, output);
    amqttclient.publish("esp32/pub",2, true, output,n);
    Serial.println(String(output));
  } else {
    checkBmeSensorStatus();
  }
}

void wifiEventHandler(WiFiEvent_t wifi_event,WiFiEventInfo_t wifi_event_info) {
  switch (wifi_event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      //Serial.println(String(millis())+" - Connected to AP "+WIFI_SSID);
      //Serial.println(wifi_event);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      //Serial.println(String(millis())+" - Disconnected from WiFi AP. Trying to reconnect...");
      Serial.println("Disconnected from WiFi");
      xTimerStop(mqttReconnectTimer, 0);
      WiFi.disconnect();
      WiFi.reconnect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to wifi");
      //Serial.print(String(millis())+" - IP : ");
      //Serial.println(WiFi.localIP());
      //Serial.println(String(millis())+" - RSSI ["+WiFi.RSSI()+" dB]");
      connectToMqtt();
      break;
    default:
      //Serial.println(wifi_event);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  //uint16_t packetIdSub = amqttclient.subscribe("esp32/sub", 2);
  //uint16_t packetIdPub1 = amqttclient.publish("esp32/connection", 1, true, "connected");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT");
  //Serial.println(String(millis())+" - Disconnected from MQTT broker : "+String(int8_t(reason)));
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  //Serial.println(qos);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total){
  Serial.println("incoming message : "+String(topic)+" - "+payload);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() { 
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  
  WiFi.onEvent(wifiEventHandler);

  amqttclient.onConnect(onMqttConnect);
  amqttclient.onDisconnect(onMqttDisconnect);
  amqttclient.onSubscribe(onMqttSubscribe);
  amqttclient.onPublish(onMqttPublish);
  amqttclient.onMessage(onMqttMessage);
  amqttclient.setServer(BROKER_ADDR,BROKER_PORT);
  amqttclient.setKeepAlive(90);

  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  //mqttclient.onMessage(incomingMessageHandler);
  //mqttclient.begin(BROKER_ADDR,BROKER_PORT,wificlient);
  //mqttclient.setKeepAlive(3500);

  // while (!mqttclient.connect("ESP32-192-168-1-36")) { //,BROKER_USER,BROKER_PWD)) {
  //   Serial.print(".");
  //   delay(100);
  // }
  // Serial.println("connected to MQTT broker");
  
  // if(!mqttclient.connected()){
  //   Serial.println("Timeout!");
  //   return;
  // }

  // mqttclient.subscribe("esp32/sub");
  
  Wire.begin();
  //bmegetDataTimer = xTimerCreate("bmeTimer", pdMS_TO_TICKS(6000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(publishMQTT));
  
  bme_dev.begin(BME680_I2C_ADDR_PRIMARY,Wire);

  //bme_dev.setConfig(bsec_config_iaq);

  bsec_virtual_sensor_t sensorList[4] = {
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_IAQ
  };
  bme_dev.updateSubscription(sensorList,4,BSEC_SAMPLE_RATE_ULP);
  checkBmeSensorStatus();

  // if (xTimerStart(bmegetDataTimer,0) != pdPASS) {
  //   Serial.println("Erreur au démarrage du timer");
  // }
}

void loop() {

  // if(!mqttclient.connected()){
  //   Serial.println(String(millis()));
  //   Serial.println("Timeout Mqtt!");
  //   while(true) {};
  // }


  // if (bme_dev.run()) {
     publishMQTT();
  // } else {
  //   checkBmeSensorStatus();
  // }
}

