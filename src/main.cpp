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

/*******************************************************
HELPERS FUNCTIONS
********************************************************/

/*!
 * @brief Get the Mac address of the ESP network interface
 *
 * This function returns a human readable of the MAC address.
 *
 * @param
 * @return [String] the MAC addresd
 *
 *
  \code{.c}
    // Example // 
    MacasString();

  \endcode
 */
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

/*!
 * @brief dead end function when blocking error triggered.
 *
 * This function is to be called whenever an blocking error occurs (wiring, network,...)
 * in order to let know an exterior intervention is required. It is an endless loop with blinking leds
 *
 * @param [in]      led_pin       the pin number where the led to be flashing. Default to 2 (ESP32)
 * 
 * @return None
 * 
 * \code{.c}
 *  errLeds();
 * \endcode
 * 
*/
void errLeds(int led_pin = 2)
{
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  delay(100);
  digitalWrite(led_pin, LOW);
  delay(100);
}

/*!
 * @brief global checker function for the BME680 sensor.
 *
 * This function triggers the error handling function depending of the BSEC device status.
 * See  type bsec_retrun_library_t in bsec_datatype.h for return codes.
 *
 * @param
 * @return None
 * 
 * \code{.c}
 *  checkBmeSensorStatus();
 * \endcode
 * 
*/
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

/*!
 * @brief Publish data from BME sensor over MQTT if ready
 *
 * This function creates a JSON Document to wrap the data collected from sensor (with run() function),
 * and publish to MQTT broker
 *
 * @param
 * @return None
 * 
 * \code{.c}
 *  publishMQTT();
 * \endcode
 * 
*/
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


/*!
 * @brief handler function for main WiFi events
 *
 * The function manages to reconnect WiFI if the status is disconnected,
 * and try to connect to the MQTT broker if an IP address is got.
 *
 * @param [in] wifi_event         the WiFi event which triggered the handler. See WiFiType.h for status return list.
 * @param [in] wifi_event_info    the info from the wifi event
 * 
 * @return None
 * 
 * \code{.c}
 *  WiFi.onEvent(wifiEventHandler);
 * \endcode
 * 
*/
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


/*!
 * @brief Callback function associated with MQTT connexion state.
 *
 * Do noting actually
 *
 * @param [out]    sessionPresent
 * 
 * @return None
 * 
 * \code{.c}
 *  amqttclient.onConnect(onMqttConnect);
 * \endcode
 * 
*/
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT");
  //uint16_t packetIdSub = amqttclient.subscribe("esp32/sub", 2);
  //uint16_t packetIdPub1 = amqttclient.publish("esp32/connection", 1, true, "connected");
}

/*!
 * @brief Callback function associated with MQTT disconnection state.
 *
 * start a timer for MQTT connection
 *
 * @param [out]    reason      the reason why the MQTT connexion is disconnected
 * 
 * @return None
 * 
 * \code{.c}
 *  amqttclient.onConnect(onMqttDisconnect);
 * \endcode
 * 
*/
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT");
  //Serial.println(String(millis())+" - Disconnected from MQTT broker : "+String(int8_t(reason)));
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*!
 * @brief Callback function associated with MQTT subscribe state.
 *
 * display acknowledged response.
 *
 * @param [out]     packetId      Id of the return packet
 * @param [out]     qos           index of Quality of Service
 * 
 * @return None
 * 
 * \code{.c}
 *  amqttclient.onConnect(onMqttSubscribe);
 * \endcode
 * 
*/
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  //Serial.println(qos);
}

/*!
 * @brief Callback function associated with MQTT incoming message state.
 *
 * deals with the message received from broker
 *
 * @param [out]     topic       topic the client subscribes to
 * @param [out]     payload     the message content
 * @param [out]     properties  
 * @param [out]     len     
 * @param [out]     index     
 * @param [out]     total     
 * 
 * 
 * @return None
 * 
 * \code{.c}
 *  amqttclient.onConnect(onMqttMessage);
 * \endcode
 * 
*/
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total){
  Serial.println("incoming message : "+String(topic)+" - "+payload);
}

/*!
 * @brief Callback function associated with MQTT publish state.
 *
 * display acknowledged response after publishing a topic message
 *
 * @param [out]     packetId      Id of the return packet
 * 
 * @return None
 * 
 * \code{.c}
 *  amqttclient.onConnect(onMqttPublish);
 * \endcode
 * 
*/
void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() { 
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  //------------------------------------------
  //WiFi and MQTT settings
  //------------------------------------------
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

  //------------------------------------------
  //BME680 settings
  //------------------------------------------
  Wire.begin();
  bme_dev.begin(BME680_I2C_ADDR_PRIMARY,Wire);

  bme_dev.setConfig(bsec_config_iaq);

  bsec_virtual_sensor_t sensorList[4] = {
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_IAQ
  };
  bme_dev.updateSubscription(sensorList,4,BSEC_SAMPLE_RATE_ULP);
  checkBmeSensorStatus();
}

void loop() {

     publishMQTT();
     
}

