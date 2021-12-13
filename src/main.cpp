#include <Arduino.h>
#include <EEPROM.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <MQTTClient.h>
#include <ArduinoJson.h>

#include <bsec.h>

// #ifdef ASYNC_TCP_SSL_ENABLED //doesn't work with ESP32
// #define MQTT_SECURE true
// #define MQTT_SERVER_FINGERPRINT {0x08, 0xf8, 0x95, 0xdc, 0x66, 0x4e, 0xa9, 0x0c, 0x67, 0xc7, 0x41, 0xad, 0x4f, 0x69, 0xd9, 0xda, 0x8a, 0x20, 0x4b, 0xf2}
// #define MQTT_PORT 8883
// #else
// #define MQTT_PORT 1883
// #endif
//#ifdef LOCAL_MQTT_BROKER
//#include <secrets_local.h>
//#else
#include <secrets.h>
//#endif

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_300s_28d/bsec_iaq.txt"
};
const uint32_t STATE_SAVE_PERIOD=(1 * 60 * 60 * 1000); // each 6 hours or 4 times a day

// Wifi & MQTT
#if TSL_ENABLED == 0
  WiFiClient wificlient;
#else
  //WiFiClientSecure netClientSecure = WiFiClientSecure();
#endif
WiFiClientSecure netClientSecure = WiFiClientSecure();
uint8_t resetCounter=0; 
//AsyncMqttClient amqttclient;
MQTTClient mqttClient = MQTTClient(512);
//TimerHandle_t mqttReconnectTimer;

// BME680
Bsec bme_dev;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE]= {0};
uint16_t stateUpdateCounter = 0;
uint32_t lastTime = 0;

/* #region Helpers */
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
  log_v(" MAC Address is : %s",ID);
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
 \code{.c}
  errLeds();
 \endcode
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
 \code{.c}
  checkBmeSensorStatus();
 \endcode
*/
void checkBmeSensorStatus(void)
{
  String output="";
  if (bme_dev.status != BSEC_OK) {
    if (bme_dev.status < BSEC_OK) {
      log_e(" BSEC error code : %d",bme_dev.status);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      log_w("BSEC warning code : %d",bme_dev.status);
    }
  }

  if (bme_dev.bme680Status != BME680_OK) {
    if (bme_dev.bme680Status < BME680_OK) {
      log_e(" BME680 error code : %d",bme_dev.status);
      // output = "BME680 error code : " + String(bme_dev.bme680Status);
      // Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      log_w(" BME680 error code : %d",bme_dev.status);
      // output = "BME680 warning code : " + String(bme_dev.bme680Status);
      // Serial.println(output);
    }
  }
}

void connectToMqtt() {
  while(!mqttClient.connect(THINGNAME)) {delay(10000);};
  log_d("MQTT connected %d",mqttClient.returnCode());
}

/*!
 * @brief Check if state is saved in EEPROM and load it
 *
 * This function helps the sensor to get a recent state to start with, 
 * speeding up trimming and calibrating virtual outputs
 * 
 * @param
 * @return void
 * 
 \code{.c}
  loadState();
 \endcode 
*/
void loadState() {
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    log_i("Reading state from EEPROM : ");
    //Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      log_v("Byte [%d] : %#x",i,bsecState[i]);
    }
    
    bme_dev.setState(bsecState);
    checkBmeSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    log_i("Erasing EEPROM");
    //Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

/*!
 * @brief Save BSEC state in EEPROM
 *
 * If time's up, use EEPROM to save state. The save rate is defined in global const STATE_SAVE_PERIOD (in ms)
 * 
 * @param
 * @return void
 * 
 \code{.c}
  saveState();
 \endcode 
*/
void saveState() {
  bool update = false;
  if (stateUpdateCounter == 0) {
    /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
    if (bme_dev.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    bme_dev.getState(bsecState);
    checkBmeSensorStatus();

    log_i("Writing state to EEPROM");
    //Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
      log_v("byte [%d] : %#x",i,bsecState[i]);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
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
 \code{.c}
  publishMQTT();
 \endcode
  
*/
void publishMQTT() {
  // while(!mqttClient.connected()) {
  //   log_w("MQTT disconnected %d",mqttClient.lastError());
  //   connectToMqtt();
  // }

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
    if (!mqttClient.connected()) {
      if (mqttClient.publish("weatherstation/pub",output,n,true,1)) {
        log_i("Publish : %s",output);
      } else {
        log_e("Publish unsuccessful : %d",mqttClient.lastError());
      }
    } else {
        if (WiFi.isConnected()) {
          //xTimerStart(mqttReconnectTimer, 0);
          connectToMqtt();
        }
    }
    
    //Serial.println(String(output));
    saveState();
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
 \code{.c}
   WiFi.onEvent(wifiEventHandler);
 \endcode
  
*/
void wifiEventHandler(WiFiEvent_t wifi_event,WiFiEventInfo_t wifi_event_info) {
  switch (wifi_event) {
    case SYSTEM_EVENT_STA_DISCONNECTED: // deconnexion from AP
      // Try 5 times to reconnect to WiFi. Reset the system otherwise.
      if (resetCounter < 5) {
        resetCounter++;
        if (mqttClient.connected()) {mqttClient.disconnect();};
        //xTimerStop(mqttReconnectTimer, 0);
        WiFi.disconnect();
        WiFi.reconnect();
      }
      else {
        ESP.restart();
      }
      break;
    case SYSTEM_EVENT_STA_GOT_IP: // connected and got an IP
      resetCounter=0;
      //connectToMqtt();
      break;
    default:
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
 \code{.c}
  amqttclient.onConnect(onMqttConnect);
 \endcode
 
 */
// void onMqttConnect(bool sessionPresent) {
//   //Serial.println("Connected to MQTT");
//   log_i("Connected");
//   //uint16_t packetIdSub = amqttclient.subscribe("esp32/sub", 2);
//   //uint16_t packetIdPub1 = amqttclient.publish("esp32/connection", 1, true, "connected");
// }

/*!
 * @brief Callback function associated with MQTT disconnection state.
 *
 * start a timer for MQTT connection
 *
 * @param [out]    reason      the reason why the MQTT connexion is disconnected
 * 
 * @return None
 * 
 \code{.c}
  amqttclient.onConnect(onMqttDisconnect);
 \endcode
 
*/
// void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
//   //Serial.println("Disconnected from MQTT");
//   log_w("Disconnected");
//   //Serial.println(String(millis())+" - Disconnected from MQTT broker : "+String(int8_t(reason)));
//   if (WiFi.isConnected()) {
//     xTimerStart(mqttReconnectTimer, 0);
//   }
// }

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
 \code{.c}
  amqttclient.onConnect(onMqttSubscribe);
 \endcode
 
*/
// void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
//   log_i("Subscribe acknowledged. PacketId : %d, qos : %d",packetId, qos);
//   // Serial.println("Subscribe acknowledged.");
//   // Serial.print("  packetId: ");
//   // Serial.println(packetId);
//   // Serial.print("  qos: ");
//   //Serial.println(qos);
// }

/*!
 * @brief Callback function associated with MQTT incoming message state.
 *
 * deals with the message received from broker
 *
 * @param [out]     topic       topic the client subscribes to
 * @param [out]     payload     the message content
 * 
 * @return None
 * 
 \code{.c}
  amqttclient.onConnect(onMqttMessage);
 \endcode
 
*/
void onMqttMessage(String &topic, String &payload) {//, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total){
  log_i("Incoming subscribed message. Topic : %s, payload : %s",topic, payload);
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
 \code{.c}
  amqttclient.onConnect(onMqttPublish);
 \endcode
 
*/
// void onMqttPublish(uint16_t packetId) {
//   log_i("Publish acknowledged. PacketId : %d",packetId); 
//   // Serial.println("Publish acknowledged.");
//   // Serial.print("  packetId: ");
//   // Serial.println(packetId);
// }
/* #endregion Helpers */

void setup() { 
  Serial.begin(115200);
  #if CORE_DEBUG_LEVEL > 5
    Serial.setDebugOutput(false);
  #endif
  
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // initialize the size of the memory. Required for ESP32 (and update not working also)
  log_v("Size of EEPROM : %d",EEPROM.length());

  //------------------------------------------
  //WiFi and MQTT settings
  //------------------------------------------
  //mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  
  //WiFi.onEvent(wifiEventHandler);

  WiFi.setHostname(THINGNAME);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);

// Only try 15 times to connect to the WiFi
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 15){
    delay(500);
    Serial.print(".");
    retries++;
  }

  // If we still couldn't connect to the WiFi, go to deep sleep for a minute and try again.
  if(WiFi.status() != WL_CONNECTED){
    esp_sleep_enable_timer_wakeup(1 * 60L * 1000000L);
    esp_deep_sleep_start();
  }
  

  #if TSL_ENABLED == 1
    netClientSecure.setCACert(CERT_CA);
    netClientSecure.setCertificate(CERT_CRT);
    netClientSecure.setPrivateKey(CERT_PRIVATE);
  #endif

  // Client *globalWiFiClient;
  // #if TSL_ENABLED == 1
  //   globalWiFiClient = dynamic_cast<Client*> (&netClientSecure);
  // #else
  //   globalWiFiClient = dynamic_cast<Client*> (&wificlient);
  // #endif

  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.subscribe("broker/pub",1);
    mqttClient.begin(BROKER_ADDR,BROKER_PORT,netClientSecure);//*globalWiFiClient);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setOptions(30,true,2000); //keepAlive, clean session, timeout
  }
  connectToMqtt();
 
  //------------------------------------------
  //BME680 settings
  //------------------------------------------
  Wire.begin();
  bme_dev.begin(BME680_I2C_ADDR_PRIMARY,Wire);

  bme_dev.setConfig(bsec_config_iaq);

  loadState();

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
  mqttClient.loop();
}

