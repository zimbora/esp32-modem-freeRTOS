

#include "credentials.h"
#include "modem-freeRTOS.hpp"
#include <ArduinoJson.h>

/*
* Edit editable_macros file in src path to change between WiFi and LTE
* This example makes an https post to get a key and uses that key to make an http get request thereafter
* Configurations relative to WiFi/LTE and hosts must be defined in an external file - "credentials.h"
*/

// HARDWARE
#define PWKEY 4

// CONTEXT
#define CLIENTID 0
#define SSLCLIENTID 0
#define CONTEXTID 1

DynamicJsonDocument doc(2048);

MODEMfreeRTOS mRTOS;

HTTP_HEADER_MSG* msg_header;
HTTP_BODY_MSG* msg_body;

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished(){
  Serial.println("mqtt is connected - impossible to happen on this code");
}

#ifdef ENABLE_LTE
void network_lte_task(void *pvParameters);
void network_lte_task(void *pvParameters){
  (void) pvParameters;

  mRTOS.init(SETTINGS_NB_COPS,AUTO,PWKEY); // initialize modem

  if(!mRTOS.set_ssl(CONTEXTID))
    Serial.println("failing configuring ssl");
  mRTOS.set_context(1,SETTINGS_NB_APN,SETTINGS_NB_USERNAME,SETTINGS_NB_PASSWORD);

  for(;;){
    mRTOS.loop();
  }
}
#endif

String do_request(String host, String path, String method, String header_key, String header_value, String body, bool json){

  #ifdef ENABLE_LTE
  mRTOS.https_pushMessage(CONTEXTID,CLIENTID,SSLCLIENTID,host,path,method,header_key,header_value,body,json);
  #else
  mRTOS.https_pushMessage(host,path,method,header_key,header_value,body,json);
  #endif

  #ifdef ENABLE_LTE
  uint32_t timeout = millis() + 60000; // 60 seconds timeout
  #else
  uint32_t timeout = millis() + 30000; // 15 seconds timeout
  #endif
  while(timeout > millis()){
    msg_header = mRTOS.http_header_getNextMessage(msg_header);
    if(msg_header != NULL){
      if(msg_header->http_response.indexOf("200") > -1){
        uint32_t len = 0;
        char* data = (char*)malloc(msg_header->body_len);
        if(data == nullptr)
          return "";
        while(msg_header->body_len > len){
          msg_body = mRTOS.http_body_getNextMessage(msg_body);

          if(msg_body != NULL){

            for(uint16_t i=0;i<msg_body->data_len;i++){
              data[len+i] = msg_body->data[i];
            }

            len += msg_body->data_len;

          }
          delay(100); // use delay to moderate concurrency access to queues
        }

        String body = "";
        for(uint16_t i=0;i<len;i++){
          body += data[i];
        }

        //free(data);
        return body;
      }
      else Serial.printf("Invalid response: %s \n",msg_header->http_response.c_str());
    }
    #ifdef ENABLE_LTE
    delay(100); // use delay to moderate concurrency access to queues
    #endif
  }

  if(timeout < millis())
    Serial.println("Request timeout");

  return "";
}

void core(void *pvParameters);
void core(void *pvParameters){
  (void) pvParameters;

  #ifndef ENABLE_LTE
  Serial.println("wait for wifi connection..");
  while(!mRTOS.isWifiConnected()) delay(100);
  Serial.println("wifi is connected");
  #else
  Serial.println("waiting for modem to register on network..");
  while(!mRTOS.isLTERegistered()) delay(100);
  Serial.println("modem is registered");
  #endif


  String host = HTTPS_HOST;
  String path = "/api/auth"; // must start with '/'
  String method = "POST";
  String token_key = "";
  String token_value = "";
  String body = HTTPS_BODY;
  bool json = false;
  String json_str = do_request(host,path,method,token_key,token_value,body,json);

  DeserializationError error = deserializeJson(doc, json_str.c_str());

  if(error){
    Serial.println("Error parsing JSON: "+String(error.c_str()));
    Serial.println(json_str);
  }

  if(doc.containsKey("status")){
    if(doc["status"] == "ok"){
      method = "GET";
      if(doc.containsKey("data") && doc["data"].containsKey("session")){
        const char* key = doc["data"]["session"];
        Serial.println("token: "+String(key));
        token_key = "X-Session";
        token_value = String(key);
        String entities = do_request(host,"/api/entities",method,token_key,token_value,"",false);

        DeserializationError error = deserializeJson(doc, entities.c_str());

        if(error){
          Serial.println("Error parsing JSON: "+String(error.c_str()));
          Serial.println(entities);
        }

        if(doc["status"] == "ok"){
          String data = doc["data"];
          Serial.println(data);
        }
      }
    }else if(doc["status"] != "ok")
      Serial.println("response came with an error");
  }else{
    Serial.println("status key not found");
  }

  for(;;){

  }
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  #ifdef ENABLE_LTE
  // give highest priority to modem - otherwise data coming from modem can be lost
  xTaskCreatePinnedToCore(
      network_lte_task
      ,  "network_lte_task"   // A name just for humans
      ,  2048*4  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. !! do not edit priority
      ,  NULL
      ,  1);
    #endif

    xTaskCreatePinnedToCore(
        core
        ,  "core"   // A name just for humans
        ,  2048*4  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. !! do not edit priority
        ,  NULL
        ,  1);

    #ifndef ENABLE_LTE
    mRTOS.init(WIFI_SSID,WIFI_PASSWORD);
    Serial.println("wifi interface configured");
    #endif
}


void loop() {
  // put your main code here, to run repeatedly:

  //ap.loop();
  #ifndef ENABLE_LTE
  mRTOS.loop();
  #endif

}
