

#include "credentials.h"
#include "modem-freeRTOS.hpp"

// HARDWARE
#define PWKEY 4

// CONTEXT
#define CLIENTID 0
#define CONTEXTID 1


MODEMfreeRTOS mRTOS;
void (*callback1)(uint8_t clientID);
void (*callback2)(uint8_t clientID);
void tcpOnConnect(uint8_t clientID){
  Serial.println("tcp is connected - sending first message");
  String host = "www.google.com";
  String path = "";
  String request = "GET " + path + " HTTP/1.1\r\n" +
         "Host: " + host + "\r\n" +
         "Cache-Control: no-cache\r\n" +
         "Connection: close\r\n\r\n";
  mRTOS.tcp_pushMessage(clientID,request.c_str(),request.length());
  return;
}
void tcpOnClose(uint8_t clientID){
  Serial.println("tcp was closed");
  mRTOS.tcp_configure_connection(clientID,0,"",0);
  return;
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


  mRTOS.init(SETTINGS_NB_COPS,AUTO,PWKEY); // initialize modem
  mRTOS.set_context(1,SETTINGS_NB_APN,SETTINGS_NB_USERNAME,SETTINGS_NB_PASSWORD);

  /* The following methods can be called at any time.
  * For changes to have effect mRTOS.mqtt_setup have to be called
  */
  // this creates a persistent connection unless it is rewritten again
  mRTOS.tcp_configure_connection(CLIENTID,CONTEXTID,"www.google.com",80);

  callback1 = &tcpOnConnect;
  callback2 = &tcpOnClose;
  mRTOS.tcp_setup(callback1,callback2);
}

TCP_MSG* msg;

void loop() {
  // put your main code here, to run repeatedly:

  mRTOS.loop();

  msg = mRTOS.tcp_getNextMessage(msg);
  if(msg != NULL){
    Serial.println("new tcp message received:");
    Serial.printf("<< [%d] ",msg->clientID);
    for(uint16_t i=0;i<msg->data_len;i++){
      Serial.print(msg->data[i]);
    }
  }

}
