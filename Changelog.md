# Changelog

## 1.0.8 - released

## 1.0.7 - !! Error
278.0 /root/Arduino/libraries/modem-freeRTOS/src/modem-freeRTOS.cpp: In member function 'bool MODEMfreeRTOS::mqtt_pushMessage(uint8_t, const String&, const String&, uint8_t, uint8_t)':
278.0 /root/Arduino/libraries/modem-freeRTOS/src/modem-freeRTOS.cpp:1718:38: error: 'clientId' was not declared in this scope; did you mean 'clientID'?
278.0  1718 |   Serial.println("clientId: "+String(clientId)+ " topic: "+topic);

	- MQTT 2nd channel fix
	- Default MQTT Queues size increased

## 1.0.6 - released
	- fixes wifi reconnect

## 1.0.5 - released
	- Gets port from url 
	- Disables DEBUG_HTTP macro
	- Adds logs

## 1.0.4 - released
	- Adds Wifi reconnect

## 1.0.3 - released
	- uses modem-freeRTOS.hpp v1.0.2
	- WiFi events changed
	- changes wifi mqtt callback: mqtt_wifi_setup
	- adds more documentation
	- adds changelog
	- changes mqtt_configure_connection calls
