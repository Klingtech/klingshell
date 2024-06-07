#ifdef ESP8266

#ifndef PLATFORM_ESP8266_H
#define PLATFORM_ESP8266_H

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HttpClient.h>
#include <ArduinoOTA.h>
#include <user_interface.h>
#include <LittleFS.h>
#include <ESP8266Ping.h>
#define SPIFFS LittleFS

void setupOTA();

#endif //PLATFORM_ESP8266_H
#endif //ESP8266
