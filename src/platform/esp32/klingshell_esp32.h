#ifndef ESP8266

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H
#include <esp_partition.h>
#include <esp_chip_info.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>

void setupOTA();

#endif // PLATFORM_ESP32_H
#endif // ESP8266
