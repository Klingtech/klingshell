#include <Arduino.h>
#include <NTPClient.h>
#include <Wire.h>
#include <esp_partition.h>
#include <esp_chip_info.h>

#ifndef ESP8266
#include "platform/esp32/KlingShell_esp32.h"

#elif defined(ESP8266)
#include "platform/esp8266/KlingShell_esp8266.h"
#endif

#include "KlingShell.h"
#include "credentials.h"
#include <vector>

// Define the GPIO pins for different ESP32 models
    // Define the GPIO pins for different ESP32/ESP8266 models
    std::vector<int> digitalPins;
    std::vector<int> analogPins;

// Function to set pins based on the chip model

#ifndef ESP8266
void setPinsBasedOnChipModel() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    switch(chip_info.model) {
        case CHIP_ESP32C3:
            digitalPins = {2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
            analogPins = {2, 3, 4, 5};
            break;
        case CHIP_ESP32S3:
            digitalPins = {1, 2, 3, 4, 5, 6, 7, 8, 9, 43, 44};
            analogPins = {1, 2, 3, 4, 5, 6, 7, 8, 9};
            break;
        default:
            digitalPins = {1, 2, 3, 4, 5, 6, 7, 8, 9, 43, 44};
            analogPins = {1, 2, 3, 4, 5, 6, 7, 8, 9};
            break;
    }
}
#elif defined(ESP8266)
void setPinsBasedOnChipModel() {
    digitalPins = {0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16};
    analogPins = {A0};
}
#endif

String getDeviceId() {
#ifndef ESP8266
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    const char* chip_model;
    switch(chip_info.model) {
        case CHIP_ESP32: 
            chip_model = "ESP32";
            break;
        case CHIP_ESP32S2: 
            chip_model = "ESP32-S2";
            break;
        case CHIP_ESP32S3: 
            chip_model = "ESP32-S3";
            break;
        case CHIP_ESP32C3: 
            chip_model = "ESP32-C3";
            break;
        case CHIP_ESP32H2: 
            chip_model = "ESP32-H2";
            break;
        default: 
            chip_model = "UNKNOWN";
    }

    int chip_revision = chip_info.revision;

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    char macStr[13];
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return String(chip_model) + "-Rev" + String(chip_revision) + "-" + String(macStr);
#elif defined(ESP8266)
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return "ESP8266-" + String(macStr);
#endif
}

String listPins() {
    String pinInfo = "Available GPIO Pins:\n";
    for (int pin : digitalPins) {
        pinInfo += "Pin " + String(pin) + ": ";

        pinMode(pin, INPUT);
        if (digitalRead(pin) != -1) {
            pinInfo += "Digital ";
        }

        if (std::find(analogPins.begin(), analogPins.end(), pin) != analogPins.end()) {
#ifndef ESP8266
            pinMode(pin, ANALOG);
#endif
            if (analogRead(pin) != -1) {
                pinInfo += "Analog ";
            }
        }

        pinInfo += "\n";
    }
    return pinInfo;
}

void reportPinStates() {
    String report;

    report += "Digital Pin States:\n";
    for (int pin : digitalPins) {
        pinMode(pin, INPUT);
        int pinState = digitalRead(pin);
        report += "Pin " + String(pin) + ": " + String(pinState) + "\n";
    }

    report += "Analog Pin States:\n";
    for (int pin : analogPins) {
        pinMode(pin, INPUT);
        int pinValue = analogRead(pin);
        report += "Pin " + String(pin) + ": " + String(pinValue) + "\n";
    }

    report.trim();
    KlingShell.println(report);
}

void KlingShellClass::reportPinStates() {
    ::reportPinStates();
}

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600 * 2);

KlingShellClass KlingShell;

unsigned long bootTime;

void setup() {
    Serial.begin(115200);
    Serial.println("Booting");

    // Set the pins based on the chip model
    setPinsBasedOnChipModel();
    pinMode(A0, INPUT);

    // Generate the device ID
    String deviceId = getDeviceId();

    // Connect to WiFi
    setPinsBasedOnChipModel();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    timeClient.begin();
    while (!timeClient.update()) {
        timeClient.forceUpdate();
    }

    bootTime = timeClient.getEpochTime();

    KlingShell.begin(server, 100, 1000, deviceId);
    KlingShell.clear();
    KlingShell.println(deviceId);

    IPAddress ip = WiFi.localIP();
    String ipStr = ip.toString();

    String formattedTime = timeClient.getFormattedTime();
    KlingShell.println("Current time: " + formattedTime);

    // Set up OTA
    setupOTA();

    KlingShell.println("Ready");
    KlingShell.cprint("[#13F700]IP address: ");
    KlingShell.cprintln("[#13F700]" + ipStr);
}

void loop() {
    ArduinoOTA.handle();
    KlingShell.tick();

    delay(200); // Optional: Delay to reduce the frequency of updates
    
}