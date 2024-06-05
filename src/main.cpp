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
#include <vector>

// Enter your wifi details here or create credentials.h in the same directory as the main.cpp file and add the following:
// const char *ssid = "*********";
// const char *password = "**********";

// Enter your KlingShell server here
// const char *server = "http://IPORDNS:10000/KlingShell";

// Comment out if not using credentials.h for above details
#include "credentials.h"

// Define the GPIO pins for different ESP32 models
    // Define the GPIO pins for different ESP32/ESP8266 models
    std::vector<int> digitalPins;
    std::vector<int> analogPins;

/**
 * @brief Function to set pins based on the chip model
 * 
 * This function uses the esp_chip_info library to determine the model of the ESP32 chip.
 * It then populates the digitalPins and analogPins vectors with the appropriate pins for the chip model.
 */
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

/**
 * @brief Function to generate the device ID
 * 
 * This function uses the esp_chip_info library to determine the model and revision of the ESP32 chip.
 * It then retrieves the MAC address of the chip and formats it into a device ID string.
 * 
 * @return A string representing the device ID
 */

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

/**
 * @brief Function to list the available GPIO pins
 * 
 * This function iterates through the digitalPins and analogPins vectors and checks the pin mode and availability.
 * It then constructs a string with information about each pin.
 * 
 * @return A string containing information about the available GPIO pins
 */

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

/**
 * @brief Function to report the current state of the GPIO pins
 * 
 * This function iterates through the digitalPins and analogPins vectors and reads the pin state or value.
 * It then constructs a string with information about the state of each pin.
 * 
 * @return A string containing information about the state of the GPIO pins
 */

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

/**
 * @brief Overloaded function to report the current state of the GPIO pins
 * 
 * This function is an overloaded version of the reportPinStates function.
 * It calls the original reportPinStates function.
 */

void KlingShellClass::reportPinStates() {
    ::reportPinStates();
}

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600 * 2);

KlingShellClass KlingShell;

unsigned long bootTime;

/**
 * @brief Main setup function
 * 
 * This function initializes the serial communication, sets up the pins based on the chip model, generates the device ID,
 * connects to the WiFi, retrieves the current time from an NTP server, and initializes the KlingShell library.
 */
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

    /**
     * @brief Function to set up Over-The-Air (OTA) updates
     * 
     * This function initializes the ArduinoOTA library and sets up the necessary callbacks for handling OTA updates.
     */
    setupOTA();

    KlingShell.println("Ready");
    KlingShell.cprint("[#13F700]IP address: ");
    KlingShell.cprintln("[#13F700]" + ipStr);
}

/**
 * @brief Main loop function
 * 
 * This function handles the Over-The-Air (OTA) updates and calls the KlingShell tick function.
 * It also includes a delay to reduce the frequency of updates.
 */

void loop() {
    ArduinoOTA.handle();
    KlingShell.tick();
    delay(200); // Optional: Delay to reduce the frequency of updates
    
}