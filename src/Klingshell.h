#ifndef KLING_SHELL_H
#define KLING_SHELL_H

#include <Wire.h>
#ifndef ESP8266
#include "platform/esp32/KlingShell_esp32.h"
#endif
#include <ArduinoJson.h>
#include <Arduino.h>
#ifndef ESP8266
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_system.h>
#include <esp_ota_ops.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <user_interface.h>
#include <ESP8266WebServer.h>
#endif
#include <ArduinoJson.h>
#include <Wire.h>
#include <vector>

#ifndef ESP8266
#include <SPIFFS.h>
#elif defined(ESP8266)
#include <FS.h>
#include <LittleFS.h>
#define SPIFFS LittleFS
#include <Ticker.h>
#endif

extern std::vector<int> digitalPins;
extern std::vector<int> analogPins;

class KlingShellClass {
private:
    struct QueueItem {
        String toJsonKeyValue(String key, String value, bool isEnd = false, bool flagSanitize = false) const {
            return "\"" + key + "\": \"" + (flagSanitize ? toJsonSanitizedValue(value) : value) + "\"" + (isEnd ? "" : ", ");
        }

        String toJsonSanitizedValue(const String &input) const {
            String output;
            output.reserve(input.length());

            for (unsigned int i = 0; i < input.length(); i++) {  
                switch (input[i]) {
                    case '\\': output += "\\\\"; break;
                    case '\"': output += "\\\""; break;
                    case '\b': output += "\\b"; break;
                    case '\f': output += "\\f"; break;
                    case '\n': output += "\\n"; break;
                    case '\r': output += "\\r"; break;
                    case '\t': output += "\\t"; break;
                    default:
                        if ('\x00' <= input[i] && input[i] <= '\x1f') {
                            char buf[7];
                            sprintf(buf, "\\u%04x", input[i]);
                            output += buf;
                        } else {
                            output += input[i];
                        }
                }
            }

            return output;
        }

        String toJson() const {
            return "{" + toJsonKeyValue("deviceId", deviceId) + toJsonKeyValue("payload", payload, false, true) + toJsonKeyValue("handler", handler, true) + "}";
        }

    public:
        String payload;
        String handler;
        String deviceId;

        QueueItem(String payload, String handler, String deviceId) : payload(payload), handler(handler), deviceId(deviceId) {}
    };

    String url;
    time_t lastFlushed;
    time_t flushInterval;
    time_t failFlushInterval;
    HTTPClient http;
    std::vector<QueueItem> queue;
    bool lastFailed;
    String deviceId;
    bool analogTracing = false;
    bool digitalTracing = false;
    std::vector<int> tracedAnalogPins;
    std::vector<int> tracedDigitalPins;
    unsigned long lastAnalogTraceTime = 0;
    unsigned long lastDigitalTraceTime = 0;
    const unsigned long traceInterval = 1000; // 1 second

public:
    KlingShellClass() : url(""), lastFlushed(clock() + 5000), lastFailed(false), deviceId("UnknownDevice") {}

    ~KlingShellClass() {}

    void begin(String url, time_t flushInterval, time_t failFlushInterval, String deviceId) {
        this->url = url;
        this->flushInterval = flushInterval;
        this->failFlushInterval = failFlushInterval;
        this->deviceId = deviceId;
        clear();
    }

    void setDeviceId(String deviceId) {
        this->deviceId = deviceId;
    }

    void tick() {
        unsigned long currentTime = millis();

        if (currentTime - lastFlushed > (lastFailed ? failFlushInterval : flushInterval)) {
            lastFlushed = currentTime;
            if (WiFi.status() == WL_CONNECTED) {
                flush();
            }
        }
        checkForCommands();

        if (analogTracing && currentTime - lastAnalogTraceTime >= traceInterval) {
            tracePins(true);
            lastAnalogTraceTime = currentTime;
        }

        if (digitalTracing && currentTime - lastDigitalTraceTime >= traceInterval) {
            tracePins(false);
            lastDigitalTraceTime = currentTime;
        }
    }

    bool send(QueueItem &queueItem) {
        bool result = false;
        int httpResponseCode;
        WiFiClient client;

        http.begin(client, url);
        http.addHeader("Content-Type", "application/json");
        //Serial.print(queueItem.toJson());
        httpResponseCode = http.POST(queueItem.toJson());

        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println(httpResponseCode);
            Serial.println(response);
            result = true;
        } else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
        }
        
        http.end();
        lastFailed = !result;
        return result;
    }

    void flush() {
        if (!queue.empty()) {
            QueueItem queueItem = queue.front();
            if (send(queueItem)) {
                queue.erase(queue.begin());
            }
        }
    }

    template<typename T>
    void println(const T &value) {
        queue.push_back(QueueItem(String(value), "println", deviceId));
    }

    template<typename T>
    void print(const T &value) {
        queue.push_back(QueueItem(String(value), "print", deviceId));
    }

    template<typename T>
    void println(const T &value, int format) {
        String formattedValue = String(value, format);
        queue.push_back(QueueItem(formattedValue, "println", deviceId));
    }

    template<typename T>
    void print(const T &value, int format) {
        String formattedValue = String(value, format);
        queue.push_back(QueueItem(formattedValue, "print", deviceId));
    }

    void cprintln(const String &value) {
        queue.push_back(QueueItem(value, "cprintln", deviceId));
    }

    void cprint(const String &value) {
        queue.push_back(QueueItem(value, "cprint", deviceId));
    }

    void clear() {
        queue.push_back(QueueItem("", "clear", deviceId));
    }

    void shell(String command) {
        queue.push_back(QueueItem(command, "shell", deviceId));
    }

    static void i2cscan();
    void reportPinStates();
    void startAnalogTracing(const String& pinList);
    void startDigitalTracing(const String& pinList);

    void listFiles();
    void readFile(const String& path);
    void writeFile(const String& path, const String& message);
    void deleteFile(const String& path);
    void scanWiFi();
    void setPWM(int pin, int dutyCycle);
    void checkForCommands();
    String getSystemInfo();
    String getIpConfiguration();
    String getHelp();
    void tracePins(bool isAnalog);
    void stopAnalogTracing();
    void stopDigitalTracing();
    float getBatteryPercentage(float maxVoltage, int pin);
    String pingHost(const String& host, int count = 4);
    void setPinMode(int pin, String mode);
    void togglePin(int pin);
    void readAnalogScaled(int pin, float scale);
    String getNRF52BLEMACAddress();
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_ARDUINOOTA)
  extern KlingShellClass KlingShell;
#endif

#endif
