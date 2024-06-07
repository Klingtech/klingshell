#include "KlingShell.h"
#ifdef ESP8266
#include "platform/esp8266/klingshell_esp8266.h"
#endif
#ifndef ESP8266
#include "platform/esp32/klingshell_esp32.h"
#endif

float KlingShellClass::getBatteryPercentage(float maxVoltage, float resistor1, float resistor2, int pin) {
#ifdef ESP8266
    int raw = analogRead(A0);
    float voltage = (raw / 1024.0) * 3.3;
    int millivolts = voltage * 1000;
#else
    int millivolts = analogReadMilliVolts(pin);
#endif

    float voltageDividerRatio = resistor2 / (resistor1 + resistor2);
    float batteryVoltage = millivolts / 1000.0 / voltageDividerRatio;

    float percentage = map(batteryVoltage, maxVoltage * 0.7, maxVoltage, 0, 100);
    percentage = constrain(percentage, 0, 100);

    return percentage;
}

void KlingShellClass::i2cscan() {
    Wire.begin();
    byte error, address;
    int nDevices = 0;
    String output = "";

    for (address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            if (nDevices == 0) {
                KlingShell.cprint("[#13F700]I2C devices found at the following addresses:");
            }
            output += "0x";
            if (address < 16) output += "0";
            output += String(address, HEX) + "\n";
            nDevices++;
        } else if (error == 4) {
            KlingShell.cprint("[#f44336]Error at address 0x");
            if (address < 16) output += "0";
            output += String(address, HEX) + "\n";
        }
    }

    if (nDevices == 0) {
        output += "No I2C devices found\n";
    } else {
        output += "Scan complete!";
    }

    KlingShell.cprint(output);

#ifndef ESP8266
    Wire.end();
#endif
}

void KlingShellClass::listFiles() {
    if (!SPIFFS.begin()) {
        cprintln("SPIFFS Mount Failed");
        return;
    }

#ifndef ESP8266
    File root = SPIFFS.open("/");
#elif defined(ESP8266)
    File root = SPIFFS.open("/", "r");
#endif

    File file = root.openNextFile();
    String fileList = "SPIFFS:";
    while (file) {
        fileList += "\n[#ffdd00]" + String(file.name()) + "[#]";
        file = root.openNextFile();
    }

    cprintln(fileList);
}

void KlingShellClass::readFile(const String& path) {
    if (!SPIFFS.begin()) {
        cprintln("SPIFFS Mount Failed");
        return;
    }

#ifndef ESP8266
    File file = SPIFFS.open(path);
#elif defined(ESP8266)
    File file = SPIFFS.open(path, "r");
#endif

    if (!file) {
        cprintln("Failed to open file for reading");
        return;
    }

    String fileContent = "File content:\n";
    while (file.available()) {
        fileContent += char(file.read());
    }
    file.close();

    cprintln(fileContent);
}

void KlingShellClass::writeFile(const String& path, const String& message) {
    if (!SPIFFS.begin()) {
        cprintln("SPIFFS Mount Failed");
        return;
    }

#ifndef ESP8266
    File file = SPIFFS.open(path, FILE_WRITE);
#elif defined(ESP8266)
    File file = SPIFFS.open(path, "w");
#endif

    if (!file) {
        cprintln("Failed to open file for writing");
        return;
    }

    if (file.print(message)) {
        cprintln("File written successfully");
    } else {
        cprintln("Write failed");
    }
    file.close();
}

void KlingShellClass::deleteFile(const String& path) {
    if (!SPIFFS.begin()) {
        cprintln("SPIFFS Mount Failed");
        return;
    }

    if (SPIFFS.remove(path)) {
        cprintln("File deleted successfully");
    } else {
        cprintln("Delete failed");
    }
}

void KlingShellClass::scanWiFi() {
    int n = WiFi.scanNetworks();
    if (n == 0) {
        cprintln("No networks found");
    } else {
        String networkList = "Networks:\n";
        for (int i = 0; i < n; ++i) {
            networkList += String(WiFi.SSID(i)) + " (" + String(WiFi.RSSI(i)) + ")\n";
        }
        cprintln(networkList);
    }
}

void KlingShellClass::setPWM(int pin, int dutyCycle) {
#ifndef ESP8266
    ledcAttachPin(pin, 0);
    ledcSetup(0, 5000, 8);
    ledcWrite(0, dutyCycle);
#elif defined(ESP8266)
    analogWrite(pin, dutyCycle);
#endif
}

void KlingShellClass::reportPinStates() {
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

void KlingShellClass::startAnalogTracing(const String& pinList) {
    tracedAnalogPins.clear();
    analogTracing = true;
    int start = 0;
    int end = pinList.indexOf(',');
    while (end != -1) {
        tracedAnalogPins.push_back(pinList.substring(start, end).toInt());
        start = end + 1;
        end = pinList.indexOf(',', start);
    }
    tracedAnalogPins.push_back(pinList.substring(start).toInt());
}

void KlingShellClass::startDigitalTracing(const String& pinList) {
    tracedDigitalPins.clear();
    digitalTracing = true;
    int start = 0;
    int end = pinList.indexOf(',');
    while (end != -1) {
        tracedDigitalPins.push_back(pinList.substring(start, end).toInt());
        start = end + 1;
        end = pinList.indexOf(',', start);
    }
    tracedDigitalPins.push_back(pinList.substring(start).toInt());
}

void KlingShellClass::tracePins(bool isAnalog) {
    if (isAnalog && analogTracing) {
        for (int pin : tracedAnalogPins) {
            int value = analogRead(pin);
            cprintln("Analog pin " + String(pin) + ": " + String(value));
        }
    }
    if (!isAnalog && digitalTracing) {
        for (int pin : tracedDigitalPins) {
            int value = digitalRead(pin);
            cprintln("Digital pin " + String(pin) + ": " + String(value));
        }
    }
}

void KlingShellClass::stopAnalogTracing() {
    analogTracing = false;
    tracedAnalogPins.clear();
}

void KlingShellClass::stopDigitalTracing() {
    digitalTracing = false;
    tracedDigitalPins.clear();
}

String KlingShellClass::getSystemInfo() {
    String info;

#ifndef ESP8266
    String wifiMac = WiFi.macAddress();
    info += "WiFi MAC Address: " + wifiMac + "\n";
    uint8_t bleMac[6];
    esp_read_mac(bleMac, ESP_MAC_BT);
    char bleMacStr[18];
    sprintf(bleMacStr, "%02X:%02X:%02X:%02X:%02X:%02X", bleMac[0], bleMac[1], bleMac[2], bleMac[3], bleMac[4], bleMac[5]);
    info += "BLE MAC Address: " + String(bleMacStr) + "\n";
    info += "CPU Frequency: " + String(getCpuFrequencyMhz()) + " MHz\n";
    info += "Free Heap Size: " + String(ESP.getFreeHeap()) + " bytes\n";
    info += "Minimum Free Heap Size: " + String(ESP.getMinFreeHeap()) + " bytes\n";
    info += "Chip Revision: " + String(ESP.getChipRevision()) + "\n";
    info += "SDK Version: " + String(ESP.getSdkVersion()) + "\n";
    const esp_partition_t* partition = esp_ota_get_running_partition();
    if (partition) {
        info += "Partition Type: " + String(partition->type) + "\n";
        info += "Partition Subtype: " + String(partition->subtype) + "\n";
        info += "Partition Size: " + String(partition->size) + " bytes\n";
        info += "Partition Label: " + String(partition->label);
    } else {
        info += "No partition information available.\n";
    }
#elif defined(ESP8266)
    String wifiMac = WiFi.macAddress();
    info += "WiFi MAC Address: " + wifiMac + "\n";
    info += "CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz\n";
    info += "Free Heap Size: " + String(ESP.getFreeHeap()) + " bytes\n";
    info += "SDK Version: " + String(ESP.getSdkVersion()) + "\n";
#endif

    return info;
}

String KlingShellClass::getHelp() {
  String help = "---------------------------------------------------------------------------\n";
  help += "Available Commands (case-insensitive):\n";
  help += "\nPin Control:\n";
  help += "  ar<pin#>      - Read analog value from pin (e.g., 'ar32')\n";
  help += "  dr<pin#>      - Read digital value (HIGH/LOW) from pin (e.g., 'dr13')\n";
  help += "  pwm<pin#>|<duty> - Set PWM duty cycle (0-255) on pin (e.g., 'pwm23|128')\n";
  help += "  tpa<pins>     - Trace analog values continuously (e.g., 'tpa 32,33')\n";
  help += "  tpa stop      - Stop analog tracing\n";
  help += "  tpd<pins>     - Trace digital values continuously (e.g., 'tpd 0,12')\n";
  help += "  tpd stop      - Stop digital tracing\n";
  help += "  ra            - Report ALL current pin states (analog & digital) Define these in main.cpp to suit your board. Else it might crash and reboot.\n";
  
  help += "\nFile System (SPIFFS):\n";
  help += "  lf            - List files stored on the device\n";
  help += "  rf<path>      - Read contents of a file (e.g., 'rf/data.txt')\n";
  help += "  wf<path>|<data>- Write data to a file (e.g., 'wf/log.txt|Hello!')\n";
  help += "  df<path>      - Delete a file (e.g., 'df/old.log')\n";

  help += "\nDiagnostics & Information:\n";
  help += "  i2c           - Scan for I2C devices on the bus\n";
  help += "  wifi          - Scan for available Wi-Fi networks\n";
  help += "  info          - Get detailed system information (CPU, memory, etc.)\n";
  help += "  ip            - Display the IP configuration of the device\n";
  help += "  ping target       - Pings target IP or DNS\n";
  help += "  bat           - Display the battery percentage assuming pin A0 on ESP8266 or GPIO14 for ESP32 with 3.3v and 220k resistors.\n";
  help += "  bat|<pin#>|<maxV>|<R1>|<R2> - Customize battery reading with battery voltage and the resistance of resistor 1 and 2 (e.g., 'bat|14|4.2|220000|220000')\n";

  help += "\nOther:\n";
  help += "  reset         - Restart the device\n";
  help += "  help or ?     - Show this help message\n";
  help += "---------------------------------------------------------------------------\n";
  return help;
}

void KlingShellClass::checkForCommands() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        WiFiClient client;
        String urlWithDeviceId = url + "/commands?deviceId=" + deviceId;
        http.begin(client, urlWithDeviceId);
        int httpResponseCode = http.GET();

        if (httpResponseCode == 200) {
            String response = http.getString();
            Serial.println(response);

            StaticJsonDocument<1024> doc;
            deserializeJson(doc, response);
            JsonArray commands = doc["commands"];

            for (JsonVariant command : commands) {
                String cmd = command.as<String>();
                String result;

                // Echo the received command
                result = "Executed command: " + cmd;
                KlingShell.println(result);

                if (cmd.startsWith("ar")) {
                    int pin = cmd.substring(2).toInt();
                    int value = analogRead(pin);
                    result = "Analog pin " + String(pin) + " value: " + String(value);
                } else if (cmd.startsWith("dr")) {
                    int pin = cmd.substring(2).toInt();
                    int value = digitalRead(pin);
                    result = "Digital pin " + String(pin) + " value: " + String(value);
                } else if (cmd == "i2c") {
                    i2cscan();
                    result = "Scanning for I2C devices...";
                } else if (cmd == "ra") {
                    reportPinStates();
                    result = "Reading all Pin States...";
                } else if (cmd == "info") {
                    result = getSystemInfo();
                } else if (cmd == "lf") {
                    listFiles();
                    result = "Listing files...";
                } else if (cmd == "reset") {
#ifndef ESP8266
                    esp_restart();
#elif defined(ESP8266)
                    ESP.restart();
#endif
                    result = "Resetting device...";
                } else if (cmd.startsWith("rf")) {
                    String path = cmd.substring(2);
                    readFile(path);
                    result = "Reading file: " + path;
                } else if (cmd.startsWith("wf")) {
                    int separator = cmd.indexOf('|');
                    if (separator != -1) {
                        String path = cmd.substring(2, separator);
                        String message = cmd.substring(separator + 1);
                        writeFile(path, message);
                        result = "Writing to file: " + path;
                    } else {
                        result = "Invalid wf command format";
                    }
                } else if (cmd.startsWith("wifi")) {
                    scanWiFi();
                    result = "Scanning for WiFi networks...";
                } else if (cmd.startsWith("pwm")) {
                    int separator = cmd.indexOf('|');
                    if (separator != -1) {
                        int pin = cmd.substring(3, separator).toInt();
                        int dutyCycle = cmd.substring(separator + 1).toInt();
                        setPWM(pin, dutyCycle);
                        result = "Setting PWM on pin " + String(pin) + " to duty cycle " + String(dutyCycle);
                    } else {
                        result = "Invalid pwm command format";
                    }
                } else if (cmd.startsWith("df")) {
                    String path = cmd.substring(2);
                    deleteFile(path);
                    result = "Deleting file: " + path;
                } else if (cmd == "help" || cmd == "?") {
                    result = getHelp();
                } else if (cmd.startsWith("tpa")) {
                    String pinList = cmd.substring(3);
                    if (pinList.startsWith(" stop")) {
                        stopAnalogTracing();
                        result = "Stopped analog tracing";
                    } else {
                        startAnalogTracing(pinList);
                        result = "Started analog tracing for pins: " + pinList;
                    }
                } else if (cmd.startsWith("bat")) {
                    float maxVoltage = 4.2;
                    float resistor1 = 220000;
                    float resistor2 = 220000;
                    int pin = 14;
#ifdef ESP8266
                    pin = A0;
#endif

                    int firstSeparator = cmd.indexOf('|');
                    if (firstSeparator != -1) {
                        pin = cmd.substring(4, firstSeparator).toInt();
                        int secondSeparator = cmd.indexOf('|', firstSeparator + 1);
                        if (secondSeparator != -1) {
                            maxVoltage = cmd.substring(firstSeparator + 1, secondSeparator).toFloat();
                            int thirdSeparator = cmd.indexOf('|', secondSeparator + 1);
                            if (thirdSeparator != -1) {
                                resistor1 = cmd.substring(secondSeparator + 1, thirdSeparator).toFloat();
                                resistor2 = cmd.substring(thirdSeparator + 1).toFloat();
                            } else {
                                resistor2 = cmd.substring(secondSeparator + 1).toFloat();
                            }
                        }
                    }
                    float percentage = getBatteryPercentage(maxVoltage, resistor1, resistor2, pin);
                    result = "Battery Percentage: " + String(percentage) + "%";
                } else if (cmd.startsWith("tpd")) {
                    String pinList = cmd.substring(3);
                    if (pinList.startsWith(" stop")) {
                        stopDigitalTracing();
                        result = "Stopped digital tracing";
                    } else {
                        startDigitalTracing(pinList);
                        result = "Started digital tracing for pins: " + pinList;
                    }
                } else if (cmd == "ip") {
                    // Display IP configuration
                    result = getIpConfiguration();
                } else if (cmd.startsWith("ping")) {
                    int separator = cmd.indexOf('|');
                    String host = cmd.substring(5);
                    if (separator != -1) {
                        host = cmd.substring(5, separator);
                    }
                    result = pingHost(host);
                } else {
                    result = "Unknown command: " + cmd;
                }

                Serial.println(result);
                QueueItem queueItem(result, "cprintln", deviceId);
                send(queueItem);
            }
        } else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
        }

        http.end();
    }
}

String KlingShellClass::getIpConfiguration() {
    IPAddress localIp = WiFi.localIP();
    IPAddress subnetMask = WiFi.subnetMask();
    IPAddress gatewayIp = WiFi.gatewayIP();
    IPAddress dnsIp = WiFi.dnsIP();

    String ipInfo = "IP Configuration:\n";
    ipInfo += "Local IP: " + localIp.toString() + "\n";
    ipInfo += "Subnet Mask: " + subnetMask.toString() + "\n";
    ipInfo += "Gateway IP: " + gatewayIp.toString() + "\n";
    ipInfo += "DNS IP: " + dnsIp.toString() + "\n";

    return ipInfo;
}

String KlingShellClass::pingHost(const String& host, int count) {
    String result = "Pinging " + host + "...\n";
    
    #ifdef ESP8266
        IPAddress ip;
        if (WiFi.hostByName(host.c_str(), ip)) {
            if (Ping.ping(ip, count)) {
                result += "Ping successful!\n";
                result += "Average response time: " + String(Ping.averageTime()) + " ms\n";
            } else {
                result += "Ping failed.\n";
            }
        } else {
            result += "Failed to resolve hostname.\n";
        }
    #elif defined(ESP32)
        if (Ping.ping(host.c_str(), count)) {
            result += "Ping successful!\n";
            result += "Average response time: " + String(Ping.averageTime()) + " ms\n";
        } else {
            result += "Ping failed.\n";
        }
    #endif
    
    return result;
}

