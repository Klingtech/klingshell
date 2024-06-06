#include "KlingShell.h"

// Function to read and calculate battery percentage
float KlingShellClass::getBatteryPercentage(float maxVoltage, float resistor1, float resistor2, int pin) {
#ifdef ESP8266
    int raw = analogRead(A0);
    float voltage = (raw / 1024.0) * 3.3; // assuming the max ADC voltage is 3.3V
    int millivolts = voltage * 1000;
#else
    int millivolts = analogReadMilliVolts(pin);
#endif

    // Calculate voltage divider output based on resistor values
    float voltageDividerRatio = resistor2 / (resistor1 + resistor2);
    float batteryVoltage = millivolts / 1000.0 / voltageDividerRatio;

    // Calculate percentage based on battery's voltage range
    float percentage = map(batteryVoltage, maxVoltage * 0.7, maxVoltage, 0, 100);
    percentage = constrain(percentage, 0, 100);

    return percentage;
}

void KlingShellClass::i2cscan() {
    Wire.begin();
    byte error, address;
    int nDevices = 0;

    // Collect all the output lines
    String output = "";

    for (address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            // If this is the first device found, print the header
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

    // Send the entire output at once
    KlingShell.cprint(output);

#ifndef ESP8266
    Wire.end();
#endif
}

// Define other member functions

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
    ledcAttachPin(pin, 0); // Set pin as PWM
    ledcSetup(0, 5000, 8); // 5kHz PWM, 8-bit resolution
    ledcWrite(0, dutyCycle);
#elif defined(ESP8266)
    analogWrite(pin, dutyCycle);
#endif
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
    // Get WiFi MAC address
    String wifiMac = WiFi.macAddress();
    info += "WiFi MAC Address: " + wifiMac + "\n";

    // Get BLE MAC address
    uint8_t bleMac[6];
    esp_read_mac(bleMac, ESP_MAC_BT);
    char bleMacStr[18];
    sprintf(bleMacStr, "%02X:%02X:%02X:%02X:%02X:%02X", bleMac[0], bleMac[1], bleMac[2], bleMac[3], bleMac[4], bleMac[5]);
    info += "BLE MAC Address: " + String(bleMacStr) + "\n";

    // Get CPU frequency
    info += "CPU Frequency: " + String(getCpuFrequencyMhz()) + " MHz\n";

    // Get free heap size
    info += "Free Heap Size: " + String(ESP.getFreeHeap()) + " bytes\n";

    // Get minimum free heap size since boot
    info += "Minimum Free Heap Size: " + String(ESP.getMinFreeHeap()) + " bytes\n";

    // Get chip revision
    info += "Chip Revision: " + String(ESP.getChipRevision()) + "\n";

    // Get SDK version
    info += "SDK Version: " + String(ESP.getSdkVersion()) + "\n";

    // Get partition information
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
    // Get WiFi MAC address
    String wifiMac = WiFi.macAddress();
    info += "WiFi MAC Address: " + wifiMac + "\n";

    // Get CPU frequency
    info += "CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz\n";

    // Get free heap size
    info += "Free Heap Size: " + String(ESP.getFreeHeap()) + " bytes\n";

    // Get SDK version
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
  help += "  ra            - Report ALL current pin states (analog & digital)\n";
  
  help += "\nFile System (SPIFFS):\n";
  help += "  lf            - List files stored on the device\n";
  help += "  rf<path>      - Read contents of a file (e.g., 'rf/data.txt')\n";
  help += "  wf<path>|<data>- Write data to a file (e.g., 'wf/log.txt|Hello!')\n";
  help += "  df<path>      - Delete a file (e.g., 'df/old.log')\n";

  help += "\nDiagnostics & Information:\n";
  help += "  i2c           - Scan for I2C devices on the bus\n";
  help += "  wifi          - Scan for available Wi-Fi networks\n";
  help += "  info          - Get detailed system information (CPU, memory, etc.)\n";
  help += "  bat           - Display the battery percentage assuming pin A0 on ESP8266 or GPIO14 for ESP32 with 3.3v and 220k resistors.\n";
  help += "  bat|<pin#>|<maxV>|<R1>|<R2> - Customize battery reading with battery voltage and the resistance of resistor 1 and 2 (e.g., 'bat|14|4.2|220000|220000')\n";

  help += "\nOther:\n";
  help += "  reset         - Restart the device\n";
  help += "  help or ?     - Show this help message\n";
  help += "---------------------------------------------------------------------------\n";
  return help;
}

// Make sure the checkForCommands method is defined here
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

            // Parse response and execute commands
            StaticJsonDocument<1024> doc;  // Updated to StaticJsonDocument
            deserializeJson(doc, response);
            JsonArray commands = doc["commands"];

            for (JsonVariant command : commands) {
                String cmd = command.as<String>();
                Serial.println("Executing command: " + cmd);
                String result;

                // Interpret and execute the command
                if (cmd.startsWith("ar")) { // Alias for analogRead
                    int pin = cmd.substring(2).toInt();
                    int value = analogRead(pin);
                    result = "Analog pin " + String(pin) + " value: " + String(value);
                } else if (cmd.startsWith("dr")) { // Alias for digitalRead
                    int pin = cmd.substring(2).toInt();
                    int value = digitalRead(pin);
                    result = "Digital pin " + String(pin) + " value: " + String(value);
                } else if (cmd == "i2c") { // Command for I2C scan
                    i2cscan();
                    result = "Scanning for I2C devices...";
                } else if (cmd == "ra") { // Command for reporting all pin states
                    reportPinStates();
                    result = "Reading all Pin States...";
                } else if (cmd == "info") { // Command for system info
                    result = getSystemInfo();
                } else if (cmd == "lf") { // List files in SPIFFS
                    listFiles();
                    result = "Listing files...";
                } else if (cmd == "reset") { // Reset device
#ifndef ESP8266
                    esp_restart();
#elif defined(ESP8266)
                    ESP.restart();
#endif
                    result = "Reseting device...";
                } else if (cmd.startsWith("rf")) { // Read a specific file
                    String path = cmd.substring(2);
                    readFile(path);
                    result = "Reading file: " + path;
                } else if (cmd.startsWith("wf")) { // Write to a specific file
                    int separator = cmd.indexOf('|');
                    if (separator != -1) {
                        String path = cmd.substring(2, separator);
                        String message = cmd.substring(separator + 1);
                        writeFile(path, message);
                        result = "Writing to file: " + path;
                    } else {
                        result = "Invalid wf command format";
                    }
                } else if (cmd.startsWith("wifi")) { // WiFi scanning
                    scanWiFi();
                    result = "Scanning for WiFi networks...";
                } else if (cmd.startsWith("pwm")) { // Set PWM
                    int separator = cmd.indexOf('|');
                    if (separator != -1) {
                        int pin = cmd.substring(3, separator).toInt();
                        int dutyCycle = cmd.substring(separator + 1).toInt();
                        setPWM(pin, dutyCycle);
                        result = "Setting PWM on pin " + String(pin) + " to duty cycle " + String(dutyCycle);
                    } else {
                        result = "Invalid pwm command format";
                    }
                } else if (cmd.startsWith("df")) { // Delete a specific file
                    String path = cmd.substring(2);
                    deleteFile(path);
                    result = "Deleting file: " + path;
                } else if (cmd == "help" || cmd == "?") { // Help command
                    result = getHelp();
                } else if (cmd.startsWith("tpa")) { // Tail pin analog
                    String pinList = cmd.substring(3);
                    if (pinList.startsWith(" stop")) {
                        stopAnalogTracing();
                        result = "Stopped analog tracing";} 
                    else {
                    startAnalogTracing(pinList);
                    result = "Started analog tracing for pins: " + pinList;
                    } 
                }   else if (cmd.startsWith("bat")) {
                    // Parse parameters (if any) from the command
                    float maxVoltage = 4.2; // Default values
                    float resistor1 = 220000;
                    float resistor2 = 220000;
                    int pin = 14; // Default pin for ESP32
#ifdef ESP8266
                    pin = A0; // Default pin for ESP8266
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
                } else if (cmd.startsWith("tpd")) { // Tail pin digital
                String pinList = cmd.substring(3);
                if (pinList.startsWith(" stop")) {
                    stopDigitalTracing();
                    result = "Stopped digital tracing";}
                    else {
                    startDigitalTracing(pinList);
                    result = "Started digital tracing for pins: " + pinList;}
                } else {
                    result = "Unknown command: " + cmd;
                }

                Serial.println(result);

                // Send the result back to the server
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
