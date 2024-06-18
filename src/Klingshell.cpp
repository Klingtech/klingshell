#include "KlingShell.h"

#ifdef ESP8266
#include "platform/esp8266/klingshell_esp8266.h"
#endif

#ifndef ESP8266
#include "platform/esp32/klingshell_esp32.h"
#endif


/**
 * @brief Get the battery percentage based on the measured voltage.
 * 
 * @param maxVoltage The maximum voltage of the battery.
 * @param pin The analog pin used to measure the voltage.
 * @return float The calculated battery percentage.
 */
float KlingShellClass::getBatteryPercentage(float maxVoltage, int pin) {
    // Set pin as analog input
    pinMode(pin, INPUT);

    int adcValue;                 // Variable to store ADC value
    float measuredVoltage;        // Variable to store measured voltage
    float referenceVoltage = 3.3; // Reference voltage for ESP8266/ESP32 ADC

#ifdef ESP8266
    adcValue = analogRead(pin); // ESP8266 has 10-bit ADC (0-1023)
    measuredVoltage = (adcValue / 1024.0) * referenceVoltage;
#else
    int millivolts = analogReadMilliVolts(pin); // ESP32 function directly gives millivolts
    measuredVoltage = millivolts / 1000.0; // Convert millivolts to volts
#endif

    // Calculate actual battery voltage
    // Assuming a voltage divider, the measured voltage needs to be scaled up to the battery voltage.
    float batteryVoltage = measuredVoltage * 2; // Assuming a 1:1 voltage divider

    // Map battery voltage to percentage
    float percentage = 0;
    if (batteryVoltage >= 4.2) {
        percentage = 100.0;
    } else if (batteryVoltage >= 4.1) {
        percentage = 90.0 + (batteryVoltage - 4.1) * 100.0;
    } else if (batteryVoltage >= 4.0) {
        percentage = 80.0 + (batteryVoltage - 4.0) * 100.0;
    } else if (batteryVoltage >= 3.9) {
        percentage = 70.0 + (batteryVoltage - 3.9) * 100.0;
    } else if (batteryVoltage >= 3.8) {
        percentage = 60.0 + (batteryVoltage - 3.8) * 100.0;
    } else if (batteryVoltage >= 3.7) {
        percentage = 50.0 + (batteryVoltage - 3.7) * 100.0;
    } else if (batteryVoltage >= 3.6) {
        percentage = 40.0 + (batteryVoltage - 3.6) * 100.0;
    } else if (batteryVoltage >= 3.5) {
        percentage = 30.0 + (batteryVoltage - 3.5) * 100.0;
    } else if (batteryVoltage >= 3.4) {
        percentage = 20.0 + (batteryVoltage - 3.4) * 100.0;
    } else if (batteryVoltage >= 3.3) {
        percentage = 10.0 + (batteryVoltage - 3.3) * 100.0;
    } else if (batteryVoltage >= 3.2) {
        percentage = (batteryVoltage - 3.2) * 100.0;
    } else {
        percentage = 0.0;
    }

    percentage = constrain(percentage, 0.0, 100.0);

    return percentage;
}

String KlingShellClass::getNRF52BLEMACAddress() {
    typedef volatile uint32_t REG32;
    #define pREG32 (REG32 *)
    #define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
    #define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))
    #define BLE_MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
    #define BLE_MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))

    uint32_t addr_high = ((BLE_MAC_ADDRESS_HIGH) & 0x0000ffff) | 0x0000c000;
    uint32_t addr_low  = BLE_MAC_ADDRESS_LOW;

    char BLEmacAddress[18];  // Buffer for MAC address string

    snprintf(BLEmacAddress, sizeof(BLEmacAddress), "%02X:%02X:%02X:%02X:%02X:%02X",
             (addr_high >> 8) & 0xFF,
             (addr_high) & 0xFF,
             (addr_low >> 24) & 0xFF,
             (addr_low >> 16) & 0xFF,
             (addr_low >> 8) & 0xFF,
             (addr_low) & 0xFF);
    return String(BLEmacAddress);
}

/**
 * @brief Scan for I2C devices on the bus and print their addresses.
 */
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

/**
 * @brief List all files stored on the device.
 */
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

/**
 * @brief Read the contents of a file and print it.
 * 
 * @param path The path to the file.
 */
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

/**
 * @brief Write a message to a specified file.
 * 
 * @param path The path to the file.
 * @param message The message to write.
 */
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

/**
 * @brief Delete a specified file.
 * 
 * @param path The path to the file.
 */
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

/**
 * @brief Scan for available Wi-Fi networks and print their SSIDs and signal strengths.
 */
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

/**
 * @brief Set the PWM duty cycle on a specified pin.
 * 
 * @param pin The pin to set the PWM duty cycle on.
 * @param dutyCycle The duty cycle (0-255).
 */
void KlingShellClass::setPWM(int pin, int dutyCycle) {
#ifndef ESP8266
    #ifndef ESP32_C6
    ledcAttachPin(pin, 0);
    ledcSetup(0, 5000, 8);
    ledcWrite(0, dutyCycle);
    #endif
#elif defined(ESP8266)
    analogWrite(pin, dutyCycle);
#endif
}

/**
 * @brief Report the current states of all digital and analog pins.
 */
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

/**
 * @brief Start tracing analog values on specified pins.
 * 
 * @param pinList The list of pins to trace.
 */
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

/**
 * @brief Start tracing digital values on specified pins.
 * 
 * @param pinList The list of pins to trace.
 */
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

/**
 * @brief Trace the values on analog or digital pins.
 * 
 * @param isAnalog If true, trace analog pins; otherwise, trace digital pins.
 */
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

/**
 * @brief Stop tracing analog pins.
 */
void KlingShellClass::stopAnalogTracing() {
    analogTracing = false;
    tracedAnalogPins.clear();
}

/**
 * @brief Stop tracing digital pins.
 */
void KlingShellClass::stopDigitalTracing() {
    digitalTracing = false;
    tracedDigitalPins.clear();
}

/**
 * @brief Get detailed system information.
 * 
 * @return String The system information.
 */
String KlingShellClass::getSystemInfo() {
    String info;

#ifndef ESP8266
    #ifndef ESP32_C6
    String wifiMac = WiFi.macAddress();
    info += "WiFi MAC Address: " + wifiMac + "\n";
    uint8_t bleMac[6];
    esp_read_mac(bleMac, ESP_MAC_BT);
    char bleMacStr[18];
    #ifndef NRF52840_XXAA
    sprintf(bleMacStr, "%02X:%02X:%02X:%02X:%02X:%02X", bleMac[0], bleMac[1], bleMac[2], bleMac[3], bleMac[4], bleMac[5]);
    info += "BLE MAC Address: " + String(bleMacStr) + "\n";
    #endif
    #ifdef NRF52840_XXAA
    info += "Test BLE MAC Address: " + String(getBLEMACAddress()) + "\n";
    #endif
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
    #endif
#elif defined(ESP8266)
    String wifiMac = WiFi.macAddress();
    info += "WiFi MAC Address: " + wifiMac + "\n";
    info += "CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz\n";
    info += "Free Heap Size: " + String(ESP.getFreeHeap()) + " bytes\n";
    info += "SDK Version: " + String(ESP.getSdkVersion()) + "\n";
#endif

    return info;
}

/**
 * @brief Get help information for all available commands.
 * 
 * @return String The help information.
 */
String KlingShellClass::getHelp() {
    String help = "---------------------------------------------------------------------------\n";
    help += "Available Commands (case-insensitive):\n";
    help += "\nPin Control:\n";
    help += "  ar<pin#>      - Read analog value from pin (e.g., 'ar32')\n";
    help += "  dr<pin#>      - Read digital value (HIGH/LOW) from pin (e.g., 'dr13')\n";
    help += "  pwm<pin#>|<duty> - Set PWM duty cycle (0-255) on pin (e.g., 'pwm23|128')\n";
    help += "  setpin<pin#>|<mode> - Set pin mode (e.g., 'setpin13|OUTPUT')\n";
    help += "  toggle<pin#>  - Toggle the state of a digital pin (e.g., 'toggle13')\n";
    help += "  tpa<pins>     - Trace analog values continuously (e.g., 'tpa32,33')\n";
    help += "  tpa stop      - Stop analog tracing\n";
    help += "  tpd<pins>     - Trace digital values continuously (e.g., 'tpd0,12')\n";
    help += "  tpd stop      - Stop digital tracing\n";
    help += "  ra            - Report ALL current pin states (analog & digital)\n";
    help += "  anascale<pin#>|<scale> - Read and scale analog value (e.g., 'anascale32|2.5')\n";

    help += "\nFile System (SPIFFS):\n";
    help += "  lf            - List files stored on the device\n";
    help += "  rf<path>      - Read contents of a file (e.g., 'rf/data.txt')\n";
    help += "  wf<path>|<data> - Write data to a file (e.g., 'wf/log.txt|Hello!')\n";
    help += "  df<path>      - Delete a file (e.g., 'df/old.log')\n";

    help += "\nDiagnostics & Information:\n";
    help += "  i2c           - Scan for I2C devices on the bus\n";
    help += "  wifi          - Scan for available Wi-Fi networks\n";
    help += "  info          - Get detailed system information (CPU, memory, etc.)\n";
    help += "  ip            - Display the IP configuration of the device\n";
    help += "  ping<target>  - Ping a target IP or DNS (e.g., 'ping www.google.com')\n";
    help += "  bat           - Display the battery percentage assuming pin A0 on ESP8266 or GPIO14 for ESP32 with 3.3v and 220k resistors.\n";
    help += "  bat|<pin#>|<maxV> - Customize battery reading with battery voltage (e.g., 'bat|14|4.2')\n";

    help += "\nOther:\n";
    help += "  reset         - Restart the device\n";
    help += "  help or ?     - Show this help message\n";
    help += "---------------------------------------------------------------------------\n";
    return help;
}

/**
 * @brief Check for incoming commands from the server and execute them.
 */
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
                } else if (cmd.startsWith("anascale")) {
                    int firstSeparator = cmd.indexOf('|');
                    if (firstSeparator != -1) {
                        int pin = cmd.substring(8, firstSeparator).toInt();
                        float scale = cmd.substring(firstSeparator + 1).toFloat();
                        readAnalogScaled(pin, scale);
                        result = "Reading and scaling analog value from pin " + String(pin);
                    } else {
                        result = "Invalid anascale command format";
                    }
                } else if (cmd.startsWith("toggle")) {
                    int pin = cmd.substring(6).toInt();
                    togglePin(pin);
                    result = "Toggling pin " + String(pin);
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
                    float maxVoltage = 4.2; // Default max voltage for a typical Li-Ion battery
                    int pin = 0; // Default pin

                    int separator = cmd.indexOf('|');
                    if (separator != -1) {
                        pin = cmd.substring(3, separator).toInt();
                        maxVoltage = cmd.substring(separator + 1).toFloat();
                    } else {
                        pin = cmd.substring(3).toInt();
                    }

                #ifdef ESP8266
                    pin = A0; // ESP8266 typically uses A0 for analog read
                #endif

                    KlingShell.println("Checking battery with settings: Max Voltage: " + String(maxVoltage) + " Pin: " + String(pin));
                    float percentage = getBatteryPercentage(maxVoltage, pin);
                    result = "Battery Percentage: " + String(percentage) + "%";
                } else if (cmd.startsWith("setpin")) {
                    int separator = cmd.indexOf('|');
                    if (separator != -1) {
                        int pin = cmd.substring(6, separator).toInt();
                        String mode = cmd.substring(separator + 1);
                        setPinMode(pin, mode);
                        result = "Setting pin " + String(pin) + " to mode " + mode;
                    } else {
                        result = "Invalid setpin command format";
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

/**
 * @brief Get the IP configuration of the device.
 * 
 * @return String The IP configuration details.
 */
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

/**
 * @brief Ping a host and return the results.
 * 
 * @param host The host to ping.
 * @param count The number of ping attempts.
 * @return String The ping results.
 */
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

/**
 * @brief Set the mode of a specified pin.
 * 
 * @param pin The pin number.
 * @param mode The mode to set (e.g., "INPUT", "OUTPUT").
 */
void KlingShellClass::setPinMode(int pin, String mode) {
    if (mode == "INPUT") {
        pinMode(pin, INPUT);
        KlingShell.println("Set pin " + String(pin) + " to INPUT mode.");
    } else if (mode == "OUTPUT") {
        pinMode(pin, OUTPUT);
        KlingShell.println("Set pin " + String(pin) + " to OUTPUT mode.");
    } else if (mode == "INPUT_PULLUP") {
        pinMode(pin, INPUT_PULLUP);
        KlingShell.println("Set pin " + String(pin) + " to INPUT_PULLUP mode.");
    } else if (mode == "INPUT_PULLDOWN") {
        #ifdef INPUT_PULLDOWN
        pinMode(pin, INPUT_PULLDOWN);
        KlingShell.println("Set pin " + String(pin) + " to INPUT_PULLDOWN mode.");
        #else
        KlingShell.println("INPUT_PULLDOWN mode is not supported on this platform.");
        #endif
    } else {
        KlingShell.println("Invalid mode: " + mode);
    }
}

/**
 * @brief Toggle the state of a specified digital pin.
 * 
 * @param pin The pin number to toggle.
 */
void KlingShellClass::togglePin(int pin) {
    int state = digitalRead(pin);
    if (state == HIGH) {
        digitalWrite(pin, LOW);
        KlingShell.println("Pin " + String(pin) + " set to LOW.");
    } else {
        digitalWrite(pin, HIGH);
        KlingShell.println("Pin " + String(pin) + " set to HIGH.");
    }
}

/**
 * @brief Read an analog value, scale it, and print the result.
 * 
 * @param pin The analog pin to read.
 * @param scale The scale factor to apply to the reading.
 */
void KlingShellClass::readAnalogScaled(int pin, float scale) {
    int rawValue = analogRead(pin);
    float scaledValue = (rawValue / 1024.0) * scale;
    KlingShell.println("Analog pin " + String(pin) + " value: " + String(scaledValue) + " (scaled)");
}

