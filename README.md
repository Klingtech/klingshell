# KlingShell

KlingShell is a comprehensive firmware solution designed for ESP8266 and ESP32 microcontroller platforms. It provides robust support for remote shell access, GPIO pin management, over-the-air (OTA) updates, Wi-Fi connectivity, and various diagnostics, making it ideal for IoT projects that require remote monitoring and control without the need for reflashing the firmware.

![image](https://github.com/Klingtech/klingshell/assets/151225560/0dd9fe09-9884-4ffd-80bf-306a60b4e18d)

## Important Note

**This is an early, unsecure version primarily intended for testing and learning purposes.** We are sharing this project to contribute to the community and hopefully get some good ideas and feedback. Please do not use this version in production environments.


## Prerequisites

### Hardware

- A compatible ESP8266 or ESP32 board.

### Software

- [PlatformIO](https://platformio.org/) installed on your system.
- [Node.js](https://nodejs.org/) installed on your system.

### Server Setup

KlingShell requires a server running on port 80 to receive and send commands. This server can be run using Node.js. The server files are located in the `/server` folder, and the main file to start the server is `server.js`.

## Dependencies

### Common Libraries

- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [NTPClient](https://github.com/arduino-libraries/NTPClient)
- [Wire](https://www.arduino.cc/en/Reference/Wire)

### ESP32 Specific Libraries

- [WiFi](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi)
- [ArduinoOTA](https://github.com/espressif/arduino-esp32/tree/master/libraries/ArduinoOTA)
- [ESPmDNS](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESPmDNS)
- [HTTPClient](https://github.com/espressif/arduino-esp32/tree/master/libraries/HTTPClient)
- [WiFiClientSecure](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFiClientSecure)
- [SPIFFS](https://github.com/espressif/arduino-esp32/tree/master/libraries/SPIFFS)
- [ESP32Ping](https://github.com/marian-craciunescu/ESP32Ping) - This is a forked repository that is actively used and maintained.

### ESP8266 Specific Libraries

- [ESP8266WiFi](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi)
- [ESP8266mDNS](https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS)
- [ArduinoOTA](https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA)
- [LittleFS](https://github.com/earlephilhower/LittleFS)
- [Ticker](https://github.com/esp8266/Arduino/tree/master/libraries/Ticker)
- [ESP8266Ping](https://github.com/dancol90/ESP8266Ping) - A reliable library for ping functionality on the ESP8266.

## Features

- **Remote Shell Access**: Execute commands remotely on your ESP8266/ESP32 device.
- **GPIO Management**: Read and write digital and analog values to GPIO pins.
- **Wi-Fi Connectivity**: Connect to Wi-Fi networks and manage network settings.
- **OTA Updates**: Seamlessly update firmware over-the-air.
- **File Management**: Manage files stored in SPIFFS or LittleFS.
- **Diagnostics**: Retrieve system information, scan I2C devices, and perform other diagnostic tasks.
- **Pin Tracing**: Continuously trace and log analog and digital pin values.
- **Battery Monitoring**: Calculate and report battery percentage based on voltage readings.

## Supported Platforms

- **ESP32**: Various models including ESP32, ESP32S2, ESP32S3, ESP32C3, and ESP32H2.
- **ESP8266**: Supported with specific GPIO configurations.

## Getting Started

### Installation

1. **Clone the repository:**

2. **Open the project in PlatformIO IDE:**
   - Launch the PlatformIO IDE.
   - Open the `KlingShell` project directory.

3. **Configure your board:**
   - Edit the `platformio.ini` file to select your target board. Example for different ESP32 and ESP8266:
     ```ini
     [platformio]
     default_envs = esp32
     description = Klingshell

     [env]
     lib_deps = 
         ArduinoJson
         NTPClient
         Wire

     [env:esp32]
     platform = espressif32
     board = esp32dev
     framework = arduino
     lib_deps = 
         ${env.lib_deps}
         WiFi
         ArduinoOTA
         ESPmDNS
         HTTPClient
         WiFiClientSecure
         SPIFFS
         ESP32Ping

     [env:esp8266]
     platform = espressif8266
     board = esp12e
     framework = arduino
     lib_deps = 
         ${env.lib_deps}
         ESP8266WiFi
         ESP8266mDNS
         ArduinoOTA
         LittleFS
         Ticker
         ESP8266Ping

     [env:ESP32_C3_XIAO]
     platform = espressif32
     board = seeed_xiao_esp32c3
     framework = arduino
     upload_speed = 256000
     monitor_speed = 115200
     lib_deps = ${env:esp32.lib_deps}

     [env:ESP32_C3_DEVKIT]
     platform = espressif32
     board = esp32-c3-devkitc-02
     framework = arduino
     upload_speed = 256000
     monitor_speed = 115200
     lib_deps = ${env:esp32.lib_deps}

     [env:ESP32_S3_XIAO]
     platform = espressif32
     board = seeed_xiao_esp32s3
     framework = arduino
     monitor_speed = 115200
     lib_deps = ${env:esp32.lib_deps}

     [env:ESP8266_Wemos_Battery]
     platform = espressif8266
     board = esp12e
     framework = arduino
     upload_speed = 256000
     monitor_speed = 115200
     lib_deps = ${env:esp8266.lib_deps}

     [env:ESP32_ATOM_LITE]
     platform = espressif32
     board = m5stack-atom
     framework = arduino
     monitor_speed = 115200
     lib_deps = ${env:esp32.lib_deps}
     ```

4. **Add your Wi-Fi credentials and server address:**
   - Create a `credentials.h` file in the `src` directory with your Wi-Fi details:
     ```cpp
     const char *ssid = "your_ssid";
     const char *password = "your_password";
     const char *server = "http://your.server.address:10000/KlingShell";
     ```

5. **Build and upload the firmware:**
   - Connect your board to the computer.
   - Build and upload the firmware using PlatformIO.

5. **Start a webbrowser and browse to your servers ip and port!**
 
### Usage

Once the firmware is running on your ESP8266/ESP32 device, you can connect to it and execute various commands remotely through the KlingShell interface. This allows for easy testing, monitoring, and control of the device without the need for constant reflashing.

#### Available Commands

- **Pin Control:**
  - `ar<pin#>`: Read analog value from pin.
  - `dr<pin#>`: Read digital value (HIGH/LOW) from pin.
  - `pwm<pin#>|<duty>`: Set PWM duty cycle (0-255) on pin.
  - `tpa<pins>`: Trace analog values continuously.
  - `tpd<pins>`: Trace digital values continuously.
  - `ra`: Report ALL current pin states.

- **File System (SPIFFS):**
  - `lf`: List files stored on the device.
  - `rf<path>`: Read contents of a file.
  - `wf<path>|<data>`: Write data to a file.
  - `df<path>`: Delete a file.

- **Diagnostics & Information:**
  - `i2c`: Scan for I2C devices on the bus.
  - `wifi`: Scan for available Wi-Fi networks.
  - `info`: Get detailed system information (CPU, memory, etc.).
  - `bat`: Display the battery percentage.
  - `bat|<maxV>|<R1>|<R2>`: Customize battery reading.

- **Other:**
  - `reset`: Restart the device.
  - `help` or `?`: Show the help message.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request with any improvements or bug fixes.

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License**. This means you can share and adapt the material for non-commercial purposes, as long as you provide attribution and share any derivative works under the same license.

For more details, see the [LICENSE](LICENSE) file.

## Acknowledgments

- [PlatformIO](https://platformio.org/) for providing a powerful development environment for embedded systems.
- [Arduino](https://www.arduino.cc/) for the foundational libraries and frameworks.
