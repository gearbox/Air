#include "Arduino.h"

#include <FS.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

// Wifi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

// HTTP requests
#include <ESP8266HTTPClient.h>

// OTA updates
#include <ESP8266httpUpdate.h>

// Blynk
#include <BlynkSimpleEsp8266.h>
// Set Blynk token and server
#include "blynk_setup.h"
// Keep this flag not to re-sync on every reconnection
bool isFirstConnect = true;
WidgetTerminal lcd(V0); //Init Blynk LCD widget for debug information
WidgetLED led_b(V23);
WidgetLED led_r(V24);
WidgetLED led_g(V25);

// JSON
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <Wire.h>

// GPIO Defines
#define I2C_SDA 0 // D1 Yellow
#define I2C_SCL 2 // D2 Orange

// Humidity/Temperature
#include <SimpleDHT.h>
int pinDHT11 = 10;
SimpleDHT11 dht11;

// Pressure and Temperature
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bme;

// Handy timers
#include <SimpleTimer.h>
SimpleTimer timer;

// CO2 SERIAL
#define DEBUG_SERIAL Serial1
#define SENSOR_SERIAL Serial
//#define HIGH_CO2 1000 //High concentration of CO2 - Alert
int HIGH_CO2 = 0;
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
unsigned char response[7];

// Device Id
char device_id[17] = "Air";
const char fw_ver[17] = "0.2.1";

// Stringifying the BUILD_TAG parameter
#define TEXTIFY(A) #A
#define ESCAPEQUOTE(A) TEXTIFY(A)
String buildTag = ESCAPEQUOTE(BUILD_TAG);

// Setup Wifi connection
WiFiManager wifiManager;

// Network credentials
String ssid { "air_" +  String(ESP.getChipId())};
String pass {"air_" + String(ESP.getChipId())};

//flag for saving data
bool shouldSaveConfig = false;

//Builin led (breathe)
#ifndef LED_BUILTIN
#define LED_BUILTIN 16 //Led in NodeMCU at pin GPIO16 (D0)
#endif
#define BRIGHT    350     //max led intensity (1-500)
#define INHALE    1250    //Inhalation time in milliseconds.
#define PULSE     INHALE*1000/BRIGHT
#define REST      2000    //Rest Between Inhalations.

// Sensors data
byte humidity = -1, t_dht11 = -100; // Values read from DHT11 sensor
float t {-100}; //Average Temperature
float t_bmp180 {-100}; // Temperature read from BMP180 sensor
int p {-1}; // Atmospheric Pressure read from BMP180 sensor
int co2 {-1}; // CO2 read from MH-Z19 sensor

char loader[4] {'.'};

//callback notifying the need to save config
void saveConfigCallback() {
        //DEBUG_SERIAL.println("Should save config");
        lcd.println("DBG: Should save config");
        shouldSaveConfig = true;
}

void factoryReset() {
        wifiManager.resetSettings();
        SPIFFS.format();
        ESP.reset();
}

void printString(String str) {
        //DEBUG_SERIAL.println(str);
        lcd.println("DBG: "+str);
}

void breathe() {
  //Builtin Led breathe function
  //ramp increasing intensity, Inhalation:
  for (int i=1;i<BRIGHT;i++){
    digitalWrite(LED_BUILTIN, LOW);          // turn the LED on.
    delayMicroseconds(i*10);         // wait
    digitalWrite(LED_BUILTIN, HIGH);         // turn the LED off.
    delayMicroseconds(PULSE-i*10);   // wait
    delay(0);                        //to prevent watchdog firing.
  }
  //ramp decreasing intensity, Exhalation (half time):
  for (int i=BRIGHT-1;i>0;i--){
    digitalWrite(LED_BUILTIN, LOW);          // turn the LED on.
    delayMicroseconds(i*10);          // wait
    digitalWrite(LED_BUILTIN, HIGH);         // turn the LED off.
    delayMicroseconds(PULSE-i*10);  // wait
    i--;
    delay(0);                        //to prevent watchdog firing.
  }
  delay(REST);                       //take a rest...
}

void readCO2() {
        // CO2
        bool header_found {false};
        char tries {0};

        SENSOR_SERIAL.write(cmd, 9);
        memset(response, 0, 7);

        // Looking for packet start
        while(SENSOR_SERIAL.available() && (!header_found)) {
                if(SENSOR_SERIAL.read() == 0xff ) {
                        if(SENSOR_SERIAL.read() == 0x86 ) header_found = true;
                }
        }

        if (header_found) {
                SENSOR_SERIAL.readBytes(response, 7);

                byte crc = 0x86;
                for (char i = 0; i < 6; i++) {
                        crc+=response[i];
                }
                crc = 0xff - crc;
                crc++;

                if ( !(response[6] == crc) ) {
                        //DEBUG_SERIAL.println("CO2: CRC error: " + String(crc) + " / "+ String(response[6]));
                } else {
                        unsigned int responseHigh = (unsigned int) response[0];
                        unsigned int responseLow = (unsigned int) response[1];
                        unsigned int ppm = (256*responseHigh) + responseLow;
                        co2 = ppm;
                        //DEBUG_SERIAL.println("CO2:" + String(co2));
                }
        } else {
                //DEBUG_SERIAL.println("CO2: Header not found");
        }

}

float roundResult_f(float res) {
  res = (int)(res * 100 + (res >= 0 ? 0.5 : -0.5)) / 100.0;
  return res;
}

void sendMeasurements() {
        // Read data

        // Temperature in C & Humidity in %
        if (dht11.read(pinDHT11, &t_dht11, &humidity, NULL)) {
          Serial.print("Read DHT11 failed.");
        }

        // Temperature in C & Pressure (in mmHg)
        t_bmp180 = bme.readTemperature(); //Read the Temp from BMP180
        p = static_cast<int>(bme.readPressure() * 760.0 / 101325);

        if (!isnan(t_dht11) && !isnan(t_bmp180)) {
          t_dht11 = roundResult_f(t_dht11);
          t_bmp180 = roundResult_f(t_bmp180);
          t = roundResult_f((t_dht11 + t_bmp180) / 2); //Average Temp
        }

        // CO2
        readCO2();

        // Send to server
        if (humidity > -1 && humidity < 100 && t_dht11 > -100) {
          Blynk.virtualWrite(V1, t_dht11);
          Blynk.virtualWrite(V2, humidity);
        }
        if (t_bmp180 > -100 && p > -1){
          Blynk.virtualWrite(V3, t_bmp180);
          Blynk.virtualWrite(V4, p);
        }
        if (co2 > -1){
          Blynk.virtualWrite(V5, co2);
        }
        if (t > -100){
          Blynk.virtualWrite(V6, t);
        }
        /*
        // Write to debug console
        printString("H: " + String(humidity) + "%");
        printString("T: " + String(t) + "C");
        printString("P: " + String(p) + "mmHg");
        printString("CO2: " + String(co2) + "ppm");
        */
}

void setBlinkProperty() {
  Blynk.setProperty(V1, "label", "Temperature (DHT11), °C");
  Blynk.setProperty(V2, "label", "Humidity, %");
  Blynk.setProperty(V3, "label", "Temperature (BMP180), °C");
  Blynk.setProperty(V4, "label", "Pressure, mmHg");
  Blynk.setProperty(V5, "label", "CO2, ppm");
  Blynk.setProperty(V6, "label", "Average Temperature, °C");
}

void loading() {
        long unsigned int count {(millis() / 500) % 4};
        memset(loader, '.', count);
        memset(&loader[count], 0, 1);
}

bool loadConfig() {
        File configFile = SPIFFS.open("/config.json", "r");
        if (!configFile) {
                //DEBUG_SERIAL.println("Failed to open config file");
                lcd.println("DBG: Failed to open config file");
                return false;
        }

        size_t size = configFile.size();
        if (size > 1024) {
                //DEBUG_SERIAL.println("Config file size is too large");
                lcd.println("DBG: Config file size is too large");
                return false;
        }

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        // We don't use String here because ArduinoJson library requires the input
        // buffer to be mutable. If you don't use ArduinoJson, you may as well
        // use configFile.readString instead.
        configFile.readBytes(buf.get(), size);

        StaticJsonBuffer<200> jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());

        if (!json.success()) {
                //DEBUG_SERIAL.println("Failed to parse config file");
                lcd.println("DBG: Failed to parse config file");
                return false;
        }

        // Save parameters
        strcpy(device_id, json["device_id"]);
        strcpy(blynk_token, json["blynk_token"]);
}

void configModeCallback (WiFiManager *wifiManager) {
        String url {"http://192.168.4.1"};
        printString("Connect to WiFi:");
        printString("net: " + ssid);
        printString("pw: "+ pass);
        printString("Open browser:");
        printString(url);
        printString("to setup device");

        // drawConnectionDetails(ssid, pass, url);
        lcd.println("WiFi:");
        lcd.println(ssid);
        lcd.println(pass);
}

void setupWiFi() {
        //set config save notify callback
        wifiManager.setSaveConfigCallback(saveConfigCallback);

        // Custom parameters
        WiFiManagerParameter custom_device_id("device_id", "Device name", device_id, 16);
        WiFiManagerParameter custom_blynk_server("blynk_server", "Blynk server", blynk_server, 64);
        WiFiManagerParameter custom_blynk_token("blynk_token", "Blynk token", blynk_token, 34);
        wifiManager.addParameter(&custom_blynk_server);
        wifiManager.addParameter(&custom_blynk_token);
        wifiManager.addParameter(&custom_device_id);

        // wifiManager.setTimeout(180);
        wifiManager.setAPCallback(configModeCallback);

        if (!wifiManager.autoConnect(ssid.c_str(), pass.c_str())) {
                //DEBUG_SERIAL.println("failed to connect and hit timeout");
        }

        //save the custom parameters to FS
        if (shouldSaveConfig) {
                //DEBUG_SERIAL.println("saving config");
                DynamicJsonBuffer jsonBuffer;
                JsonObject &json = jsonBuffer.createObject();
                json["device_id"] = custom_device_id.getValue();
                json["blynk_server"] = custom_blynk_server.getValue();
                json["blynk_token"] = custom_blynk_token.getValue();

                File configFile = SPIFFS.open("/config.json", "w");
                if (!configFile) {
                        //DEBUG_SERIAL.println("failed to open config file for writing");
                }

                //json.printTo(DEBUG_SERIAL);
                json.printTo(configFile);
                configFile.close();
                //end save
        }

        //if you get here you have connected to the WiFi
        /*DEBUG_SERIAL.println("WiFi connected");

        DEBUG_SERIAL.print("IP address: ");
        DEBUG_SERIAL.println(WiFi.localIP());*/
}

//
BLYNK_WRITE(V15) {
  HIGH_CO2 = param.asInt();
}

// Virtual pin update FW
BLYNK_WRITE(V22) {
//        if (param.asInt() == 1) {
                //DEBUG_SERIAL.println("Got a FW update request");
                char full_version[34] {""};
                strcat(full_version, device_id);
                strcat(full_version, "::");
                strcat(full_version, fw_ver);
                led_b.on();

                t_httpUpdate_return ret = ESPhttpUpdate.update("http://geariot-air.appspot.com/update/fw.bin", full_version);
                //t_httpUpdate_return ret = ESPhttpUpdate.update("http://geariot-air.appspot.com/update/fw.bin");
                //t_httpUpdate_return ret = ESPhttpUpdate.update("http://geariot-air.appspot.com", 80, "/update/fw.bin");

                switch (ret) {
                case HTTP_UPDATE_FAILED:
                        //DEBUG_SERIAL.println("[update] Update failed.");
                        led_r.on();
                        lcd.println("Update failed");

                        break;
                case HTTP_UPDATE_NO_UPDATES:
                        //DEBUG_SERIAL.println("[update] Update no Update.");
                        led_b.on();
                        led_r.on();
                        led_g.on();
                        lcd.println("Update no update");
                        break;
                case HTTP_UPDATE_OK:
                        //DEBUG_SERIAL.println("[update] Update ok.");
                        led_g.on();
                        lcd.println("Update Ok!");
                        break;
                }

//        }
}


// Virtual pin reset settings
BLYNK_WRITE(V23) {
        factoryReset();
}

BLYNK_CONNECTED() {
  if (isFirstConnect) {
    //Blynk.syncAll();
    Blynk.syncVirtual(V15);
  }
  isFirstConnect = false;
}

void setup() {
        // initialize LED digital pin as an output.
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);

        // Init serial ports
        //DEBUG_SERIAL.begin(115200);
        SENSOR_SERIAL.begin(9600);
        SENSOR_SERIAL.swap();  // to use GPIO15 (TX) and GPIO13 (RX) instead of GPIO01 (TX) and GPIO03 (RX)

        // Init I2C interface
        Wire.begin(I2C_SDA, I2C_SCL);

        // Init Pressure/Temperature sensor
        if (!bme.begin()) {
                //DEBUG_SERIAL.println("Could not find a valid BMP085 sensor, check wiring!");
        }

        // Init filesystem
        if (!SPIFFS.begin()) {
                //DEBUG_SERIAL.println("Failed to mount file system");
                ESP.reset();
        }

        // Setup WiFi
        setupWiFi();

        // Load config
        if (!loadConfig()) {
                //DEBUG_SERIAL.println("Failed to load config");
                factoryReset();
        } else {
                //DEBUG_SERIAL.println("Config loaded");
        }

        // Start blynk
        Blynk.config(blynk_token, blynk_server, blynk_port);

        // Setup a function to be called every 15 second
        timer.setInterval(15000L, sendMeasurements);

        setBlinkProperty();

        lcd.println("Gear IoT Air: "+buildTag);
}

void loop() {
        Blynk.run();
        timer.run();
        if (HIGH_CO2 > 500) {
          if (co2 > HIGH_CO2){
            breathe();
          }
        }
}
