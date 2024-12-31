#include <Wire.h>
#include <BH1750.h>
#include <SensirionI2cScd30.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include "DFRobot_RTU.h"
#include "ShiftRegister74HC595_NonTemplate.h"
#include "pin_config.h"                // File konfigurasi untuk pin-pin yang mcu dan relay yang terhubung ke ic HT74HC595
// #include <iostream>                    // Library untuk fungsi I/O standar C++
// #include <memory> 

// Konfigurasi WiFi
const char ssid[] = "SMART GREENHOUSE UNINUS (2Ghz)";
const char pass[] = "UNINUSLAB530";

// Konfigurasi MQTT
const char* mqtt_server = "88.222.214.56";
const int mqtt_port = 1883;
const char* mqtt_topic = "SmartGreenhouse/SensorData";
const char* mqtt_username = "SGH_1.0";
const char* mqtt_password = "Teknik_Pertanian_2025";
const char* clientID = "Smartroomgreenhouse_Uninus";

// MQTT client setup
WiFiClient net;
MQTTClient mqttClient;

// Define the serial communication pins for Modbus
#define RXD 36
#define TXD 37

// DHT22 configuration
#define DHTPIN 18
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Initialize LCD 20x4 with I2C address
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Create objects for sensors and Modbus
BH1750 lightMeter;
SensirionI2cScd30 sensor;

// === Definisi Tombol dan Relay ===
#define BUTTON_S1 12     // Tombol Blower
#define RELAY_BLOWER 0   // relay blowwer  
bool blowerState = false;
bool lastButtonS1State = HIGH;

float cooling_inactive = 0, cooling_active = 0;
String mode = "Manual";
String device;
std::shared_ptr<ShiftRegister74HC595_NonTemplate> HT74HC595 =
    std::make_shared<ShiftRegister74HC595_NonTemplate>(6, HT74HC595_DATA, HT74HC595_CLOCK, HT74HC595_LATCH);

// Mutex untuk RTOS
SemaphoreHandle_t xMutex;

DFRobot_RTU Modbus_Master(/*s =*/&Serial2);
uint16_t Read_Holding_Reg_temp1;
uint16_t Read_Holding_Reg_humi1;
uint16_t Read_Holding_Reg_temp2;
uint16_t Read_Holding_Reg_humi2;
uint16_t Read_Holding_Reg_temp3;
uint16_t Read_Holding_Reg_humi3;
uint16_t Read_Holding_Reg_temp4;
uint16_t Read_Holding_Reg_humi4;
uint16_t Read_Holding_Reg_temp5;
uint16_t Read_Holding_Reg_humi5;
uint16_t Read_Holding_Reg_temp6;
uint16_t Read_Holding_Reg_humi6;

static char errorMessage[128];
static int16_t error;

float scd30Temperature = 0.0;
float scd30Humidity = 0.0;
float avgtempall = 0.0;
float avgHumall = 0.0;

float panel_temp = 0.0;
float co2 =0.0;
int lightIntensity = 0.0;

// Deklarasi variabel untuk suhu dan kelembaban dari sensor Modbus (SHT20)
float temp1 = 0.0;
float temp2 = 0.0;
float temp3 = 0.0;
float temp4 = 0.0;
float temp5 = 0.0;
float temp6 = 0.0;

float humi1 = 0.0;
float humi2 = 0.0;
float humi3 = 0.0;
float humi4 = 0.0;
float humi5 = 0.0;
float humi6 = 0.0;

// Variabel untuk interval pembacaan sensor (500ms)
unsigned long previousMillis = 0;
const long interval = 500;  // interval untuk pembacaan sensor

// Variabel untuk interval restart otomatis setelah 12 jam (43200000 ms)
unsigned long previousRestartMillis = 0;
const long restartInterval = 43200000;  // 12 jam dalam milidetik (12 * 60 * 60 * 1000)


// // Function prototypes
// void displayDataOnLCD(void *pvParameters);
// void setupWiFi();
// void connectMQTT();
// void publishData();
// void TaskReadBH1750(void *pvParameters);
// void TaskReadSCD30(void *pvParameters);
// void TaskReadDHT22(void *pvParameters);
// void readSensorData(int slaveID);


// Initialize WiFi
void setupWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, pass);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime >= 10000) {  // 10 detik limit
            Serial.println("WiFi connection timeout");
            return;  // Keluar jika WiFi tidak terhubung dalam 10 detik
        }
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void mqttCallback(String &topic, String &payload) {
  // payload[length] = '\0'; // Null-terminate the payload string
  // String message = String((char * ) payload);
  Serial.println("incoming: " + topic + " - " + payload);
  // Debugging
  Serial.println("debug 0");

  // Parsing JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.f_str());
    return;
  }

  Serial.print("Debug 1");
  Serial.println(topic);

  if (String(topic) == "SmartGreenHouse/Settings") {
    Serial.println("Debug 2 Topic settings masuk");
    cooling_active = doc["Cooling_Sistem_Active"];
    cooling_inactive = doc["Cooling_Sistem_Inactive"];
    device = doc["device"].as < String > ();

    Serial.print("Coolling Active Limit: ");
    Serial.println(cooling_active);
    Serial.print("Coolling Inactive Limit: ");
    Serial.println(cooling_inactive);
    
  }

  // Cek apakah tipe pesan adalah 'control'
  if (String(topic) == "SmartGreenHouse/Control") {
    Serial.println("Debug 3 Topic control masuk");
  String type = doc["type"];
  String control = doc["control"];
  int status = doc["status"];
  String device = doc["device"];

  // Cek apakah tipe yang diterima adalah "control"
  if (type == "control") {
    if (control == "Cooling_System") {
      Serial.println("Debug 4 Data ON OF masuk");
      blowerState = (status == 1);
      HT74HC595 -> set(RELAY_BLOWER, blowerState ? HIGH : LOW, true);
      // sendPumpStatus("Pompa_PHUP", pumpPhUpState);
    }
  }else if (type == "mode"){
    mode = doc["mode"].as < String > ();
  }
  }
}


void connectMQTT() {
    while (WiFi.status() != WL_CONNECTED) {
        setupWiFi();
    }

    // Inisialisasi MQTT
  mqttClient.begin(mqtt_server, mqtt_port, net);
  mqttClient.onMessage(mqttCallback); // Callback biasa tanpa lambda

    // Coba koneksi ke broker
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect(clientID, mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT Broker!");
            mqttClient.subscribe("SmartGreenHouse/Control");
            mqttClient.subscribe("SmartGreenHouse/Settings");
        } else {
            Serial.print("Failed, error code: ");
            Serial.println(mqttClient.lastError());
            Serial.println("Retrying in 5 seconds...");
            delay(5000);
        }
    }
}


void sendPumpStatus(const char * pumpName, bool state, const char * type) {
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected. Skipping status update.");
    return;
  }

  StaticJsonDocument < 128 > jsonDoc;
  jsonDoc["type"] = type;
  jsonDoc["control"] = pumpName;
  jsonDoc["status"] = state ? 1 : 0;
  jsonDoc["device"] = "Greenhouse";

  char jsonBuffer[128];
  serializeJson(jsonDoc, jsonBuffer);

  if (mqttClient.publish("SmartGreenHouse/Control_Send", jsonBuffer)) {
    Serial.printf("Status %s terkirim: %s\n", pumpName, state ? "ON" : "OFF");
  } else {
    Serial.printf("Gagal mengirim status %s!\n", pumpName);
  }
}

void blowertask(void *pvParameters){
  const int debounceDelay = 50;
  static unsigned long lastDebounceTimeS1 = 0;
  for (;;) {
    unsigned long currentTime = millis();
    bool currentButtonS1State = digitalRead(BUTTON_S1);
    if (currentButtonS1State == LOW && lastButtonS1State == HIGH && (currentTime - lastDebounceTimeS1 > debounceDelay)) {
      blowerState = !blowerState;
      HT74HC595 -> set(RELAY_BLOWER, blowerState ? HIGH : LOW, true);
      sendPumpStatus("Cooling_System", blowerState, "status");
      lastDebounceTimeS1 = currentTime;
    }
    lastButtonS1State = currentButtonS1State;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Publish data to MQTT
void publishData() {
    StaticJsonDocument<512> jsonDoc;

    jsonDoc["suhu1"] = temp1;
    jsonDoc["kelembaban1"] = humi1;

    jsonDoc["suhu2"] = temp2;
    jsonDoc["kelembaban2"] = humi2;

    jsonDoc["suhu3"] = temp3;
    jsonDoc["kelembaban3"] = humi3;

    jsonDoc["suhu4"] = temp4;
    jsonDoc["kelembaban4"] = humi4;

    jsonDoc["suhu5"] = temp5;
    jsonDoc["kelembaban5"] = humi5;

    jsonDoc["suhu6"] = temp6;
    jsonDoc["kelembaban6"] = humi6;

    // Add SCD30 data as suhu7 and kelembaban7
    jsonDoc["suhu7"] = scd30Temperature;
    jsonDoc["kelembaban7"] = scd30Humidity;

    // Add averages
    jsonDoc["suhu_AVG"] = avgtempall;
    jsonDoc["kelembaban_AVG"] = avgHumall;

    // Add additional data
    jsonDoc["co2"] =  co2;
    jsonDoc["intensitas"] = lightIntensity;
    jsonDoc["panel_temp"] =  panel_temp;
    jsonDoc["device_id"] = 3;

    char jsonBuffer[512];
    serializeJson(jsonDoc, jsonBuffer);

    mqttClient.publish(mqtt_topic, jsonBuffer);
    Serial.println("Data published to MQTT:");
    Serial.println(jsonBuffer);
  //  } else {
  //           Serial.println("Reconnecting to MQTT...");
  //           mqttClient.connect(clientID, mqtt_username, mqtt_password);
  //       }

      //delay(5000); // Interval pengiriman data 5 detik
    
}

// Display data on LCD
void displayDataOnLCD(void* pvParameters) {
  for (;;){
    //if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    lcd.setCursor(0, 0);
    lcd.print("Avg Temp: ");
    lcd.print(avgtempall, 1);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Avg Hum : ");
    lcd.print(avgHumall, 1);
    lcd.print(" %");

    lcd.setCursor(0, 2);
    lcd.print("CO2     : ");
    lcd.print(co2, 0);
    lcd.print(" ppm");

    lcd.setCursor(0, 3);
    lcd.print("Light   : ");
    lcd.print(lightIntensity, 0);
    lcd.print(" lx");

    //  xSemaphoreGive(xMutex);
    // }
    
  vTaskDelay(pdMS_TO_TICKS(1000)); // Interval pembaruan LCD 1 detik
  }
}

void readsensorsht20(){
  float divider = 10;
//===========================================================================================================================

  //Pembacaan sensor SHT20 slave 1
  Read_Holding_Reg_temp1 = Modbus_Master.readInputRegister(1, 1) ;//(1 = Slave ID:1 , 0 = Address : 0 (D0) what is the value)
  float nil1 = Read_Holding_Reg_temp1 ;
  float temperature1 = nil1 / divider ;
  //Serial.print("Temperature1 = "); Serial.println(temperature1);
  temp1 = temperature1;
  delay(500);

  Read_Holding_Reg_humi1 = Modbus_Master.readInputRegister(1, 2) ;//(1 = Slave ID:1 , 0 = Address : 0 (D0) what is the value)
  float nil2 = Read_Holding_Reg_humi1;
  float humidity1 = nil2 / divider;
  //Serial.print("Humidity1 = "); Serial.println(humidity1);
  humi1= humidity1;
  delay(500);
//===========================================================================================================================

  //Pembacaan sensor SHT20 slave 2
  Read_Holding_Reg_temp2 = Modbus_Master.readInputRegister(2, 1) ;//(2 = Slave ID:2 , 0 = Address : 0 (D0) what is the value)
  float nil3 = Read_Holding_Reg_temp2 ;
  float temperature2 = nil3 / divider ;
  //Serial.print("Temperature2 = "); Serial.println(temperature2);
  temp2 = temperature2;
  delay(500);

  Read_Holding_Reg_humi2 = Modbus_Master.readInputRegister(2, 2) ;//(2 = Slave ID:2 , 0 = Address : 0 (D0) what is the value)
  float nil4 = Read_Holding_Reg_humi2;
  float humidity2 = nil4 / divider;
  //Serial.print("Humidity2 = "); Serial.println(humidity2);
  humi2 = humidity2;
  delay(500);
//===========================================================================================================================

  //Pembacaan sensor SHT20 slave 3
  Read_Holding_Reg_temp3 = Modbus_Master.readInputRegister(3, 1) ;//(3 = Slave ID:3 , 0 = Address : 0 (D0) what is the value)
  float nil5 = Read_Holding_Reg_temp3 ;
  float temperature3 = nil5 / divider ;
  //Serial.print("Temperature3 = "); Serial.println(temperature3);
  temp3 = temperature3;
  delay(500);

  Read_Holding_Reg_humi3 = Modbus_Master.readInputRegister(3, 2) ;//(3 = Slave ID:3 , 0 = Address : 0 (D0) what is the value)
  float nil6 = Read_Holding_Reg_humi3;
  float humidity3 = nil6 / divider;
  //Serial.print("Humidity3= "); Serial.println(humidity3);
  humi3 = humidity3;
  delay(500);
//===========================================================================================================================

  //Pembacaan sensor SHT20 slave 4
  Read_Holding_Reg_temp4 = Modbus_Master.readInputRegister(4, 1) ;//(4 = Slave ID:4 , 0 = Address : 0 (D0) what is the value)
  float nil7 = Read_Holding_Reg_temp4 ;
  float temperature4 = nil7 / divider ;
  //Serial.print("Temperature4 = "); Serial.println(temperature4);
  temp4= temperature4;
  delay(500);

  Read_Holding_Reg_humi4 = Modbus_Master.readInputRegister(4, 2) ;//(4 = Slave ID:4 , 0 = Address : 0 (D0) what is the value)
  float nil8 = Read_Holding_Reg_humi4;
  float humidity4 = nil8 / divider;
  //Serial.print("Humidity4 = "); Serial.println(humidity4);
  humi4 = humidity4;
  delay(500);
//===========================================================================================================================

  //Pembacaan sensor SHT20 slave 5
  Read_Holding_Reg_temp5 = Modbus_Master.readInputRegister(5, 1) ;//(5 = Slave ID:5 , 0 = Address : 0 (D0) what is the value)
  float nil9 = Read_Holding_Reg_temp5 ;
  float temperature5 = nil9 / divider ;
  //Serial.print("Temperature5 = "); Serial.println(temperature5);
  temp5 = temperature5;
  delay(500);

  Read_Holding_Reg_humi5 = Modbus_Master.readInputRegister(5, 2) ;//(5 = Slave ID:5 , 0 = Address : 0 (D0) what is the value)
  float nil10 = Read_Holding_Reg_humi5;
  float humidity5 = nil10 / divider;
  //Serial.print("Humidity5 = "); Serial.println(humidity5);
  humi5 = humidity5;
  delay(500);
//===========================================================================================================================

  //Pembacaan sensor SHT20 slave 6
  Read_Holding_Reg_temp6 = Modbus_Master.readInputRegister(6, 1) ;//(6 = Slave ID:6 , 0 = Address : 0 (D0) what is the value)
  float nil11 = Read_Holding_Reg_temp6 ;
  float temperature6 = nil11 / divider ;
  //Serial.print("Temperature6 = "); Serial.println(temperature6);
  temp6 = temperature6;
  delay(500);

  Read_Holding_Reg_humi6 = Modbus_Master.readInputRegister(6, 2) ;//(6 = Slave ID:6 , 0 = Address : 0 (D0) what is the value)
  float nil12 = Read_Holding_Reg_humi6;
  float humidity6 = nil12 / divider;
  //Serial.print("Humidity6= "); Serial.println(humidity6);
  humi6=humidity6;
  delay(500);
}

// Task to read BH1750
void TaskReadBH1750(void *pvParameters) {
    for (;;) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) { 
        int lux = lightMeter.readLightLevel() * 5;
        // Serial.print("Light: ");
        // Serial.print(lux);
        // Serial.println(" lx");
        vTaskDelay(pdMS_TO_TICKS(2000));
        lightIntensity = lux;
        xSemaphoreGive(xMutex);
      }
    }
}

// Task to read SCD30
void TaskReadSCD30(void *pvParameters) {
    for (;;) {
        float co2Concentration = 0.0;
        float temperaturescd = 0.0;
        float humidityscd = 0.0;

        error = sensor.blockingReadMeasurementData(co2Concentration, temperaturescd, humidityscd);
       // if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        if (error != NO_ERROR) {
            Serial.print("Error trying to execute blockingReadMeasurementData(): ");
            errorToString(error, errorMessage, sizeof errorMessage);
            Serial.println(errorMessage);
        } else {
          //   Serial.print("CO2 Concentration: ");
          //   Serial.print(co2Concentration);
          //   Serial.print("\t");
          //   Serial.print("Temperature: ");
          //   Serial.print(temperaturescd);
          //   Serial.print("\t");
          //   Serial.print("Humidity: ");
          //   Serial.print(humidityscd);
          //   Serial.println();
          scd30Temperature = temperaturescd;
          scd30Humidity = humidityscd;
          co2 = co2Concentration;
        }
        //xSemaphoreGive(xMutex);
       // }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

//Task to read DHT22
void TaskReadDHT22(void *pvParameters) {
    for (;;) {
        float temperaturedht = dht.readTemperature();
        float humiditydht = dht.readHumidity();
      
        if (isnan(temperaturedht) || isnan(humiditydht)) {
            Serial.println("Failed to read from DHT22 sensor!");
        } else {
           //if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            // Serial.print("DHT22 -> Temp: ");
            // Serial.print(temperaturedht);
            // Serial.print(" C, Hum: ");
            // Serial.print(humiditydht);
            // Serial.println(" %");
            panel_temp = temperaturedht;
            //xSemaphoreGive(xMutex);
           //}
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_S1, INPUT_PULLUP);

      // Initialize Modbus
    Serial2.begin(9600, SERIAL_8N1, RX, TX);
    Serial.println(F("Modbus communication started"));

    // Initialize WiFi and MQTT
    setupWiFi();
    mqttClient.begin(mqtt_server, mqtt_port, net);

    // Initialize I2C
    Wire.begin(16, 17); // SDA: GPIO16, SCL: GPIO17 for ESP32

    // Initialize LCD
    lcd.init(16,17);
    lcd.backlight();
    //lcd.clear();

    // Initialize BH1750 light sensor
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    Serial.println(F("BH1750 Test begin"));

    // Initialize SCD30 sensor
    sensor.begin(Wire, SCD30_I2C_ADDR_61);

    // Reset and setup SCD30 sensor
    sensor.stopPeriodicMeasurement();
    sensor.softReset();
    delay(2000);

   // Pastikan firmware version SCD30 terbaca
    uint8_t major = 0, minor = 0;
    error = sensor.readFirmwareVersion(major, minor);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute readFirmwareVersion(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware version major: ");
        Serial.print(major);
        Serial.print("\t");
        Serial.print("minor: ");
        Serial.print(minor);
        Serial.println();
    }
    
    // Start periodic measurement on SCD30
    error = sensor.startPeriodicMeasurement(0);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }

    // Initialize DHT22
    dht.begin();

     // Buat mutex
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
      Serial.println("Failed to create mutex!");
      while (1); 
    }

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(blowertask, "Blower Task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskReadBH1750, "Read BH1750", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskReadSCD30, "Read SCD30", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskReadDHT22, "Read DHT22", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(connectMQTT, "MQTT Task", 4098, NULL, 2, NULL, 1); 
    //xTaskCreatePinnedToCore(publishData, "Publish data", 4096, NULL,1, NULL, 1);
    xTaskCreatePinnedToCore(displayDataOnLCD, "display LCD", 4096, NULL, 2,NULL,1);
}

void loop() {
    // Ensure WiFi and MQTT connection
    if (WiFi.status() != WL_CONNECTED) {
        setupWiFi();  // Coba koneksi WiFi lagi jika terputus
    }

    if (!mqttClient.connected()) {
        connectMQTT();  // Coba koneksi MQTT lagi jika terputus
    }

    mqttClient.loop();  // Mengupdate status MQTT

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        readsensorsht20();
    }

    // Logika restart otomatis setelah 12 jam
    if (currentMillis - previousRestartMillis >= restartInterval) {
        previousRestartMillis = currentMillis;
        Serial.println("12 jam telah berlalu, merestart ESP32...");
        ESP.restart();  // Merestart ESP32
    }

    // Calculate averages
    float avgTemp = 0.0;
    float avgHum = 0.0;

  // Menghitung rata-rata suhu dari semua sensor Modbus
  avgTemp = (temp1 + temp2 + temp3 + temp4 + temp5 + temp6 + scd30Temperature) / 7.0;

  // Menghitung rata-rata kelembaban dari semua sensor Modbus
  avgHum = (humi1 + humi2 + humi3 + humi4 + humi5 + humi6 + scd30Humidity) / 7.0;

  avgtempall = avgTemp ;
  avgHumall = avgHum;

    // Publish data to MQTT
    publishData(); 

    if(mode == "Automatic"){
    if(avgtempall >= cooling_active){
      HT74HC595 -> set(RELAY_BLOWER,HIGH, true);
      blowerState = true;
      sendPumpStatus("Cooling_System", blowerState, "status");
    }else if(avgtempall <= cooling_inactive){
      blowerState = false;
      HT74HC595 -> set(RELAY_BLOWER,LOW, true);
      sendPumpStatus("Cooling_System", blowerState, "status");
    }
  }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Prevent loop blocking
}