#include "HardwareSerial.h"            // Library untuk Serial Hardware
#include <ModbusMaster.h>              // Library untuk komunikasi Modbus RTU
//#include "DFRobot_RTU.h"             // Library untuk komunikasi Modbus RTU
#include <SensirionI2cScd30.h>         // Library untuk sensor SCD30
#include <BH1750.h>                    // Library untuk sensor cahaya BH1750
#include <Wire.h>                      // Library untuk komunikasi I2C
#include <WiFi.h>                      // Library untuk WiFi
#include <MQTT.h>                      // Library untuk MQTT
#include <ArduinoJson.h>               // Library untuk JSON (untuk data komunikasi)
#include <LiquidCrystal_I2C.h>         // Library untuk LCD I2C
#include "ShiftRegister74HC595_NonTemplate.h"   // Library untuk shift register 74HC595 (pengendalian pin digital tambahan)
#include "pin_config.h"                // File konfigurasi untuk pin-pin yang mcu dan relay yang terhubung ke ic HT74HC595
#include <iostream>                    // Library untuk fungsi I/O standar C++
#include <memory>   
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>                   // Library untuk manajemen memori dinamis (smart pointers)


#define DHTPIN 1    // Ganti dengan pin yang Anda gunakan
#define DHTTYPE DHT22  // Jenis sensor

// Inisialisasi DHT
DHT dht(DHTPIN, DHTTYPE);

#define RXD 36                         // Pin untuk RXD Modbus
#define TXD 37                         // Pin untuk TXD Modbus
// #define TRIGPIN 17                     // Pin trigger untuk sensor Ultrasonik
// #define ECHOPIN 16                     // Pin echo untuk sensor Ultrasonik
#define BUTTON_PIN 2                  // Pin untuk push button
#define AVG_BUFFER_SIZE 10             // Ukuran buffer untuk perhitungan rata-rata
#define RELAY_CHANNEL 0                // Channel relay pada shift register (CH1)

// Detail Koneksi WiFi dan MQTT
const char* ssid = "R. MEETING/KELAS UNINUS (G102)";        // Nama jaringan WiFi
const char* password = "uninusunggul2025"; // Password jaringan WiFi
const char* mqtt_server = "your_MQTT_SERVER"; // Server MQTT

// Inisialisasi objek sensor
ModbusMaster node;
SensirionI2cScd30 sensor;              // Objek untuk sensor SCD30
BH1750 lightMeter(0x23);               // Objek untuk sensor cahaya BH1750, alamat I2C 0x23
LiquidCrystal_I2C lcd(0x27, 20, 4);    // LCD 20x4 I2C, alamat 0x27

// Inisialisasi objek WiFi dan MQTT
WiFiClient net;
MQTTClient mqttClient;

// Buffer untuk menyimpan data suhu dan kelembapan untuk perhitungan rata-rata
float tempBuffer[AVG_BUFFER_SIZE];
float humBuffer[AVG_BUFFER_SIZE];
int bufferIndex = 0;                   // Index untuk buffer, akan terus berputar

// Variabel untuk batas suhu, mode, dan status
float temperatureThresholdOn = 30.0;   // Batas suhu untuk menyalakan relay
float temperatureThresholdOff = 25.0;  // Batas suhu untuk mematikan relay
bool isManualMode = false;             // Status mode: manual (true) atau otomatis (false)
bool manualRelayState = false;         // Status relay untuk mode manual
bool isThresholdConfigured = false;    // Menandakan apakah threshold telah diatur dari MQTT
bool debugMode = true;                 // Jika true, menampilkan data di Serial Monitor
bool isWiFiConnected = false;          // Status koneksi WiFi

// Variabel untuk data sensor tambahan
float co2 = 0.0;                       // Variabel untuk menyimpan data CO2
//float distance = 0.0;                  // Variabel untuk menyimpan data jarak
int lux = 0.0;                         // Variabel untuk menyimpan data intensitas cahaya
float temperature = 0.0;               // Variabel untuk menyimpan data temperature sht20
float humidity = 0.0;                  // Variavel untuk menyimpan data humidity sht20 
float tempscd30 = 0.0;                 // Variabel untuk menyimpan data temp scd30
float humscd30 = 0.0;                  // Variabel untuk menyimpan data hum scd30
float tempdht22 = 0.0; // Variabel untuk menyimpan data temp dht22
float humdht22 = 0.0; // Variabel untuk menyimpan data dht22

// Inisialisasi shift register untuk mengontrol relay
std::shared_ptr<ShiftRegister74HC595_NonTemplate> HT74HC595 =
    std::make_shared<ShiftRegister74HC595_NonTemplate>(8, HT74HC595_DATA,
                                                        HT74HC595_CLOCK, HT74HC595_LATCH);

// Task handles untuk setiap sensor
TaskHandle_t scd30Task, bh1750Task, dht22task;

// Fungsi untuk mempublikasikan data JSON ke topik MQTT tertentu
void publishJsonData(const char* topic, JsonObject json) {
  if (isWiFiConnected) {
    String payload;
    serializeJson(json, payload);      // Mengubah objek JSON menjadi string
    mqttClient.publish(topic, payload); // Mengirim data ke topik MQTT
    if (debugMode) {
      Serial.print("Published to MQTT - Topic: ");
      Serial.print(topic);
      Serial.print(", Payload: ");
      Serial.println(payload);
    }
  }
}

// Fungsi untuk menghitung rata-rata nilai dalam buffer
float calculateAverage(float *buffer, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += buffer[i];                  // Menjumlahkan semua nilai dalam buffer
  }
  return sum / size;                   // Mengembalikan nilai rata-rata
}

// Fungsi untuk mempublikasikan semua data sensor dalam satu JSON ke topik MQTT "SmartGreenHouse/SensorData"
void publishAllSensorData() {
  StaticJsonDocument<256> jsonDoc;

  // Data suhu dan kelembapan rata-rata
  float avgTemperature = calculateAverage(tempBuffer, AVG_BUFFER_SIZE);
  float avgHumidity = calculateAverage(humBuffer, AVG_BUFFER_SIZE);
  jsonDoc["avg_temp"] = avgTemperature;
  jsonDoc["avg_humi"] = avgHumidity;

  jsonDoc["temperature"] = temperature;
  jsonDoc["humidity"] = humidity;
  
  jsonDoc["temperature7"] = tempscd30; 
  jsonDoc["humidity7"]= humscd30;

  // Data tambahan dari sensor CO2 (SCD30)
  jsonDoc["co2"] = co2;
  
  // Data jarak (Ultrasonik)
  //jsonDoc["distance"] = distance;

  // Data intensitas cahaya (BH1750)
  jsonDoc["lux"] = lux;

  jsonDoc["room_temp"] = tempdht22;
  jsonDoc["room_humi"] = humdht22;

  // Ubah JSON menjadi string dan kirim ke MQTT dengan topik SmartGreenHouse/SensorData
  String payload;
  serializeJson(jsonDoc, payload);  
  mqttClient.publish("SmartGreenHouse/SensorData", payload.c_str());

  // Debug output untuk memastikan data yang dipublikasikan
  if (debugMode) {
    Serial.print("Published to SmartGreenHouse/SensorData: ");
    Serial.println(payload);
  }
}


// Fungsi interrupt untuk mendeteksi tombol yang ditekan pada mode manual
void IRAM_ATTR handleButtonPress() {
  if (isManualMode) {                  // Tombol hanya aktif jika dalam mode manual
    manualRelayState = !manualRelayState; // Toggle status relay (ON/OFF)
    HT74HC595->set(RELAY_CHANNEL, manualRelayState ? HIGH : LOW, true); // Ubah status relay
    if (debugMode) {
      Serial.print("Manual Control - Relay State: ");
      Serial.println(manualRelayState ? "ON" : "OFF");
    }
  }
}

// Fungsi untuk mengontrol relay dalam mode manual
void manualControlRelay() {
  if (isManualMode) {
    // Mengatur pin relay sesuai dengan status relay di mode manual
    HT74HC595->set(RELAY_CHANNEL, manualRelayState ? HIGH : LOW, true); // Kontrol relay manual
    if (debugMode) {
      Serial.print("Manual Control - Relay State: ");
      Serial.println(manualRelayState ? "ON" : "OFF");
    }
  }
}

// Fungsi untuk mengontrol relay dalam mode otomatis
void autoControlRelay(float avgTemperature) {
  if (!isManualMode && isThresholdConfigured) { // Kontrol hanya bekerja jika mode otomatis aktif
    if (avgTemperature >= temperatureThresholdOn && !manualRelayState) {
      HT74HC595->set(RELAY_CHANNEL, HIGH, true); // Relay ON
      manualRelayState = true;
      if (debugMode) Serial.println("Auto Control - Relay turned ON due to temperature threshold.");
    } else if (avgTemperature <= temperatureThresholdOff && manualRelayState) {
      HT74HC595->set(RELAY_CHANNEL, LOW, true);  // Relay OFF
      manualRelayState = false;
      if (debugMode) Serial.println("Auto Control - Relay turned OFF due to temperature threshold.");
    }
  }
}

// Fungsi untuk mengirim status mode dan relay ke MQTT
void publishModeStatus() {
  if (isWiFiConnected) {
    StaticJsonDocument<64> jsonDoc;  // Menggunakan StaticJsonDocument lagi dengan ukuran tetap
    jsonDoc["mode"] = isManualMode ? "manual" : "auto";
    jsonDoc["manualRelayState"] = manualRelayState;
    
    String output;
    serializeJson(jsonDoc, output);  // Serialisasi JSON menjadi string sebelum dikirim
    mqttClient.publish("sensors/mode_status", output.c_str());
    
    if (debugMode) {
      Serial.print("Published Mode Status - Mode: ");
      Serial.print(isManualMode ? "Manual" : "Auto");
      Serial.print(", Relay State: ");
      Serial.println(manualRelayState ? "ON" : "OFF");
    }
  }
}

// Fungsi untuk mengatur batas suhu dan mode melalui MQTT
void messageReceived(String &topic, String &payload) {
  if (isWiFiConnected) {               // Proses hanya jika terhubung ke WiFi
    StaticJsonDocument<128> jsonDoc;
    deserializeJson(jsonDoc, payload); // Parse JSON payload
  
    // Jika ada pengaturan batas suhu ON dari MQTT, set nilai dan aktifkan threshold
    if (jsonDoc["temperatureThresholdOn"].is<int>()) {  // Ganti containsKey dengan is<int>
      temperatureThresholdOn = jsonDoc["temperatureThresholdOn"];
      isThresholdConfigured = true;
      if (debugMode) {
        Serial.print("Received MQTT - Temperature Threshold ON: ");
        Serial.println(temperatureThresholdOn);
      }
    }

    // Jika ada pengaturan batas suhu OFF dari MQTT, set nilai dan aktifkan threshold
    if (jsonDoc["temperatureThresholdOff"].is<int>()) {  // Ganti containsKey dengan is<int>
      temperatureThresholdOff = jsonDoc["temperatureThresholdOff"];
      isThresholdConfigured = true;
      if (debugMode) {
        Serial.print("Received MQTT - Temperature Threshold OFF: ");
        Serial.println(temperatureThresholdOff);
      }
    }
  
    // Jika ada pengaturan mode dari MQTT, update mode manual/auto
    if (jsonDoc["mode"].is<const char*>()) {  // Ganti containsKey dengan is<const char*>
      String mode = jsonDoc["mode"].as<String>();
      isManualMode = (mode == "manual");
      publishModeStatus();             // Publikasikan status mode ke MQTT
      if (debugMode) {
        Serial.print("Received MQTT - Mode set to: ");
        Serial.println(isManualMode ? "Manual" : "Auto");
      }
    }

    // Jika dalam mode manual, update status relay dari nilai di MQTT
    if (isManualMode && jsonDoc["manualRelayState"].is<bool>()) {  // Ganti containsKey dengan is<bool>
      manualRelayState = jsonDoc["manualRelayState"];
      manualControlRelay();            // Panggil fungsi kontrol relay manual
      if (debugMode) {
        Serial.print("Received MQTT - Manual Relay State set to: ");
        Serial.println(manualRelayState ? "ON" : "OFF");
      }
    }
  }
}

// Fungsi untuk menampilkan data di LCD dan Serial Monitor
void updateDisplay(float avgTemperature, float avgHumidity, float co2, float lux, bool isAutoMode) {
  lcd.clear();                         // Membersihkan layar LCD sebelum update

  // Menampilkan data suhu rata-rata pada baris pertama
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(avgTemperature);
  lcd.print(" C");

  // Menampilkan data kelembapan rata-rata pada baris kedua
  lcd.setCursor(0, 1);
  lcd.print("Hum:  ");
  lcd.print(avgHumidity);
  lcd.print(" %");

  // Menampilkan data CO2 pada baris ketiga
  lcd.setCursor(0, 2);
  lcd.print("CO2: ");
  lcd.print(co2);
  lcd.print(" ppm");

  // // Menampilkan data jarak (ultrasonik) pada baris keempat
  // lcd.setCursor(0, 3);
  // lcd.print("Dist: ");
  // lcd.print(distance);
  // lcd.print(" cm");

  // Menampilkan data intensitas cahaya (lux) di pojok kanan bawah
  lcd.setCursor(12, 2);
  lcd.print("Lux: ");
  lcd.print(lux);

  // Menampilkan status mode di pojok kanan bawah (Auto/Manual)
  lcd.setCursor(12, 3);
  lcd.print(isAutoMode ? "Auto" : "Manual");

  // Jika debugMode aktif, tampilkan data di Serial Monitor
  if (debugMode) {
    Serial.println("===== LCD Display =====");
    Serial.print("Average Temperature: ");
    Serial.print(avgTemperature);
    Serial.println(" C");
    
    Serial.print("Average Humidity: ");
    Serial.print(avgHumidity);
    Serial.println(" %");

    Serial.print("CO2 Level: ");
    Serial.print(co2);
    Serial.println(" ppm");

    // Serial.print("Distance: ");
    // Serial.print(distance);
    // Serial.println(" cm");

    Serial.print("Light Intensity: ");
    Serial.print(lux);
    Serial.println(" lux");

    Serial.print("Mode: ");
    Serial.println(isAutoMode ? "Auto" : "Manual");
    Serial.println("=========================");
  }
}

// Fungsi untuk membaca data dari sensor Modbus (6 slave ID)
// void readSensor(void *pvParameters) {
//   int slaveID = *((int *)pvParameters); // Ambil ID dari parameter yang diteruskan
//   uint16_t regTemp, regHum;

//   while (1) {
//     // Membaca register suhu dan kelembapan dari Modbus
//     regTemp = Modbus_Master.readInputRegister(slaveID, 1);
//     float temperature = regTemp / 10.0;
//     tempBuffer[bufferIndex] = temperature;

//     regHum = Modbus_Master.readInputRegister(slaveID, 2);
//     float humidity = regHum / 10.0;
//     humBuffer[bufferIndex] = humidity;

//     // Update bufferIndex secara melingkar
//     bufferIndex = (bufferIndex + 1) % AVG_BUFFER_SIZE;

//     // Buat objek JSON untuk data Modbus
//     StaticJsonDocument<128> jsonDoc;
//     jsonDoc["slaveID"] = slaveID;
//     jsonDoc["temperature"] = temperature;
//     jsonDoc["humidity"] = humidity;

//     // Publish data Modbus ke topik MQTT yang sesuai
//     String topic = "sensors/modbus/" + String(slaveID);
//     publishJsonData(topic.c_str(), jsonDoc.as<JsonObject>());

//     // Output debug untuk pembacaan sensor Modbus
//     if (debugMode) {
//       Serial.print("Modbus Sensor - Slave ID: ");
//       Serial.print(slaveID);
//       Serial.print(", Temperature: ");
//       Serial.print(temperature);
//       Serial.print(" C, Humidity: ");
//       Serial.print(humidity);
//       Serial.println(" %");
//     }

//     vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay untuk loop berikutnya
//   }
// }

void readSensorData(int slaveID) {
    // Set slave ID for each reading
    node.begin(slaveID, Serial2);

    // Read 2 registers from address 1 (0x0001) on the slave
    uint8_t result = node.readInputRegisters(1, 2); // Read 2 registers starting from address 1
    uint16_t data[2];

    if (result == node.ku8MBSuccess) {
        // Store register values
        data[0] = node.getResponseBuffer(0); // First register
        data[1] = node.getResponseBuffer(1); // Second register
        float temperaturesht20 = data[0] / 10.0;
        float humiditysht20 = data[1] / 10.0;
        tempBuffer[bufferIndex] = temperaturesht20;
        humBuffer[bufferIndex] = humiditysht20;
        temperature = temperaturesht20;
        humidity = humiditysht20;
        if (debugMode) {
          // Print the reading results for each slave
          Serial.print("Slave ID ");
          Serial.print(slaveID);
          Serial.print(" - Temperature: ");
          Serial.print(temperaturesht20);
          Serial.print(" °C, Humidity: ");
          Serial.print(humiditysht20);
          Serial.println(" %");
        }
    } else {
        if (debugMode) {
          // Print error if failed to read
          Serial.print("Error reading input registers from Slave ID ");
          Serial.print(slaveID);
          Serial.print(": ");
          Serial.println(result);
        }
    }
}

// Fungsi untuk membaca data dari sensor SCD30
void readSCD30(void *pvParameters) {
  Wire.begin();
  sensor.begin(Wire, SCD30_I2C_ADDR_61);
  sensor.stopPeriodicMeasurement();
  sensor.softReset();
  delay(2000);

  sensor.startPeriodicMeasurement(0);
  float co2Value, temp, hum;

  while (1) {
    sensor.blockingReadMeasurementData(co2Value, temp, hum);
    tempBuffer[bufferIndex] = temp;
    humBuffer[bufferIndex] = hum;
    bufferIndex = (bufferIndex + 1) % AVG_BUFFER_SIZE;
    co2 = co2Value;
    tempscd30 = temp;
    humscd30 = hum;

    // Debug output untuk sensor SCD30
    if (debugMode) {
      Serial.print("SCD30 - CO2: ");
      Serial.print(co2Value);
      Serial.print(" ppm, Temp: ");
      Serial.print(temp);
      Serial.print(" C, Hum: ");
      Serial.println(hum);
    }

    // StaticJsonDocument<128> jsonDoc;
    // jsonDoc["co2"] = co2Value;
    // jsonDoc["temperature"] = temp;
    // jsonDoc["humidity"] = hum;

    //publishJsonData("sensors/scd30", jsonDoc.as<JsonObject>());

    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }
}

// Fungsi untuk membaca data dari sensor BH1750
void readBH1750(void *pvParameters) {
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    if (debugMode) Serial.println(F("BH1750 Initialized Successfully"));
  } else {
    Serial.println(F("Error initializing BH1750"));
    vTaskDelete(NULL);
  }

  while (1) {
    if (lightMeter.measurementReady()) {
      int luxValue = lightMeter.readLightLevel();
      lux = luxValue;

      // Debug output untuk sensor BH1750
      if (debugMode) {
        Serial.print("BH1750 - Light Intensity: ");
        Serial.print(luxValue);
        Serial.println(" lux");
      }

      // StaticJsonDocument<64> jsonDoc;
      // jsonDoc["lux"] = luxValue;

      // publishJsonData("sensors/bh1750", jsonDoc.as<JsonObject>());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// // Fungsi untuk membaca data dari sensor ultrasonik
// void readUltrasonic(void *pvParameters) {
//   pinMode(TRIGPIN, OUTPUT);
//   pinMode(ECHOPIN, INPUT);

//   while (1) {
//     digitalWrite(TRIGPIN, LOW);
//     delayMicroseconds(2);
//     digitalWrite(TRIGPIN, HIGH);
//     delayMicroseconds(20);
//     digitalWrite(TRIGPIN, LOW);

//     float duration = pulseIn(ECHOPIN, HIGH);
//     float distanceValue = (duration / 2) * 0.343 / 10;

//     distance = distanceValue;

//     // Debug output untuk sensor Ultrasonik
//     if (debugMode) {
//       Serial.print("Ultrasonic - Distance: ");
//       Serial.print(distanceValue);
//       Serial.println(" cm");
//     }

//     StaticJsonDocument<64> jsonDoc;
//     jsonDoc["distance"] = distanceValue;

//     publishJsonData("sensors/ultrasonic", jsonDoc.as<JsonObject>());

//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//   }
// }

// Fungsi untuk membaca data dari DHT20
void readDHT22(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    // Baca data suhu
    float temperature1 = dht.readTemperature();
    // Baca data kelembapan
    float humidity1 = dht.readHumidity();
    tempdht22 = temperature1;
    humdht22 = humidity1;

    // // Periksa apakah pembacaan valid
    // if (isnan(temperature1) || isnan(humidity1)) {
    //   Serial.println("Gagal membaca data dari DHT20!");
    // } else {
    //   // Cetak hasil ke Serial Monitor
    //   Serial.print("Temperature: ");
    //   Serial.print(temperature1);
    //   Serial.println(" °C");

    //   Serial.print("Humidity: ");
    //   Serial.print(humidity1);
    //   Serial.println(" %");
    // }

    // Delay sebelum pembacaan berikutnya
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// Fungsi untuk koneksi WiFi
void connectToWiFi() {
  if (debugMode) Serial.println("Attempting to connect to WiFi...");

  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    if (debugMode) Serial.print(".");
  }

  isWiFiConnected = (WiFi.status() == WL_CONNECTED);

  if (isWiFiConnected) {
    if (debugMode) Serial.println("\nWiFi connected successfully.");
  } else {
    if (debugMode) Serial.println("\nWiFi connection failed. Switching to manual mode.");
    isManualMode = true;
  }
}

// Fungsi untuk koneksi ke broker MQTT
void connectToMQTT() {
  if (isWiFiConnected) {
    mqttClient.begin(mqtt_server,1883, net);  // Atur port untuk koneksi MQTT
    mqttClient.onMessage(messageReceived);
    
    if (debugMode) Serial.println("Attempting to connect to MQTT broker...");
    while (!mqttClient.connect("SRGH_1", "UNINUSUNGGUL", "UNINUSUNGGUL123")) {
      delay(500);
      if (debugMode) Serial.print(".");
    }

    if (debugMode) Serial.println("\nConnected to MQTT broker.");
    mqttClient.subscribe("sensors/threshold");
  }
}

static char errorMessage[128];
static int16_t error;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);

  lcd.init();                         // Inisialisasi LCD
  lcd.backlight();    
  
   // Inisialisasi sensor DHT22
  dht.begin();                 // Aktifkan backlight LCD

   // Initialize the I2C bus
  Wire.begin(16, 17);  // SDA pada GPIO 16, SCL pada GPIO 17 untuk ESP32
  // Initialize BH1750 light sensor
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(F("BH1750 Test begin"));

   // Initialize SCD30 sensor
    sensor.begin(Wire, SCD30_I2C_ADDR_61);

    // Reset and setup SCD30 sensor
    sensor.stopPeriodicMeasurement();
    sensor.softReset();
    delay(2000);

    uint8_t major = 0;
    uint8_t minor = 0;
    error = sensor.readFirmwareVersion(major, minor);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute readFirmwareVersion(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
    Serial.print("Firmware version major: ");
    Serial.print(major);
    Serial.print("\t");
    Serial.print("minor: ");
    Serial.print(minor);
    Serial.println();
    
    // Start periodic measurement on SCD30
    error = sensor.startPeriodicMeasurement(0);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
  
   // Allocate the JSON document
  JsonDocument doc;

   // Konfigurasi shift register 74HC595 untuk relay
  Serial.println("Setting up 74HC595 and relay control...");
  pinMode(HT74HC595_OUT_EN, OUTPUT);
  digitalWrite(HT74HC595_OUT_EN, HIGH); // Mulai dalam keadaan mati
  
  Serial.println("Set all relay outputs to low level");
  HT74HC595->setAllLow();

  Serial.println("Set GPIO4 to low level to enable relay output");
  digitalWrite(HT74HC595_OUT_EN, LOW);  // Aktifkan output shift register
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);   // Set pin tombol sebagai input dengan pull-up
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  connectToWiFi();                     // Hubungkan ke WiFi
  connectToMQTT();                     // Hubungkan ke broker MQTT jika WiFi terhubung

  //static int slaveID1 = 1, slaveID2 = 2, slaveID3 = 3, slaveID4 = 4, slaveID5 = 5, slaveID6 = 6;

  // Buat task untuk setiap sensor Modbus, SCD30, BH1750, dan Ultrasonik
  // xTaskCreate(readSensor, "SensorTask1", 2048, &slaveID1, 1, &sensorTask1);
  // xTaskCreate(readSensor, "SensorTask2", 2048, &slaveID2, 1, &sensorTask2);
  // xTaskCreate(readSensor, "SensorTask3", 2048, &slaveID3, 1, &sensorTask3);
  // xTaskCreate(readSensor, "SensorTask4", 2048, &slaveID4, 1, &sensorTask4);
  // xTaskCreate(readSensor, "SensorTask5", 2048, &slaveID5, 1, &sensorTask5);
  // xTaskCreate(readSensor, "SensorTask6", 2048, &slaveID6, 1, &sensorTask6);

  xTaskCreate(readSCD30, "SCD30Task", 4096, NULL, 1, &scd30Task);
  xTaskCreate(readBH1750, "BH1750Task", 2048, NULL, 1, &bh1750Task);
 // xTaskCreate(readUltrasonic, "UltrasonicTask", 2048, NULL, 1, &ultrasonicTask);
  xTaskCreate(readDHT22, "TaskReadDHT22", 2048,NULL, 1, &dht22task);
}

void loop() {
  if (isWiFiConnected) {
    mqttClient.loop();                 // Update MQTT jika terhubung ke WiFi
  }

  // Reading data from each Modbus slave with ID 1 to 6
    for (int slaveID = 1; slaveID <= 6; slaveID++) {
        readSensorData(slaveID);
        delay(500); // Small delay between each slave reading for stability
    }
    delay(1000); // Delay 1 second before the next read cycle

  float avgTemperature = calculateAverage(tempBuffer, AVG_BUFFER_SIZE); // Hitung suhu rata-rata
  float avgHumidity = calculateAverage(humBuffer, AVG_BUFFER_SIZE);     // Hitung kelembapan rata-rata

  if (!isManualMode) {
    autoControlRelay(avgTemperature);  // Kontrol relay otomatis jika mode auto
  }

  updateDisplay(avgTemperature, avgHumidity, co2, lux, isManualMode); // Update tampilan LCD

  // StaticJsonDocument<128> jsonDoc;
  // jsonDoc["average_temperature"] = avgTemperature;
  // jsonDoc["average_humidity"] = avgHumidity;

  // publishJsonData("sensors/average", jsonDoc.as<JsonObject>());

  delay(5000);                         // Delay 5 detik untuk loop utama
}
