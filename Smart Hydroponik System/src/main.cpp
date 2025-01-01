// Code for generating the Smart Hydroponic System (UNINUS)

// === Included Libraries ===
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <MQTT.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include "ShiftRegister74HC595_NonTemplate.h"
#include <HardwareSerial.h>
#include <esp_now.h>
#include "pin_config.h"

// === WiFi configuration ===
const char * ssid = "SMART GREENHOUSE UNINUS (2Ghz)";
const char * password = "UNINUSLAB530";

// === MQTT configuration ===
const char* mqtt_server = "88.222.214.56";
const int mqtt_port = 1883;
const char* mqtt_user = "SGH_1.0";
const char* mqtt_password = "Teknik_Pertanian_2025";
const char * clientID = "SmartHydroponik_Uninus";
const char * mqttSensorTopic = "SmartHydroponik/SensorData";
const char * mqttControlTopic = "SmartHydroponik/Control_Send";


// === WiFi and MQTT client objects ===
WiFiClient wifiClient;
MQTTClient mqttClient;

// === Button and Relay Definitions ===
// - Buttons are used for manual control (PH UP, PH DOWN, Nutrient AB)
// - Relays are used to control hardware (pumps or actuators)
#define BUTTON_S1 4                  // GPIO pin for PH UP button
#define BUTTON_S2 8                  // GPIO pin for PH DOWN button
#define BUTTON_S3 14                 // GPIO pin for Nutrient AB button
//#define BUTTON_S4 12               // GPIO pin for Spraying button
#define RELAY_PH_UP 3                // GPIO pin for PH UP relay
#define RELAY_PH_DOWN 2              // GPIO pin for PH DOWN relay
#define RELAY_ZAT_A 1                // GPIO pin for Nutrient A relay
#define RELAY_ZAT_B 0                // GPIO pin for Nutrient B relay
//#define RELAY_SPRAYING 4           // GPIO pin for Spraying relay
#define DS18B20PIN 9                 // GPIO pin for DS18B20 temperature sensor
#define DHTPIN 1                     // GPIO pin for the DHT sensor
#define DHTTYPE DHT22                // DHT sensor type (DHT22)
DHT dht(DHTPIN, DHTTYPE);            // DHT sensor instance
#define TdsSensorPin 11              // GPIO pin for the TDS sensor
#define phSensorPin 13               // GPIO pin for the pH sensor
#define BUZZER_PIN 2                 // GPIO pin for the buzzer
#define RXD 44                       // RX pin for the ultrasonic sensor
#define TXD 43                       // TX pin for the ultrasonic sensor
//#define flowratepin  12            // GPIO pin fotr the flow meter

// === Pump and Button States ===
// Variables to track the state of pumps and the last state of buttons.
bool pumpPhUpState = false, pumpPhDownState = false, pumpZatAState = false, pumpZatBState = false, pumpSprayingState = false;
bool lastButtonS1State = HIGH, lastButtonS2State = HIGH, lastButtonS3State = HIGH, lastButtonS4State = HIGH;

// === Sensor and Device Initialization ===
// Initialization of sensors and devices.
DHT dht(DHTPIN, DHTTYPE);            // DHT sensor instance
OneWire oneWire(DS18B20PIN);           // OneWire instance for DS18B20 sensor
DallasTemperature ds18b20(&oneWire);  // DS18B20 temperature sensor instance
LiquidCrystal_I2C lcd(0x27, 20, 4);   // LCD with I2C address 0x27, 20x4 display
#define TANK_HEIGHT_CM 36             // Water tank height in centimeters
HardwareSerial Ultrasonic_Sensor(2);  // Serial instance for ultrasonic sensor (UART2)

// === Data Buffers and Timing ===
// Variables for sensor data processing and timing.
unsigned char data[4] = {};
unsigned long lastPrintTime = 0; // Waktu terakhir menampilkan data
bool buzzerState = false; // Status buzzer

// // === Flow Meter Configuration ===
// volatile long pulseCount = 0;  // Variabel untuk menghitung pulsa dari sensor
// const float flowFactor = 4.8;  // Konstanta flow meter (pulsa per liter)

// === Global Variables ===
// Variables to store sensor readings and control parameters.
float phSensorValue = 0.0, temperatureDS18B20 = 0.0;
float temperatureDHT = 0.0, humidityDHT = 0.0, waterflowdata = 0.0;
int waterlevel = 0, tdsValue = 0/*, flowRate = 0*/;
volatile int receivedFlowRate = 0; // Variabel untuk menyimpan data yang diterima

// Control limits for pH and nutrients
float limitPhMin, limitPhMax;         // pH range limits
int limitNutrisiMin, limitNutrisiMax; // Nutrient level range limits
float start_spray, end_spray;         // Spray timing parameters
float currentPhValue;                 // Current pH value
int currentNutrisiValue;              // Current nutrient value
int tangkiAir;                        // Water tank status

// Operating mode
String mode = "Manual";               // System mode (Manual/Automatic)

// === FreeRTOS Mutex ===
// Mutex for protecting shared resources in FreeRTOS tasks.
SemaphoreHandle_t xMutex;

// === Shift Register Initialization ===
// Creates an instance of ShiftRegister74HC595_NonTemplate to control relays or LEDs.
std::shared_ptr < ShiftRegister74HC595_NonTemplate > HT74HC595 =
  std::make_shared < ShiftRegister74HC595_NonTemplate > (6, HT74HC595_DATA, HT74HC595_CLOCK, HT74HC595_LATCH);

// === WiFi Setup ===
// Connects to the specified WiFi network and prints the IP address if successful.
void setupWiFi() {
  Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime >= 10000) {  // 10-second timeout
            Serial.println("WiFi connection timeout");
            return;  // exit function if timeout
        }
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

// === MQTT Callback Function ===
void mqttCallback(String &topic, String &payload) {
  // Debug: Print incoming topic and payload
  Serial.println("incoming: " + topic + " - " + payload);
  // Debugging
  Serial.println("debug 0");

    // Parsing JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.f_str());
        return;
    }
    // Update setting from "SmartHydroponik/Settings" topic
    if (String(topic) == "SmartHydroponik/Settings") {
    limitPhMin = doc["Limit_ph_min"];
    limitPhMax = doc["Limit_ph_max"];
    limitNutrisiMin = doc["Limit_nutrisi_min"];
    limitNutrisiMax = doc["Limit_nutrisi_max"];
    tangkiAir = doc["tangki_air"];
  }

    if (String(topic) == "SmartHydroponik/Control") {
    String type = doc["type"];
    // Debugging control
    if (type == "control") {
    String pompa = doc["pompa"];
    int status = doc["status"];
    String device = doc["device"];
      if (pompa == "Pompa_PHUP") {
        bool pumpPhUpState = (status == 1);
        HT74HC595->set(RELAY_PH_UP, pumpPhUpState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_PHUP", pumpPhUpState);
      } else if (pompa == "Pompa_PHDOWN") {
        bool pumpPhDownState = (status == 1);
        HT74HC595->set(RELAY_PH_DOWN, pumpPhDownState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_PHDOWN", pumpPhDownState);
      } else if (pompa == "Pompa_Nutrisi") {
        bool pumpZatAState = (status == 1);
        bool pumpZatBState = (status == 1);
        HT74HC595->set(RELAY_ZAT_A, pumpZatAState ? HIGH : LOW, true);
        HT74HC595->set(RELAY_ZAT_B, pumpZatBState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_Nutrisi", pumpZatBState);
      } else if (pompa == "Pompa_Spraying") {
        bool pumpSprayingState = (status == 1);
        //HT74HC595->set(RELAY_SPRAYING, pumpSprayingState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_Spraying", pumpSprayingState);
      }
    }
    if (type == "mode") {
      int dump =  doc["status"];
      mode = dump ? "Automatis" : "Manual";
      Serial.println("Mode set to: " + mode);
    }
  }
}

// === MQTT Connection Function ===
void connectMQTT() {
  while (WiFi.status() != WL_CONNECTED) {
    setupWiFi(); //ensure WiFi connection
  }

  mqttClient.begin(mqtt_server, mqtt_port, wifiClient);
  mqttClient.onMessage(mqttCallback); // Set callback function for incoming messages

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("SmartHydroponik_Uninus", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT Broker!");
      mqttClient.subscribe("SmartAerophonik/Settings");
      mqttClient.subscribe("SmartAerophonik/Control"); // Subscribe to control topic
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
  jsonDoc["pompa"] = pumpName;
  jsonDoc["status"] = state ? 1 : 0;
  jsonDoc["device"] = "Hydroponik";

  char jsonBuffer[128];
  serializeJson(jsonDoc, jsonBuffer);

  if (mqttClient.publish(mqttControlTopic, jsonBuffer)) {
    Serial.printf("Status %s terkirim: %s\n", pumpName, state ? "ON" : "OFF");
  } else {
    Serial.printf("Gagal mengirim status %s!\n", pumpName);
  }
}


void controlPumpTask(void * pvParameters) {
  const int debounceDelay = 50;
  static unsigned long lastDebounceTimeS1 = 0, lastDebounceTimeS2 = 0, lastDebounceTimeS3 = 0, lastDebounceTimeS4 = 0;
  for (;;) {
    unsigned long currentTime = millis();

    bool currentButtonS1State = digitalRead(BUTTON_S1);
    if (currentButtonS1State == LOW && lastButtonS1State == HIGH && (currentTime - lastDebounceTimeS1 > debounceDelay)) {
      pumpPhUpState = !pumpPhUpState;
      HT74HC595 -> set(RELAY_PH_UP, pumpPhUpState ? HIGH : LOW, true);
      sendPumpStatus("Pompa_PHUP", pumpPhUpState, "status");
      lastDebounceTimeS1 = currentTime;
    }
    lastButtonS1State = currentButtonS1State;

    bool currentButtonS2State = digitalRead(BUTTON_S2);
    if (currentButtonS2State == LOW && lastButtonS2State == HIGH && (currentTime - lastDebounceTimeS2 > debounceDelay)) {
      pumpPhDownState = !pumpPhDownState;
      HT74HC595 -> set(RELAY_PH_DOWN, pumpPhDownState ? HIGH : LOW, true);
      sendPumpStatus("Pompa_PHDOWN", pumpPhDownState, "status");
      lastDebounceTimeS2 = currentTime;
    }
    lastButtonS2State = currentButtonS2State;

    // Button 3 - ZAT AB
    bool currentButtonS3State = digitalRead(BUTTON_S3);
    if (currentButtonS3State == LOW && lastButtonS3State == HIGH && (currentTime - lastDebounceTimeS3 > debounceDelay)) {
      pumpZatAState = !pumpZatAState;
      pumpZatBState = !pumpZatBState;
      HT74HC595 -> set(RELAY_ZAT_A, pumpZatAState ? HIGH : LOW, true);
      HT74HC595 -> set(RELAY_ZAT_B, pumpZatBState ? HIGH : LOW, true);
      sendPumpStatus("Pompa_Nutrisi", pumpZatBState, "status");
      lastDebounceTimeS3 = currentTime;
    }
    lastButtonS3State = currentButtonS3State;

    // Button 4 - Spraying
    // bool currentButtonS4State = digitalRead(BUTTON_S4);
    // if (currentButtonS4State == LOW && lastButtonS4State == HIGH && (currentTime - lastDebounceTimeS4 > debounceDelay)) {
    //   pumpSprayingState = !pumpSprayingState;
    //   HT74HC595 -> set(RELAY_SPRAYING, pumpSprayingState ? HIGH : LOW, true);
    //   sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
    //   lastDebounceTimeS4 = currentTime;
    // }
    // lastButtonS4State = currentButtonS4State;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Fungsi Auto Pompa pH (Kontrol pH Up/Down secara otomatis)
void auto_pH_pump() {
  if (phSensorValue < limitPhMin) {
    // Jika pH terlalu rendah, nyalakan Pompa PH Up
    if (!pumpPhUpState) {
      pumpPhUpState = true;
      HT74HC595->set(RELAY_PH_UP, HIGH, true);
      sendPumpStatus("Pompa_PHUP", pumpPhUpState, "status");
      Serial.println("Pompa PH UP dihidupkan (pH terlalu rendah)");
    }
  } else if (phSensorValue > limitPhMax) {
    // Jika pH terlalu tinggi, nyalakan Pompa PH Down
    if (!pumpPhDownState) {
      pumpPhDownState = true;
      HT74HC595->set(RELAY_PH_DOWN, HIGH, true);
      sendPumpStatus("Pompa_PHDOWN", pumpPhDownState, "status");
      Serial.println("Pompa PH DOWN dihidupkan (pH terlalu tinggi)");
    }
  } else {
    // Jika pH dalam batas normal, matikan pompa
    if (pumpPhUpState || pumpPhDownState) {
      pumpPhUpState = false;
      pumpPhDownState = false;
      HT74HC595->set(RELAY_PH_UP, LOW, true);
      HT74HC595->set(RELAY_PH_DOWN, LOW, true);
      sendPumpStatus("Pompa_PHUP", pumpPhUpState, "status");
      sendPumpStatus("Pompa_PHDOWN", pumpPhDownState, "status");
      Serial.println("Pompa PH dimatikan (pH dalam batas normal)");
    }
  }
}

// Fungsi Auto Nutrisi (Kontrol Pompa Nutrisi secara otomatis)
void auto_Nutrient_pump() {
  if (tdsValue < limitNutrisiMin) {
    // Jika nutrisi terlalu rendah, nyalakan Pompa Nutrisi
    if (!pumpZatAState && !pumpZatBState) {
      pumpZatAState = true;
      pumpZatBState = true;
      HT74HC595->set(RELAY_ZAT_A, HIGH, true);
      HT74HC595->set(RELAY_ZAT_B, HIGH, true);
      sendPumpStatus("Pompa_Nutrisi", pumpZatBState, "status");
      Serial.println("Pompa Nutrisi dihidupkan (Nutrisi terlalu rendah)");
    }
  } else if (tdsValue > limitNutrisiMax) {
    // Jika nutrisi terlalu tinggi, matikan Pompa Nutrisi
    if (pumpZatAState && pumpZatBState) {
      pumpZatAState = false;
      pumpZatBState = false;
      HT74HC595->set(RELAY_ZAT_A, LOW, true);
      HT74HC595->set(RELAY_ZAT_B, LOW, true);
      sendPumpStatus("Pompa_Nutrisi", pumpZatBState, "status");
      Serial.println("Pompa Nutrisi dimatikan (Nutrisi cukup tinggi)");
    }
  }
}

// Fungtion to play melody
void playMelody() {
  playTone(262, 400); // C4
  playTone(262, 400); // C4
  playTone(392, 400); // G4
  playTone(392, 400); // G4
  playTone(440, 400); // A4
  playTone(440, 400); // A4
  playTone(392, 800); // G4

  if (!buzzerState) return;

  playTone(349, 400); // F4
  playTone(349, 400); // F4
  playTone(330, 400); // E4
  playTone(330, 400); // E4
  playTone(294, 400); // D4
  playTone(294, 400); // D4
  playTone(262, 800); // C4
}

// Helper function to play a tone with duration
void playTone(int frequency, int duration) {
  unsigned long startTime = millis();
  tone(BUZZER_PIN, frequency);
  while (millis() - startTime < duration) {
    if (!buzzerState) {
      noTone(BUZZER_PIN); // Matikan jika kondisi tidak kritis
      return;
    }
  }
  noTone(BUZZER_PIN);
}

// Task: Read Water Level Sensor (Ultrasonic)
void waterlevelTask(void *pvParameters) {
  int distance;
  while (true) {
      if ( Ultrasonic_Sensor.available() >= 4) {
          if ( Ultrasonic_Sensor.read() == 0xFF) {
              data[0] = 0xFF;
              for (int i = 1; i < 4; i++) {
                  data[i] =  Ultrasonic_Sensor.read();
              }

              // Checksum verification
              unsigned char CS = data[0] + data[1] + data[2];
                if (data[3] == CS) {
                  distance = (data[1] << 8) + data[2];
                  unsigned long currentTime = millis(); // Current time

                  // calculate water level in cm
                  int waterLevel1 = TANK_HEIGHT_CM - (distance / 10); // convert mm to cm

                  // Buzzer condition: water level <= 10 cm or distance <= 50 cm
                  if (waterLevel1 <= 10 || (distance / 10) <= 5) {
                      if (!buzzerState) { // Only print if status changes
                          Serial.println("BUZZER ON: Ketinggian kritis terdeteksi!");
                          buzzerState = true;
                          playMelody(); // Playing melody
                      }
                      digitalWrite(BUZZER_PIN, HIGH); // Buzzer on
                  } else {
                      if (buzzerState) { // Only print if status changes
                          Serial.println("BUZZER OFF: Ketinggian aman.");
                          buzzerState = false;
                      }
                      digitalWrite(BUZZER_PIN, LOW); // Buzzer off
                  }

                  // Display data on Serial Monitor every 1 second
                  if (currentTime - lastPrintTime >= 1000) { // Check if 1 second has passed
                      Serial.print("distance=");
                      Serial.print(distance / 10);
                      Serial.println(" cm");
                      
                      Serial.print("Water Level=");
                      Serial.print(waterLevel1);
                      Serial.println(" cm");
                      
                      lastPrintTime = currentTime; // Update last print time
                  }
              } else {
                  Serial.println("ERROR: Checksum mismatch");
              }
          }
      }

      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
          waterlevel = TANK_HEIGHT_CM - (distance / 10);;
          xSemaphoreGive(xMutex);
      }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// === FreeRTOS Tasks ===
// Tasks for reading sensor data periodically using FreeRTOS.

// Task: Read DS18B20 Temperature Sensor
void ds18b20Task(void * pvParameters) {
  for (;;) {
    ds18b20.requestTemperatures();
    float temp = ds18b20.getTempCByIndex(0);
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      temperatureDS18B20 = temp;
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Interval 2 second
  }
}

// Task: Read DHT Sensor (Temperature and Humidity)
void dhtTask(void * pvParameters) {
  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        humidityDHT = h;
        temperatureDHT = t;
        xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task: Read pH Sensor
void phTask(void * pvParameters) {
  for (;;) {
    float rawValue = analogRead(phSensorPin);
    float voltage = rawValue * (3.3 / 4095.0);
    float ph = 3.3 * voltage;
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      phSensorValue = ph;
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task: Read TDS Sensor (Nutrient Level)
void tdsTask(void * pvParameters) {
  for (;;) {
    float rawValue = analogRead(TdsSensorPin);
    float voltage = rawValue * (3.3 / 4095.0); // Cnvert ADC to voltage
    float tds = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      tdsValue = tds;
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}

// Task: Read Flow Meter (pulse counter)
// void calculateFlowRate(void *parameter) {
//   unsigned long lastTime = 0; // Menyimpan waktu terakhir untuk interval 1 detik

//   for (;;) { // Loop task untuk FreeRTOS
//     unsigned long currentTime = millis();

//     // Hitung flow rate dalam L/m
//     if (currentTime - lastTime >= 5000) { // Perbarui setiap 1 detik
//       flowRate = (pulseCount * 60) / flowFactor;

//       // Kirim hasil ke serial monitor
//       Serial.print("Flow Rate: ");
//       Serial.print(flowRate/1000);
//       Serial.println(" L/m");

//       // Reset hitungan pulsa untuk periode berikutnya
//       pulseCount = 0;
//       lastTime = currentTime;
//     }  

//     if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
//       waterflowdata = flowRate;
//       xSemaphoreGive(xMutex);
//     }
//     // Task delay untuk memberikan waktu ke tugas lain
//     vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2000 ms
//   }
// }

// void pulseCounter(){
//   // Increment the pulse counter
//     pulseCount++;
//   }

// Task: Read LCD Display 
void lcdTask(void * pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      //lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WT:");
      lcd.print(temperatureDS18B20, 1);
      lcd.print("C");

      lcd.setCursor(12, 0);
      lcd.print("PT:");
      lcd.print(temperatureDHT, 1);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("WF:");
      lcd.print(receivedFlowRate / 100.0, 1);
      lcd.print("L/m");

      lcd.setCursor(12, 1);
      lcd.print("WL:");
      lcd.print(waterlevel, 1);
      lcd.print("cm");

      lcd.setCursor(0, 2);
      lcd.print("N :");
      lcd.print(tdsValue, 1);
      lcd.print("ppm");

      if (WiFi.status() != WL_CONNECTED) {
      // Ketika tidak terhubung ke WiFi
        lcd.setCursor(12, 2);
        lcd.print("Status:");
        lcd.setCursor(12, 3);
        lcd.print("Offline  "); // Pastikan membersihkan area dengan spasi
      } else if (mode == "Manual") {
      // Ketika mode Manual dan terhubung ke WiFi
        lcd.setCursor(12, 2);
        lcd.print("S1:");
        lcd.print(pumpPhUpState ? "1" : "0");
        lcd.setCursor(16, 2);
        lcd.print("S2:");
        lcd.print(pumpPhDownState ? "1" : "0");

        lcd.setCursor(12, 3);
        lcd.print("S3:");
        lcd.print(pumpZatAState ? "1" : "0");
        lcd.setCursor(16, 3);
        lcd.print("S4:");
        lcd.print(pumpSprayingState ? "1" : "0");
      } else {
      // Ketika mode Auto dan terhubung ke WiFi
        lcd.setCursor(12, 2);
        lcd.print("Mode:");
        lcd.setCursor(12, 3);
        lcd.print("Auto    "); // Pastikan membersihkan area dengan spasi
      }

      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Interval pembaruan LCD 1 detik
  }
}

void sendSensorData() {
    if (!mqttClient.connected()) {
        Serial.println("MQTT disconnected. Skipping data send.");
        return;
    }

    // Membuat objek JSON
    StaticJsonDocument<256> jsonDoc;
    
    // Menggunakan nilai random untuk sensor sementara
        jsonDoc["ph_air"] = phSensorValue;
        jsonDoc["tds"] = tdsValue;
        jsonDoc["suhu_air"] = temperatureDS18B20;
        jsonDoc["laju_air"] = receivedFlowRate / 100.0;
        jsonDoc["volume_air"] = waterlevel;
        jsonDoc["panel_temp"] = temperatureDHT;
        jsonDoc["device_id"] = 1;                 // ID perangkat (misalnya 1)

    // Mengonversi JSON ke buffer string
    char jsonBuffer[256]; serializeJson(jsonDoc, jsonBuffer);

    // Mengirimkan data ke topik MQTT
    if (mqttClient.publish("SmartHydroponik/SensorData", jsonBuffer)) {
        Serial.println("Data sensor terkirim:");
        Serial.println(jsonBuffer);
    } else {
        Serial.println("Gagal mengirim data sensor.");
    }
}

// === Callback Data Receive to Espnow  ===
void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(receivedFlowRate)) {
    memcpy((void *)&receivedFlowRate, incomingData, sizeof(receivedFlowRate));
    Serial.print("Received Flow Rate: ");
    Serial.print(receivedFlowRate / 100.0);
    Serial.println(" L/min");
    xSemaphoreGive(xMutex);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  mqttClient.begin(mqtt_server, mqtt_port, wifiClient);
  mqttClient.onMessage(mqttCallback); 
  mqttClient.publish("SmartHydroponik/RequestSetting","Request");
  
  Wire.begin(16, 17);
  ds18b20.begin();
  dht.begin();
  
  Ultrasonic_Sensor.begin(9600, SERIAL_8N1, RX, TX);
  
  lcd.init(16,17);
  lcd.backlight();
  
  pinMode(BUTTON_S1, INPUT_PULLUP);
  pinMode(BUTTON_S2, INPUT_PULLUP);
  pinMode(BUTTON_S3, INPUT_PULLUP);
  // pinMode(BUTTON_S4, INPUT);

  pinMode(HT74HC595_OUT_EN, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  digitalWrite(BUZZER_PIN, LOW); 

  //pinMode(flowratepin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(flowratepin), pulseCounter, RISING); // Interrupt pada sinyal RISING

  HT74HC595 -> set(0, LOW, true);
  HT74HC595 -> set(1, LOW, true);
  HT74HC595 -> set(2, LOW, true);
  HT74HC595 -> set(3, LOW, true);
  HT74HC595 -> set(4, LOW, true);
  HT74HC595 -> set(5, LOW, true);

  // Wi-Fi initialization
  WiFi.mode(WIFI_STA); // ESP32 mode station (client)
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Inisialisasi ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Error");
    lcd.setCursor(0, 1);
    lcd.print("ESP-NOW Error");
    return;
  }
  esp_now_register_recv_cb(onDataReceive); // Register callback to receive data

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);
  }

  if (xTaskCreatePinnedToCore(controlPumpTask, "Pump Control Task", 2048, NULL, 5, NULL, 1) != pdPASS) {
    Serial.println("Failed to create Pump Control Task!");
  }
  if (xTaskCreatePinnedToCore(waterlevelTask, "Water Level Task", 2048, NULL, 4, NULL, 1) != pdPASS) {
    Serial.println("Failed to create Water Level Task!");
  }
  if (xTaskCreatePinnedToCore(ds18b20Task, "DS18B20 Task", 2048, NULL, 3, NULL, 0) != pdPASS) {
    Serial.println("Failed to create DS18B20 Task!");
  }
  if (xTaskCreatePinnedToCore(phTask, "pH Task", 2048, NULL, 3, NULL, 0) != pdPASS) {
    Serial.println("Failed to create pH Task!");
  }
  if (xTaskCreatePinnedToCore(tdsTask, "TDS Task", 2048, NULL, 3, NULL, 0) != pdPASS) {
    Serial.println("Failed to create TDS Task!");
  }
  if (xTaskCreatePinnedToCore(dhtTask, "DHT Task", 2048, NULL, 2, NULL, 0) != pdPASS) {
    Serial.println("Failed to create DHT Task!");
  }
  if (xTaskCreatePinnedToCore(lcdTask, "LCD Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create LCD Task!");
  }
  // if (xTaskCreatePinnedToCore(calculateFlowRate, "Flow Rate Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
  //   Serial.println("Failed to create Flow Rate Task!");
  // }

  HT74HC595 -> set(0, LOW, true);
  HT74HC595 -> set(1, LOW, true);
  HT74HC595 -> set(2, LOW, true);
  HT74HC595 -> set(3, LOW, true);
  HT74HC595 -> set(4, LOW, true);
  HT74HC595 -> set(5, LOW, true);
}
void loop() { 
  // Ensure WiFi and MQTT connection
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();  // Try to connect to the wifi device again
  }

  if (!mqttClient.connected()) {
    connectMQTT();  // Try to connect to the MQTT broker again
  }

  sendSensorData(); // Send sensor data to MQTT broker
  
  if (mode == "Automatis") {
    auto_pH_pump();
    auto_Nutrient_pump();
  }
    Serial.print("Limit PH Min: ");
    Serial.println(limitPhMin);
    Serial.print("Limit PH Max: ");
    Serial.println(limitPhMax);
    Serial.print("Limit Nutrisi Min: ");
    Serial.println(limitNutrisiMin);
    Serial.print("Limit Nutrisi Max: ");
    Serial.println(limitNutrisiMax);
    Serial.print("Tangki Air: ");
    Serial.println(tangkiAir);
    vTaskDelay(pdMS_TO_TICKS(2000));
}
