//Code for generating the Smart Aerophonik System (UNINUS)

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
#include <Adafruit_AHTX0.h>
//#include <Adafruit_VL6180X.h>
#include <HardwareSerial.h>
//#include "pin_config.h"

// === WiFi configuration ===
const char * ssid = "SMART GREENHOUSE UNINUS (2Ghz)";
const char * password = "UNINUSLAB530";

// === MQTT configuration ===
const char * mqtt_server = "88.222.214.56";
const int mqtt_port = 1883;
const char * mqtt_user = "SGH_1.0";
const char * mqtt_password = "Teknik_Pertanian_2025";
const char * clientID = "SmartAerophonik_UNINUS";
const char * mqttSensorTopic = "SmartAerophonik/SensorData";
const char * mqttControlTopic = "SmartAerophonik/Control_Send";

// === WiFi and MQTT client objects ===
WiFiClient net;
MQTTClient mqttClient;

// === Button and Relay Definitions ===
// - Buttons are used for manual control (PH UP, PH DOWN, Nutrient AB, Spraying)
// - Relays are used to control hardware (pumps or actuators)
#define BUTTON_S1 4                  // GPIO pin for PH UP button
#define BUTTON_S2 8                  // GPIO pin for PH DOWN button
#define BUTTON_S3 14                 // GPIO pin for Nutrient AB button
#define BUTTON_S4 12                 // GPIO pin for Spraying button
#define RELAY_PH_UP 0                // GPIO pin for PH UP relay
#define RELAY_PH_DOWN 1              // GPIO pin for PH DOWN relay
#define RELAY_ZAT_A 2                // GPIO pin for Nutrient A relay
#define RELAY_ZAT_B 3                // GPIO pin for Nutrient B relay
#define RELAY_SPRAYING 4             // GPIO pin for Spraying relay
#define DS18B20PIN 9                 // GPIO pin for DS18B20 temperature sensor
#define DHTPIN 1                     // GPIO pin for the DHT sensor
#define DHTTYPE DHT22                // DHT sensor type (DHT22)
#define TdsSensorPin 11              // GPIO pin for the TDS sensor
#define phSensorPin 13               // GPIO pin for the pH sensor
#define BUZZER_PIN 15                 // GPIO pin for the buzzer
#define RX 44                       // RX pin for the ultrasonic sensor
#define TX 43                       // TX pin for the ultrasonic sensor
#define Relay_Spraying 2

// === Pump and Button States ===
// Variables to track the state of pumps and the last state of buttons.
bool pumpPhUpState = false, pumpPhDownState = false, pumpZatAState = false, pumpZatBState = false, pumpSprayingState = false;
bool lastButtonS1State = HIGH, lastButtonS2State = HIGH, lastButtonS3State = HIGH, lastButtonS4State = HIGH;

// === Sensor and Device Initialization ===
// Initialization of sensors and devices.
DHT dht(DHTPIN, DHTTYPE);            // DHT sensor instance
OneWire oneWire(DS18B20PIN);          // OneWire instance for DS18B20 sensor
DallasTemperature ds18b20(&oneWire);  // DS18B20 temperature sensor instance
Adafruit_AHTX0 aht;                   // AHTX0 temperature and humidity sensor instance
LiquidCrystal_I2C lcd(0x27, 20, 4);   // LCD with I2C address 0x27, 20x4 display
//Adafruit_VL6180X vl6180x = Adafruit_VL6180X();  // VL6180X distance sensor instance
#define TANK_HEIGHT_CM 39             // Water tank height in centimeters
//#define MAX_RANGE 500                 // maximum range of the VL6180X sensor in mm
HardwareSerial Ultrasonic_Sensor(2);  // Serial instance for ultrasonic sensor (UART2)

// === Data Buffers and Timing ===
// Variables for sensor data processing and timing.
unsigned char data[4] = {};           // Buffer for ultrasonic sensor data
unsigned long lastPrintTime = 0;      // Last time data was displayed
bool buzzerState = false;             // Current state of the buzzer

// Variable for automatic restart interval after 12 hours (43200000 ms)
unsigned long previousRestartMillis = 0;
const long restartInterval = 43200000;  // 12 hours in milliseconds (12 * 60 * 60 * 1000)

// === Global Variables ===
// Variables to store sensor readings and control parameters.
float phSensorValue = 0.0, temperatureDS18B20 = 0.0;
float temperatureDHT = 0.0, humidityDHT = 0.0, RHLevel = 0.0;
int waterLevel = 0, tdsValue = 0;

// Control limits for pH and nutrients
float limitPhMin, limitPhMax;         // pH range limits
int limitNutrisiMin, limitNutrisiMax; // Nutrient level range limits
float start_spray, end_spray;         // Spray timing parameters
float currentPhValue;                 // Current pH value
int currentNutrisiValue;              // Current nutrient value
//int tangkiAir;                        // Water tank status

// Operating mode
String mode = "Manual";               // System mode (Manual/Automatic)
String device;
// === FreeRTOS Mutex ===
// Mutex for protecting shared resources in FreeRTOS tasks.
SemaphoreHandle_t xMutex;

// === Shift Register Initialization ===
// HT74HC595 PIN
#define HT74HC595_CLOCK 5
#define HT74HC595_LATCH 6
#define HT74HC595_DATA 7
#define HT74HC595_OUT_EN 4
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
void controlRelay(int relayPin, bool state, bool enableLogging) {
    digitalWrite(relayPin, state ? HIGH : LOW);  // Mengontrol relay berdasarkan state

    // Jika enableLogging true, tampilkan log status relay
    if (enableLogging) {
        Serial.print("Relay Pin: ");
        Serial.print(relayPin);
        Serial.print(" Status: ");
        Serial.println(state ? "ON" : "OFF");
    }
}

// === MQTT Callback Function ===
void mqttCallback(String &topic, String &payload) {
  Serial.println("Pesan diterima:");
  Serial.println("Topik: " + topic);
  Serial.println("Payload: " + payload);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.println("Gagal mendeserialisasi JSON");
    return;
  }

  // Logika untuk topik "SmartAerophonik/Settings"
  if (topic == "SmartAerophonik/Settings") {
    limitPhMin = doc["Limit_ph_min"];
    limitPhMax = doc["Limit_ph_max"];
    limitNutrisiMin = doc["Limit_nutrisi_min"];
    limitNutrisiMax = doc["Limit_nutrisi_max"];
    start_spray = doc["waktu_spary_start"];
    end_spray = doc["waktu_spary_end"];
     device = doc["device"].as < String > ();

    Serial.print("Limit PH Min: ");
    Serial.println(limitPhMin);
    Serial.print("Limit PH Max: ");
    Serial.println(limitPhMax);
    Serial.print("Limit Nutrisi Min: ");
    Serial.println(limitNutrisiMin);
    Serial.print("Limit Nutrisi Max: ");
    Serial.println(limitNutrisiMax);
    Serial.print("Waktu Spray Start: ");
    Serial.println(start_spray);
    Serial.print("Waktu Spray End: ");
    Serial.println(end_spray);
  }

  // Logika untuk topik "SmartAerophonik/Control"
  if (topic == "SmartAerophonik/Control") {
    String type = doc["type"];
    String pompa = doc["pompa"];
    int status = doc["status"];
    String device = doc["device"];

    if (type == "control") {
      if (pompa == "Pompa_PHUP") {
        pumpPhUpState = (status == 1);
        // HT74HC595->set(RELAY_PH_UP, pumpPhUpState ? HIGH : LOW, true);
      } else if (pompa == "Pompa_PHDOWN") {
        pumpPhDownState = (status == 1);
        // HT74HC595->set(RELAY_PH_DOWN, pumpPhDownState ? HIGH : LOW, true);
      } else if (pompa == "Pompa_Nutrisi") {
        pumpZatAState = (status == 1);
        pumpZatBState = (status == 1);
        // HT74HC595->set(RELAY_ZAT_A, pumpZatAState ? HIGH : LOW, true);
        // HT74HC595->set(RELAY_ZAT_B, pumpZatBState ? HIGH : LOW, true);
      } else if (pompa == "Pompa_Spraying") {
        pumpSprayingState = (status == 1);
        // HT74HC595->set(RELAY_SPRAYING, pumpSprayingState ? HIGH : LOW, true);
      }
    } else if (type == "mode") {
      if (doc.containsKey("mode")) {
        mode = doc["mode"].as<String>();
        Serial.println("Mode set to: " + mode);
      } else {
        Serial.println("Mode key not found in JSON");
      }
    }
  }
}


// === MQTT Connection Function ===
void connectMQTT() {
  while (WiFi.status() != WL_CONNECTED) {
    setupWiFi(); //ensure WiFi connection
  }

  mqttClient.begin(mqtt_server, mqtt_port, net);
  mqttClient.onMessage(mqttCallback); // Set callback function for incoming messages

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect(clientID, mqtt_user, mqtt_password)) {
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

// === Send Pump Status Function ===
void sendPumpStatus(const char * pumpName, bool state,const char * type) {
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected. Skipping status update.");
    return;
  }

  StaticJsonDocument < 128 > jsonDoc;
  jsonDoc["type"] = type;
  jsonDoc["pompa"] = pumpName;
  jsonDoc["status"] = state ? 1 : 0;
  jsonDoc["device"] = "Aerophonik";

  char jsonBuffer[128];
  serializeJson(jsonDoc, jsonBuffer);

  if (mqttClient.publish(mqttControlTopic, jsonBuffer)) {
    Serial.printf("Status %s terkirim: %s\n", pumpName, state ? "ON" : "OFF");
  } else {
    Serial.printf("Gagal mengirim status %s!\n", pumpName);
  }
}

// === Send ControlPump Function ===
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
    bool currentButtonS4State = digitalRead(BUTTON_S4);
    if (currentButtonS4State == LOW && lastButtonS4State == HIGH && (currentTime - lastDebounceTimeS4 > debounceDelay)) {
      pumpSprayingState = !pumpSprayingState;
      //HT74HC595 -> set(RELAY_SPRAYING, pumpSprayingState ? HIGH : LOW, true);
      controlRelay(Relay_Spraying, pumpSprayingState ? HIGH : LOW, true);
      sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
      lastDebounceTimeS4 = currentTime;
    }
    lastButtonS4State = currentButtonS4State;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// === Auto Control Spray Funtion ===
void autoSpray_pump() {
  unsigned long currentTime = millis();
  unsigned long startMillis = start_spray * 60000;
  unsigned long endMillis = end_spray * 60000;

  if (currentTime >= startMillis && currentTime <= endMillis) {
    if (!pumpSprayingState) {
      pumpSprayingState = true;
      controlRelay(Relay_Spraying, HIGH, true);
      //HT74HC595 -> set(RELAY_SPRAYING, HIGH, true);
      sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
    }
  } else {
    if (pumpSprayingState) {
      pumpSprayingState = false;
      controlRelay(Relay_Spraying, LOW, true);
      //HT74HC595 -> set(RELAY_SPRAYING, LOW, true);
      sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
    }
  }
}

// === Auto Control pH Pump Function ===
void auto_pH_pump() {
  if (phSensorValue < limitPhMin) {
    if (!pumpPhUpState) {
      pumpPhUpState = true;
      HT74HC595 -> set(RELAY_PH_UP, HIGH, true);
      sendPumpStatus("Pompa_PHUP", pumpPhUpState, "status");
      Serial.println("Pompa PH UP dihidupkan (pH terlalu rendah)");
    }
  } else if (phSensorValue > limitPhMax) {
    if (!pumpPhDownState) {
      pumpPhDownState = true;
      HT74HC595 -> set(RELAY_PH_DOWN, HIGH, true);
      sendPumpStatus("Pompa_PHDOWN", pumpPhDownState, "status");
      Serial.println("Pompa PH DOWN dihidupkan (pH terlalu tinggi)");
    }
  } else {
    if (pumpPhUpState || pumpPhDownState) {
      pumpPhUpState = false;
      pumpPhDownState = false;
      HT74HC595 -> set(RELAY_PH_UP, LOW, true);
      HT74HC595 -> set(RELAY_PH_DOWN, LOW, true);
      sendPumpStatus("Pompa_PHUP", pumpPhUpState, "status");
      sendPumpStatus("Pompa_PHDOWN", pumpPhDownState, "status");
      Serial.println("Pompa PH dimatikan (pH dalam batas normal)");
    }
  }
}

// === Auto Control Nutrient Pump Function ===
void auto_Nutrient_pump() {
  if (tdsValue < limitNutrisiMin) {
    if (!pumpZatAState && !pumpZatBState) {
      pumpZatAState = true;
      pumpZatBState = true;
      HT74HC595 -> set(RELAY_ZAT_A, HIGH, true);
      HT74HC595 -> set(RELAY_ZAT_B, HIGH, true);
      sendPumpStatus("Pompa_Nutrisi", pumpZatBState, "status");
      Serial.println("Pompa Nutrisi dihidupkan (Nutrisi terlalu rendah)");
    }
  } else if (tdsValue > limitNutrisiMax) {
    if (pumpZatAState && pumpZatBState) {
      pumpZatAState = false;
      pumpZatBState = false;
      HT74HC595 -> set(RELAY_ZAT_A, LOW, true);
      HT74HC595 -> set(RELAY_ZAT_B, LOW, true);
      sendPumpStatus("Pompa_Nutrisi", pumpZatBState, "status");
      Serial.println("Pompa Nutrisi dimatikan (Nutrisi cukup tinggi)");
    }
  }
}

void waterlevelTask(void *pvParameters) {
  int distance;
  for (;;) {
      if (Ultrasonic_Sensor.available() >= 4) {
            if (Ultrasonic_Sensor.read() == 0xFF) {
                data[0] = 0xFF;
                for (int i = 1; i < 4; i++) {
                    data[i] = Ultrasonic_Sensor.read();
                }

                // Verifikasi checksum
                unsigned char CS = data[0] + data[1] + data[2];
                if (data[3] == CS) {
                    distance = (data[1] << 8) + data[2];
                    unsigned long currentTime = millis(); // Waktu saat ini

                    // Hitung ketinggian air dalam tangki
                    int waterLevel = TANK_HEIGHT_CM - (distance / 10); // Konversi mm ke cm

                    // Kondisi untuk buzzer
                    if (waterLevel <= 10 || (distance / 10) <= 10) {
                        if (!buzzerState) { // Hanya cetak jika status berubah
                            Serial.println("BUZZER ON: Ketinggian kritis terdeteksi!");
                            buzzerState = true;
                        }
                        digitalWrite(BUZZER_PIN, HIGH); // Buzzer menyala
                    } else {
                        if (buzzerState) { // Hanya cetak jika status berubah
                            Serial.println("BUZZER OFF: Ketinggian aman.");
                            buzzerState = false;
                        }
                        digitalWrite(BUZZER_PIN, LOW); // Buzzer mati
                    }

                    // Tampilkan data di Serial Monitor setiap 1 detik
                    if (currentTime - lastPrintTime >= 1000) { // Cek jika 1 detik telah berlalu
                        Serial.print("distance=");
                        Serial.print(distance / 10);
                        Serial.println(" cm");
                        
                        Serial.print("Water Level=");
                        Serial.print(waterLevel);
                        Serial.println(" cm");
                        
                        lastPrintTime = currentTime; // Update waktu terakhir mencetak
                    }
                } else {
                    Serial.println("ERROR: Checksum mismatch");
                }
            }
        }


      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
          waterLevel = TANK_HEIGHT_CM - (distance / 10);;
          xSemaphoreGive(xMutex);
      }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// // Task: Read Water Level Sensor (VL6180X)
// void waterLevelTask(void *pvParameters) {
//   while (true) {
//     // Read distance from sensor
//     uint8_t range = vl6180x.readRange();
//     uint8_t status = vl6180x.readRangeStatus();

//     if (status == VL6180X_ERROR_NONE) {
//       int distanceCm = range / 10; // Convert to cm
//       int calculatedWaterLevel = TANK_HEIGHT_CM - distanceCm;

//       // Synchronize with mutex
//       if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
//         waterLevel = calculatedWaterLevel; // Update water level
//         xSemaphoreGive(xMutex);
//       }

//       // Buzzer condition: water level <= 10 cm
//       if (waterLevel <= 10) {
//         if (!buzzerState) {
//           Serial.println("BUZZER ON: Critical water level detected!");
//           buzzerState = true;
//           playMelody();
//         }
//         digitalWrite(BUZZER_PIN, HIGH);
//       } else {
//         if (buzzerState) {
//           Serial.println("BUZZER OFF: Water level is safe.");
//           buzzerState = false;
//         }
//         digitalWrite(BUZZER_PIN, LOW);
//       }

//       // Display data on Serial Monitor every 1 second
//       unsigned long currentTime = millis();
//       if (currentTime - lastPrintTime >= 1000) {
//         Serial.print("Distance=");
//         Serial.print(distanceCm);
//         Serial.println(" cm");

//         Serial.print("Water Level=");
//         Serial.print(waterLevel);
//         Serial.println(" cm");

//         lastPrintTime = currentTime;
//       }
//     } else {
//       Serial.println("ERROR: Failed to read distance");
//     }

//     // Delay for stability
//     vTaskDelay(50 / portTICK_PERIOD_MS);
//   }
// }

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

// Task: Read RH Sensor (Humidity)
void readRHTask(void *parameter) {
  sensors_event_t tempEvent, humidityEvent;

  for (;;) {
    aht.getEvent(&humidityEvent, &tempEvent);
    if (!isnan(humidityEvent.relative_humidity)) {
      // Serial.print("RH: ");
      // Serial.print(humidityEvent.relative_humidity);
      // Serial.println(" %");

      //Updating RHLevel value in a thread-safe manner
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        RHLevel = humidityEvent.relative_humidity;
        xSemaphoreGive(xMutex);
      }
    } else {
      Serial.println("Error reading humidity data!");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

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
      lcd.print("RH:");
      lcd.print(RHLevel, 1);
      lcd.print("% ");

      lcd.setCursor(12, 1);
      lcd.print("WL:");
      lcd.print(waterLevel, 1);
      lcd.print("cm");

      lcd.setCursor(0, 2);
      lcd.print("N :");
      lcd.print(tdsValue, 1);
      lcd.print("ppm");

      lcd.setCursor(0, 3);
      lcd.print("pH:");
      lcd.print(phSensorValue, 2);

      if (WiFi.status() != WL_CONNECTED) {
      // When not connected to WiFi
        lcd.setCursor(12, 2);
        lcd.print("Status:");
        lcd.setCursor(12, 3);
        lcd.print("Offline  "); // Ensure to clean the area with space
      } else if (mode == "Manual") {
      // When in Manual mode and connected to WiFi
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
      // When in Auto mode and connected to WiFi
        lcd.setCursor(12, 2);
        lcd.print("Mode:");
        lcd.setCursor(12, 3);
        lcd.print("Auto    "); // Ensure to clean the area with space
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

  // Create a JSON document to store sensor data
  StaticJsonDocument < 256 > jsonDoc;
  jsonDoc["ph_air"] = phSensorValue;
  jsonDoc["tds"] = tdsValue;
  jsonDoc["suhu_air"] = temperatureDS18B20;
  jsonDoc["kelembaban_udara"] = RHLevel;
  jsonDoc["volume_air"] = waterLevel;
  jsonDoc["panel_temp"] = temperatureDHT;
  jsonDoc["device_id"] = 2;

  // Convert JSON document to string
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);

  // MQTT publish sensor data to "SmartAerophonik/SensorData" topic
  if (mqttClient.publish("SmartAerophonik/SensorData", jsonBuffer)) {
    Serial.println("Data sensor terkirim:");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("Gagal mengirim data sensor.");
  }
  vTaskDelay(pdMS_TO_TICKS(3000));
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  setupWiFi();
  
  mqttClient.begin(mqtt_server, mqtt_port, net);
  mqttClient.onMessage(mqttCallback);
  mqttClient.publish("SmartAeroponik/RequestSetting", "Request");
  
  Wire.begin(16, 17);
  ds18b20.begin();
  dht.begin();
  
  if (!aht.begin()) {
    Serial.println("Sensor AHT tidak ditemukan. Periksa koneksi!");}

  // if (!vl6180x.begin()) {
  //   Serial.println("Sensor VL6180X tidak ditemukan. Periksa koneksi!");
  //   while (true) {
  //     delay(100); // Wait for sensor to be connected
  //   }
  // }

  Ultrasonic_Sensor.begin(9600, SERIAL_8N1, RX, TX);
  
  lcd.init(16, 17);
  lcd.backlight();

  pinMode(Relay_Spraying, OUTPUT);
  // Inisialisasi status awal relay blower (mati)
  controlRelay(Relay_Spraying, false,false);

  pinMode(HT74HC595_OUT_EN, OUTPUT); 
  pinMode(TdsSensorPin, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  digitalWrite(BUZZER_PIN, LOW); 

  pinMode(BUTTON_S1, INPUT_PULLUP);
  pinMode(BUTTON_S2, INPUT_PULLUP);
  pinMode(BUTTON_S3, INPUT_PULLUP);
  pinMode(BUTTON_S4, INPUT_PULLUP);

  HT74HC595 -> set(0, LOW, true);
  HT74HC595 -> set(1, LOW, true);
  HT74HC595 -> set(2, LOW, true);
  HT74HC595 -> set(3, LOW, true);
  HT74HC595 -> set(4, LOW, true);
  HT74HC595 -> set(5, LOW, true);

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
  if (xTaskCreatePinnedToCore(readRHTask, "RH Task", 2048, NULL, 1, NULL, 0) != pdPASS) {
    Serial.println("Failed to create RH Task!");
  }
}

void loop() {
  // Ensure WiFi and MQTT connection
  if (WiFi.status() != WL_CONNECTED) {
    setupWiFi();  // Try to connect to the wifi device again
  }

  if (!mqttClient.connected()) {
    connectMQTT();  // Try to connect to the MQTT broker again
  }

  mqttClient.loop(); // Keep the MQTT connection alive

  sendSensorData(); // Send sensor data to MQTT broker

  if (mode == "Automatic") {
    auto_pH_pump();
    auto_Nutrient_pump();
    autoSpray_pump();
  }

  unsigned long currentMillis = millis();
    // logical reset every 12 hours
    if (currentMillis - previousRestartMillis >= restartInterval) {
      previousRestartMillis = currentMillis;
      Serial.println("12 jam telah berlalu, merestart ESP32...");
      ESP.restart();  // Restart ESP32
    }
}
