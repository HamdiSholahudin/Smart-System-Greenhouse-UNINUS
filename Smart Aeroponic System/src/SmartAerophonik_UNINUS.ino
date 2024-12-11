#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include "ShiftRegister74HC595_NonTemplate.h"
#include <DFRobot_SHT20.h>

// === Konfigurasi WiFi dan MQTT ===
const char * ssid = "SMART GREENHOUSE UNINUS (2Ghz)";
const char * password = "UNINUSLAB530";
const char * mqtt_server = "88.222.214.56";
const int mqtt_port = 1883;
const char * mqtt_user = "SGH_1.0";
const char * mqtt_password = "Teknik_Pertanian_2025";
const char * clientID = "SmartAerophonik_UNINUS";
const char * mqttSensorTopic = "SmartAerophonik/SensorData";
const char * mqttControlTopic = "SmartAerophonik/Control_Send";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// === Definisi Tombol dan Relay ===
#define BUTTON_S1 8 // Tombol PH UP
#define BUTTON_S2 4 // Tombol PH DOWN
#define BUTTON_S3 14 // Tombol ZAT AB
#define BUTTON_S4 12 // Tombol Spraying
#define RELAY_PH_UP 3
#define RELAY_PH_DOWN 2
#define RELAY_ZAT_A 1
#define RELAY_ZAT_B 0
#define RELAY_SPRAYING 4
#define SENSOR 27
#define DS18B20PIN 9
#define FLOW_SENSOR_PIN 27

// Variabel untuk menghitung laju aliran
volatile unsigned int flowFrequency = 0;
float flowRate = 0.0;
float flowRateMps = 0.0;
unsigned long totalMilliLiter = 0;
unsigned long lastFlowCalculationTime = 0;

bool pumpPhUpState = false, pumpPhDownState = false, pumpZatAState = false, pumpZatBState = false, pumpSprayingState = false;
bool lastButtonS1State = HIGH, lastButtonS2State = HIGH, lastButtonS3State = HIGH, lastButtonS4State = HIGH;

OneWire oneWire(DS18B20PIN);
DallasTemperature ds18b20( & oneWire);
DFRobot_SHT20 sht20( & Wire, SHT20_I2C_ADDR);

#define DHTPIN 1
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define RXD 36
#define TXD 37
HardwareSerial Ultrasonic_Sensor(2);

// Sensor TDS dan pH
#define TdsSensorPin 11
#define phSensorPin 13

// Variabel Global
float phSensorValue = 0.0, tdsValue = 0.0, temperatureDS18B20 = 0.0;
float temperatureDHT = 0.0, humidityDHT = 0.0, humiditySHT20 = 0.0, tangkiAir = 0, temperatureSHT20 = 0;
int distance = 0;

float limitPhMin = 0;
float limitPhMax = 0;
float limitNutrisiMin = 0;
float limitNutrisiMax = 0;
float start_spray = 0;
float end_spray = 0;
float currentPhValue = 0;
float currentNutrisiValue = 0;

String mode = "Manual";

unsigned char data_buffer[4] = {
  0
};

SemaphoreHandle_t xMutex;

// Shift Register
#define HT74HC595_CLOCK 5
#define HT74HC595_LATCH 6
#define HT74HC595_DATA 7
#define HT74HC595_OUT_EN 4
std::shared_ptr < ShiftRegister74HC595_NonTemplate > HT74HC595 =
  std::make_shared < ShiftRegister74HC595_NonTemplate > (6, HT74HC595_DATA, HT74HC595_CLOCK, HT74HC595_LATCH);

// === TASK RTOS ===
void ds18b20Task(void * pvParameters) {
  for (;;) {
    ds18b20.requestTemperatures();
    float temp = ds18b20.getTempCByIndex(0);
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      temperatureDS18B20 = temp;
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(3000)); // Interval 3 detik
  }
}
// DHT Task
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

void setupWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  while (WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback); // Set callback function for incoming messages

  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("SmartHydroponik_Uninus", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT Broker!");
      mqttClient.subscribe("SmartAerophonik/Settings");
      mqttClient.subscribe("SmartAerophonik/Control"); // Subscribe to topic
    } else {
      Serial.print("Failed, error code: ");
      Serial.println(mqttClient.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void mqttCallback(char * topic, byte * payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char) payload[i];
  }

  // Debug: Menampilkan topik dan pesan yang diterima
  Serial.print("Received message on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);

  // Parsing JSON
  StaticJsonDocument < 512 > doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.f_str());
    return;
  }

  if (String(topic) == "SmartAerophonik/Settings") {
    limitPhMin = doc["Limit_ph_min"];
    limitPhMax = doc["Limit_ph_max"];
    limitNutrisiMin = doc["Limit_nutrisi_min"];
    limitNutrisiMax = doc["Limit_nutrisi_max"];
    tangkiAir = doc["tangki_air"];
    start_spray = doc["waktu_spary_start"];
    end_spray = doc["waktu_spary_end"];
  }

  if (String(topic) == "SmartAerophonik/Control") {
    String type = doc["type"];

    // Cek apakah tipe yang diterima adalah "control"
    if (type == "control") {
      String pompa = doc["pompa"];
      int status = doc["status"];
      String device = doc["device"];

      if (pompa == "Pompa_PHUP") {
        pumpPhUpState = (status == 1);
        HT74HC595 -> set(RELAY_PH_UP, pumpPhUpState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_PHUP", pumpPhUpState);
      } else if (pompa == "Pompa_PHDOWN") {
        pumpPhDownState = (status == 1);
        HT74HC595 -> set(RELAY_PH_DOWN, pumpPhDownState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_PHDOWN", pumpPhDownState);
      } else if (pompa == "Pompa_Nutrisi") {
        pumpZatAState = (status == 1);
        pumpZatBState = (status == 1);
        HT74HC595 -> set(RELAY_ZAT_A, pumpZatAState ? HIGH : LOW, true);
        HT74HC595 -> set(RELAY_ZAT_B, pumpZatBState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_Nutrisi", pumpZatBState);
      } else if (pompa == "Pompa_Spraying") {
        pumpSprayingState = (status == 1);
        HT74HC595 -> set(RELAY_SPRAYING, pumpSprayingState ? HIGH : LOW, true);
        // sendPumpStatus("Pompa_Spraying", pumpSprayingState);
      }
    } else if (type == "mode") {
      int dump =  doc["status"];
      mode = dump ? "Automatis" : "Manual";
      Serial.println("Mode set to: " + mode);
    }
  }
}


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


void sht20Task(void * pvParameters) {
  for (;;) {
    float temp = sht20.readTemperature();
    float hum = sht20.readHumidity();

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      temperatureSHT20 = temp;
      humiditySHT20 = hum;
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
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
    bool currentButtonS4State = digitalRead(BUTTON_S4);
    if (currentButtonS4State == LOW && lastButtonS4State == HIGH && (currentTime - lastDebounceTimeS4 > debounceDelay)) {
      pumpSprayingState = !pumpSprayingState;
      HT74HC595 -> set(RELAY_SPRAYING, pumpSprayingState ? HIGH : LOW, true);
      sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
      lastDebounceTimeS4 = currentTime;
    }
    lastButtonS4State = currentButtonS4State;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void autoSpary() {
  unsigned long currentTime = millis();
  unsigned long startMillis = start_spray * 60000;
  unsigned long endMillis = end_spray * 60000;

  if (currentTime >= startMillis && currentTime <= endMillis) {
    if (!pumpSprayingState) {
      pumpSprayingState = true;
      HT74HC595 -> set(RELAY_SPRAYING, HIGH, true);
      sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
    }
  } else {
    if (pumpSprayingState) {
      pumpSprayingState = false;
      HT74HC595 -> set(RELAY_SPRAYING, LOW, true);
      sendPumpStatus("Pompa_Spraying", pumpSprayingState, "status");
    }
  }
}

void autoPompaPH() {
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

void autoNutrisi() {
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

// Ultrasonic Task
void ultrasonicTask(void * pvParameters) {
  for (;;) {
    if (Ultrasonic_Sensor.available() > 0) {
      vTaskDelay(pdMS_TO_TICKS(4));
      if (Ultrasonic_Sensor.read() == 0xff) {
        data_buffer[0] = 0xff;
        for (int i = 1; i < 4; i++) {
          data_buffer[i] = Ultrasonic_Sensor.read();
        }
        unsigned char CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
        if (data_buffer[3] == CS) {
          int dist = (data_buffer[1] << 8) + data_buffer[2] / 10;
          if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            distance = dist;
            xSemaphoreGive(xMutex);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// pH Sensor Task
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

// TDS Sensor Task
void tdsTask(void * pvParameters) {
  for (;;) {
    float rawValue = analogRead(TdsSensorPin);
    float voltage = rawValue * (3.3 / 4095.0); // Konversi ADC ke volt
    float tds = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage);

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      tdsValue = tds;
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); // Interval pembacaan 2 detik
  }
}

// LCD Display Task
void lcdTask(void * pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperatureDS18B20, 1);
      lcd.print(" C");

      lcd.setCursor(0, 1);
      lcd.print("Panel: ");
      lcd.print(temperatureDHT, 1);
      lcd.print("C ");
      lcd.print(humidityDHT, 1);
      lcd.print("%");

      lcd.setCursor(0, 2);
      lcd.print("hum: ");
      lcd.print(humiditySHT20, 1);
      lcd.print("% ");

      lcd.setCursor(0, 3);
      lcd.print("TDS: ");
      lcd.print(tdsValue, 1);
      lcd.print(" pH: ");
      lcd.print(phSensorValue, 2);

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
  StaticJsonDocument < 256 > jsonDoc;

  // Menggunakan nilai random untuk sensor sementara
  jsonDoc["ph_air"] = phSensorValue;
  jsonDoc["tds"] = tdsValue;
  jsonDoc["suhu_air"] = temperatureDS18B20;
  jsonDoc["kelembaban_udara"] = humiditySHT20;
  jsonDoc["volume_air"] = distance;
  jsonDoc["panel_temp"] = temperatureDHT;
  jsonDoc["device_id"] = 2;

  // Mengonversi JSON ke buffer string
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);

  // Mengirimkan data ke topik MQTT
  if (mqttClient.publish("SmartAerophonik/SensorData", jsonBuffer)) {
    Serial.println("Data sensor terkirim:");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("Gagal mengirim data sensor.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  connectMQTT();
  mqttClient.publish("SmartAeroponik/RequestSetting", "Request");
  ds18b20.begin();
  dht.begin();
  Ultrasonic_Sensor.begin(9600, SERIAL_8N1, RX, TX);
  Wire.begin(16, 17);
  lcd.begin(20, 4);
  lcd.backlight();
  pinMode(TdsSensorPin, INPUT);
  pinMode(HT74HC595_OUT_EN, OUTPUT);
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
  if (xTaskCreatePinnedToCore(ds18b20Task, "DS18B20 Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create DS18B20 Task!");
  }
  if (xTaskCreatePinnedToCore(dhtTask, "DHT Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create DHT Task!");
  }
  if (xTaskCreatePinnedToCore(sht20Task, "Sht20 Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create Waterflow Task!");
  }
  if (xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create Ultrasonic Task!");
  }
  if (xTaskCreatePinnedToCore(phTask, "pH Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create pH Task!");
  }
  if (xTaskCreatePinnedToCore(tdsTask, "TDS Task", 2048, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create TDS Task!");
  }
  if (xTaskCreatePinnedToCore(lcdTask, "LCD Task", 4098, NULL, 1, NULL, 1) != pdPASS) {
    Serial.println("Failed to create LCD Task!");
  }
  xTaskCreatePinnedToCore(controlPumpTask, "Pump Control Task", 2048, NULL, 1, NULL, 1);
}

void loop() {

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
  Serial.print("Waktu Spray Start: ");
  Serial.println(start_spray);
  Serial.print("Waktu Spray End: ");
  Serial.println(end_spray);
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  sendSensorData();
  mqttClient.loop();
  if (mode == "Automatic") {
    autoPompaPH();
    autoNutrisi();
    autoSpary();
  }
}