#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "image_data.h"  // Include the image data

// Pin Definitions
const uint8_t PIN_MP3_TX = 17;      // DFPlayer Mini RX
const uint8_t PIN_MP3_RX = 16;      // DFPlayer Mini TX
const int trigPin = 2;              // Ultrasonic sensor trigger pin
const int echoPin = 15;             // Ultrasonic sensor echo pin
const int relayPin = 5;             // Relay control pin
const int DHTPIN = 25;              // DHT22 sensor pin
#define DHTTYPE DHT22               // DHT type

// TFT Display Pins (ESP32)
#define TFT_CS   33                 // TFT chip select pin
#define TFT_RST  4                  // TFT reset pin
#define TFT_DC   21                 // TFT data/command pin

// Initialize objects for peripherals
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);  // DFPlayer serial communication
DFRobotDFPlayerMini player;                             // DFPlayer Mini object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);  // TFT display object
DHT dht(DHTPIN, DHTTYPE);  // DHT sensor object

// Timing variables for alternating display
unsigned long previousMillis = 0;
const long interval = 5000;  // 5 seconds delay for alternating display
bool displayImage = true;    // Flag to alternate between image and sensor data

// Function to draw image from image_data array
void drawImage(int16_t x, int16_t y) {
  tft.setRotation(2);  // Set to portrait for the image
  for (int16_t j = 0; j < 320; j++) {
    for (int16_t i = 0; i < 172; i++) {
      tft.drawPixel(x + i, y + j, image_data[j][i]);
    }
  }
}

// Function to draw temperature and humidity on TFT
void drawTemperatureHumidity(float temperature, float humidity) {
  tft.setRotation(3);  // Set to landscape for the DHT data
  tft.fillScreen(ST77XX_BLACK);  // Clear the screen

  // Draw temperature icon and value
  tft.fillCircle(30, 50, 20, ST77XX_RED);  // Temperature icon
  tft.setCursor(70, 40);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.print("Temp: ");
  tft.print(temperature);
  tft.println(" C");

  // Draw humidity icon and value
  tft.fillCircle(30, 120, 20, ST77XX_BLUE);  // Humidity icon
  tft.setCursor(70, 110);
  tft.print("Hum: ");
  tft.print(humidity);
  tft.println(" %");
}

// Function to initialize the TFT display
void setupDisplay() {
  tft.init(172, 320);  // Initialize display with resolution
  tft.setRotation(2);  // Set to portrait for the initial display (image)
  tft.fillScreen(ST77XX_BLACK);  // Clear the screen
  drawImage(0, 0);  // Display the initial image
}

// Setup function
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize software serial for DFPlayer Mini
  softwareSerial.begin(9600);

  // Initialize DHT sensor
  dht.begin();

  // Pin configurations
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relayPin, OUTPUT);

  // Ensure relay is off initially
  digitalWrite(relayPin, LOW);

  // Initialize DFPlayer Mini
  if (player.begin(softwareSerial)) {
    Serial.println("DFPlayer Mini ready.");
    player.volume(30);  // Set volume (0 to 30)
  } else {
    Serial.println("Failed to communicate with DFPlayer Mini.");
  }

  // Initialize the TFT display
  setupDisplay();

  delay(5000);  // Initial delay for stabilization
}

// Function to measure distance using the ultrasonic sensor
long measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);  // Get echo time in microseconds
  long distance = duration * 0.034 / 2;    // Convert time to distance in cm
  return distance;
}

// Function to activate relay and play audio
void activateRelayAndPlayAudio() {
  digitalWrite(relayPin, HIGH);  // Turn on relay
  Serial.println("Relay ON");
  
  player.play(1);  // Play the first audio track
  Serial.println("Playing audio");
  
  delay(10000);  // Prevent constant retriggering for 10 seconds
}

// Function to deactivate relay
void deactivateRelay() {
  digitalWrite(relayPin, LOW);  // Turn off relay
  Serial.println("Relay OFF");
}

// Main loop function
void loop() {
  long distance = measureDistance();  // Measure distance using ultrasonic sensor
  Serial.print("Distance: ");
  Serial.println(distance);

  // If distance is between 60 cm and 80 cm, activate relay and play audio
  if (distance >= 60 && distance <= 80) {
    activateRelayAndPlayAudio();
  } else {
    deactivateRelay();
  }

  // Alternate display every 5 seconds between image and temperature/humidity
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the previousMillis time
    displayImage = !displayImage;    // Toggle display state

    if (displayImage) {
      drawImage(0, 0);  // Show image
    } else {
      float temperature = dht.readTemperature();  // Get temperature in Celsius
      float humidity = dht.readHumidity();        // Get humidity in percentage

      if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT sensor!");
      } else {
        drawTemperatureHumidity(temperature, humidity);  // Show temperature and humidity
      }
    }
  }

  delay(500);  // Small delay before the next loop iteration
}
