#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "image_data.h"  // Include the image data
#include <IRremoteESP8266.h> 
#include <IRsend.h>

// Pin Definitions
const uint8_t PIN_MP3_TX = 17;      // DFPlayer Mini RX
const uint8_t PIN_MP3_RX = 16;      // DFPlayer Mini TX
const int trigPin = 2;              // Ultrasonic sensor trigger pin
const int echoPin = 15;             // Ultrasonic sensor echo pin
//const int relayPin = 5;             // Relay control pin
const int DHTPIN = 25;              // DHT22 sensor pin
#define DHTTYPE DHT22               // DHT type

// TFT Display Pins (ESP32)
#define TFT_CS   33                 // TFT chip select pin
#define TFT_RST  4                  // TFT reset pin
#define TFT_DC   21                 // TFT data/command pin

const int BuzzerPin = 27;  // Tentukan pin untuk buzzer
// IR LED Pin and Initialization
const uint16_t kIrLedPin = 26;  // Pin for IR LED
IRsend irsend(kIrLedPin);

const uint16_t irCodeOn[] = {
  1310, 380, 1314, 378, 488, 1204, 1310, 382, 1314, 390, 464, 1226, 458, 1234, 490, 1202, 464, 1230, 458,
  1232, 490, 1200, 1308, 7126, 1348, 392, 1298, 394, 454, 1236, 1300, 394, 1298, 404, 452, 1240, 426,
  1266, 426, 1264, 452, 1242, 426, 1264, 450, 1242, 1272, 8254, 1294, 398, 1296, 400, 448, 1244, 1294,
  424, 1246, 432, 448, 1244, 448, 1244, 424, 1270, 422, 1270, 422, 1270, 422, 1272, 1266, 7212, 1266,
  430, 1264, 450, 396, 1272, 1268, 450, 1242, 460, 394, 1296, 396, 1272, 418, 1296, 394, 1274, 440,
  1274, 394, 1274, 1266, 8282, 1242, 450, 1242
};

const uint16_t irCodeOff[] = {
  1316, 374, 1316, 378, 494, 1196, 1342, 352, 1316, 388, 468, 1226, 492, 1198, 470, 1222, 492, 1198, 494,
  1198, 494, 1198, 1340, 7136, 1338, 356, 1316, 376, 468, 1224, 1316, 376, 1316, 386, 492, 1198, 470,
  1222, 470, 1220, 494, 1198, 492, 1198, 496, 1196, 1318, 8208, 1340, 352, 1318, 376, 492, 1200, 1314,
  380, 1340, 360, 492, 1200, 494, 1198, 492, 1200, 492, 1198, 490, 1200, 492, 1202, 1340, 7140, 1340,
  352, 1342, 354, 490, 1202, 1340, 354, 1338, 364, 490, 1202, 490, 1202, 490, 1202, 490, 1202, 490,
  1202, 492, 1200, 1340, 8188, 1336, 358, 1338
};


// Initialize objects for peripherals
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);  // DFPlayer serial communication
DFRobotDFPlayerMini player;                             // DFPlayer Mini object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);  // TFT display object
DHT dht(DHTPIN, DHTTYPE);  // DHT sensor object

// Timing variables for alternating display
unsigned long previousMillis = 0;
const long interval = 3000;  // 3 seconds delay for alternating display
int displayMode = 0;         // 0 = image, 1 = temperature/humidity, 2 = distance

// Timing variables for AC control duration
unsigned long acOnStartTime = 0;
bool isAcOn = false;

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

// Function to draw distance on TFT
void drawDistance(long distance) {
  tft.setRotation(3);  // Set to landscape for the distance data
  tft.fillScreen(ST77XX_BLACK);  // Clear the screen

  // Draw distance value
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(5);
  tft.setCursor(40, 40);
  tft.print("DISTANCE: ");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(6);
  tft.setCursor(60, 90);
  tft.print(distance);
  tft.println(" cm");
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

  // Initialize IR sending
  irsend.begin();

  // Initialize DHT sensor
  dht.begin();

  // Pin configurations
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //pinMode(relayPin, OUTPUT);

  // Ensure relay is off initially
  //digitalWrite(relayPin, LOW);

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

// // Function to draw a smiley face on the TFT
// void drawSmiley(int16_t x, int16_t y) {
//   // Draw face (yellow circle)
//   tft.fillCircle(x + 86, y + 160, 50, ST77XX_YELLOW); // Center at (86, 160) with radius 50

//   // Draw eyes (black circles)
//   tft.fillCircle(x + 70, y + 150, 8, ST77XX_BLACK);   // Left eye
//   tft.fillCircle(x + 50, y + 150, 8, ST77XX_BLACK);  // Right eye

//   // Draw mouth (smile using lines)
//   int mouthCenterX = x + 86;   // X position of the mouth center
//   int mouthCenterY = y + 180;   // Y position of the mouth center
//   int mouthRadius = 30;         // Radius of the mouth arc

//   // Draw smile using small line segments
//   for (int angle = 0; angle <= 180; angle += 5) { // Draw from 0 to 180 degrees
//     // Convert angle to radians
//     float rad = angle * (PI / 180);
//     int x1 = mouthCenterX + (mouthRadius * cos(rad)); // X coordinate
//     int y1 = mouthCenterY + (mouthRadius / 2 * sin(rad)); // Y coordinate (vertical stretch)
//     tft.drawPixel(x1, y1, ST77XX_BLACK); // Draw the pixel
//   }
// }

/// Function to activate relay and play audio
// Function to activate relay and play audio
// void activateRelayAndPlayAudio() {
//   digitalWrite(relayPin, HIGH);  // Turn on relay
//   Serial.println("Relay ON");

//   unsigned long startTime = millis();
  
//   // Measure and display distance every 1 second during audio play
//   while (millis() - startTime < 25000) {
//     long distance = measureDistance();  // Measure distance
//     Serial.print("Distance during audio: ");
//     Serial.println(distance);

//     drawDistance(distance);  // Show distance on TFT
//     delay(1000);  // Update distance every 1 second
//   }

//   player.play(1);  // Play the first audio track
//   Serial.println("Playing audio");
// }

// Function to draw countdown on TFT
void drawCountdown(int secondsLeft) {
  tft.setRotation(3);  // Set to landscape for countdown
  tft.fillScreen(ST77XX_BLACK);  // Clear the screen

  // Draw countdown title
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.setCursor(40, 40);
  tft.print("STERILIZATION: ");

  // Draw countdown seconds left
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(6);
  tft.setCursor(100, 90);
  tft.print(secondsLeft);
  tft.println(" s");
}

// Function to activate IR send for AC ON/OFF based on distance
// Function to control AC based on distance with 25 seconds duration
void controlACBasedOnDistance(long distance) {
  if (distance >= 60 && distance <= 80 && !isAcOn) {
    // Mengirim sinyal IR untuk ON dan mulai menghitung waktu
    Serial.println("Sending AC ON command via IR for 25 seconds");
    irsend.sendRaw(irCodeOn, sizeof(irCodeOn) / sizeof(irCodeOn[0]), 38);
    acOnStartTime = millis();
    isAcOn = true;

    // Memulai audio
    player.play(1);  // Start playing the first audio track
    Serial.println("Playing audio");

    int countdownTime = 25;  // Set countdown time in secondsit 
    unsigned long startTime = millis();

    // Countdown loop during audio play
    while (countdownTime > 0) {
      unsigned long currentMillis = millis();

      // Check if 1 second has passed
      if (currentMillis - startTime >= 1000) {
        startTime = currentMillis;  // Reset start time
        drawCountdown(countdownTime);  // Display the countdown
        
        // Bunyikan buzzer setiap detik
        tone(BuzzerPin, 1000, 200);  // Menghasilkan bunyi dengan frekuensi 1000 Hz selama 200 ms
        delay(200);  // Menunggu hingga buzzer selesai berbunyi

        countdownTime--;  // Decrease countdown by 1 second
      }
    }

    // Clear the countdown display after the audio ends
    tft.fillScreen(ST77XX_BLACK);
    Serial.println("Countdown finished");
    
    // Matikan buzzer setelah countdown selesai
    noTone(BuzzerPin);
  } 
  else if (isAcOn && (millis() - acOnStartTime >= 25000)) {
    // Mengirim sinyal IR untuk OFF setelah 25 detik
    Serial.println("Sending AC OFF command via IR");
    irsend.sendRaw(irCodeOff, sizeof(irCodeOff) / sizeof(irCodeOff[0]), 38);
    isAcOn = false;
     // Tambahkan delay 3 detik sebelum siap untuk menyalakan kembali
    delay(5000);
  }
 
}

// Function to activate relay, play audio, and display countdown
// void activateRelayAndPlayAudio() {
//   digitalWrite(relayPin, HIGH);  // Turn on relay
//   Serial.println("Relay ON");

//   player.play(1);  // Start playing the first audio track
//   Serial.println("Playing audio");

//   int countdownTime = 25;  // Set countdown time in seconds
//   unsigned long startTime = millis();

//   // Countdown loop during audio play
//   while (countdownTime > 0) {
//     unsigned long currentMillis = millis();

//     // Check if 1 second has passed
//     if (currentMillis - startTime >= 1000) {
//       startTime = currentMillis;  // Reset start time
//       drawCountdown(countdownTime);  // Display the countdown
//       countdownTime--;  // Decrease countdown by 1 second
//     }
//   }

//   // Clear the countdown display after the audio ends
//   tft.fillScreen(ST77XX_BLACK);
//   Serial.println("Countdown finished");
// }

// Function to deactivate relay
// void deactivateRelay() {
//   digitalWrite(relayPin, LOW);  // Turn off relay
//   Serial.println("Relay OFF");
// }

// Main loop function
void loop()
{
  long distance = measureDistance();  // Measure distance using ultrasonic sensor
  Serial.print("Distance: ");
  Serial.println(distance);

  // // Check distance for relay activation and specific display condition
  // if (distance >= 60 && distance <= 80) {
  //   activateRelayAndPlayAudio();
  // } else {
  //   deactivateRelay();
  // }

  // Check distance and control AC using IR
  controlACBasedOnDistance(distance);

  // Display distance on TFT if it's below 100 cm
  if (distance >= 60 && distance <= 80) {
    drawDistance(distance);  // Show distance on TFT when below 100 cm
  } else {
    // Alternate display every 3 seconds between image and temperature/humidity
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;  // Update the previousMillis time
      displayMode = (displayMode + 1) % 2;  // Toggle display mode

      if (displayMode == 0) {
        drawImage(0, 0);  // Show image
      } else if (displayMode == 1) {
        float temperature = dht.readTemperature();  // Get temperature in Celsius
        float humidity = dht.readHumidity();        // Get humidity in percentage

        if (isnan(temperature) || isnan(humidity)) {
          Serial.println("Failed to read from DHT sensor!");
        } else {
          drawTemperatureHumidity(temperature, humidity);  // Show temperature and humidity
        }
      }
    }
  }

  delay(500);  // Small delay before the next loop iteration
}