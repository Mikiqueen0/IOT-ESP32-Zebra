#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>
#include "time.h"
#include <Arduino.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>

// Change to your own wifi
const char* ssid = "";
const char* password = "";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7 * 60 * 60;
const int   daylightOffset_sec = 0;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x2C, 0xBC, 0xBB, 0x85, 0x4A, 0xA4};

typedef struct struct_message {
  tm time;
  bool status;
} struct_message;

// Create a struct_message called myData
struct_message data;

esp_now_peer_info_t peerInfo;

// State variables
bool buttonPressed = false;
unsigned long interval = 1000; // 1 second

// Constants
const int COUNTDOWN_CAR = 2+1;
const int COUNTDOWN_PEDESTRIAN = 2;
const int COUNTDOWN_YELLOW = 2+1;

// Define pins for ultrasonic sensor
const int trigPin = 12;
const int echoPin = 14;

// IR sensor
const int IRPin = 26;

// Servo motor pin
Servo myServo;

// Define LED pins for traffic lights
const int greenLedCar = 18;
const int yellowLedCar = 5;
const int redLedCar = 17;
const int greenLedPedestrian = 16;
const int redLedPedestrian = 4;

const int onBoardLED = 2;

// Define button pins
const int buttonPin1 = 15;

const int buzzer = 25;

static int countdownCar = COUNTDOWN_CAR;
static int countdownPedestrian = COUNTDOWN_PEDESTRIAN;
static int state = 0;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G oled = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000
};

const int servoOpen = 90;
const int servoClose = 30;

// Setup function
void setup() {
  // Wire.setClock(100000); // Set I2C to 100 kHz

  // Initialize LEDs as OUTPUT
  pinMode(greenLedCar, OUTPUT);
  pinMode(yellowLedCar, OUTPUT);
  pinMode(redLedCar, OUTPUT);
  pinMode(greenLedPedestrian, OUTPUT);
  pinMode(redLedPedestrian, OUTPUT);

  pinMode(onBoardLED, OUTPUT);

  // Initialize buttons as INPUT with pull-up resistors
  pinMode(buttonPin1, INPUT_PULLUP);

  // Initialize LCDs
  // lcdCar.init();
  // lcdCar.backlight();
  
  delay(1000); // wait for the OLED to power up
  oled.begin(i2c_Address, true); // Address 0x3C default

  oled.setTextSize(2);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(0, 0);
  oled.println("Starting...");
  oled.display();

  delay(2000);

  // Clear the buffer.
  oled.clearDisplay();
  oled.display();

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  // ledcSetup(0, 1000, 8);       // Set up PWM channel 0 with 1 kHz frequency
  // ledcAttachPin(buzzer, 0);
  // ledcWriteTone(0, 0);
  pinMode(buzzer, OUTPUT);
  // delay(50);
  digitalWrite(buzzer, HIGH);

  // Initialize IR sensor pins
  pinMode(IRPin,INPUT);

  // Initialize servo motor
  myServo.write(servoOpen);
  myServo.attach(19);
  delay(1000);
  // myServo.write(90); // Start with barrier open

  // Start serial communication
  //Serial.begin(9600);

  // Start in initial state
  resetToDefaultState();

  //// Wifi setup
  Serial.begin(115200);

  // Connect to esp32cam
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    digitalWrite(onBoardLED, HIGH);
    delay(250);
    digitalWrite(onBoardLED, LOW);
    retry++;
    if (retry >= 15) {
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  digitalWrite(onBoardLED, HIGH);
  
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  localTime();

  // //disconnect WiFi as it's no longer needed
  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);
}

static unsigned int temp;
void loop() {
  unsigned long currentMillis = millis(); // Use this as the main reference for time
  
  switch (state) {
    case 0: // button not presssed
      setCarLight(0);
      setPedestrianLight(2);
      if (digitalRead(buttonPin1) == HIGH && !buttonPressed) {
        buttonPressed = true;
        countdownCar = COUNTDOWN_CAR;
        countdownPedestrian = COUNTDOWN_CAR + COUNTDOWN_YELLOW;
        state = 1;
      }
      break;
    case 1: // pressed
      nonBlockingCountdown(countdownCar, countdownPedestrian, currentMillis);
      if (countdownCar == 0) {
        countdownCar = COUNTDOWN_YELLOW;
        state = 2;
      }
      break;
    case 2: // yellow
      setCarLight(1);
      nonBlockingCountdown(countdownCar, countdownPedestrian, currentMillis);
      if (countdownCar == 0) {
        state = 3;
        countdownCar = COUNTDOWN_PEDESTRIAN + COUNTDOWN_YELLOW;
        countdownPedestrian = COUNTDOWN_PEDESTRIAN + COUNTDOWN_YELLOW;
        temp = countdownPedestrian;
      }
      break;
    case 3: // ped
      
      setCarLight(2);
      setPedestrianLight(0);
      myServo.write(servoClose);
      isCarCross(currentMillis);
      // if(countdown(2, currentMillis)) {
      //   // ledcWrite(0, 0);
      //   digitalWrite(buzzer, HIGH);
      //   // ledcWriteTone(0, 1000); // Generate a 1 kHz tone
      //   // delay(2000); 
      // }
      // else {
      //   // ledcWriteTone(0, 0);
      //   digitalWrite(buzzer, LOW);
      // }
      // if (currentMillis)
      nonBlockingCountdown(countdownCar, countdownPedestrian, currentMillis);
      if (countdownPedestrian > temp - 2) {
        Serial.println(countdownPedestrian);
        digitalWrite(buzzer, LOW);
      } else {
        digitalWrite(buzzer, HIGH);
      }
      if (countdownPedestrian == 0) {
        state = 4;
      }
      break;
    case 4: // green blink
      isCarCross(currentMillis);
      blinkingGreenLightState(currentMillis);
      nonBlockingCountdown(countdownCar, countdownPedestrian, currentMillis);
      if (countdownPedestrian == 0) {
        if (isCrosswalkOccupied(currentMillis)) {
          blinkingGreenLightState(currentMillis);
          oled.clearDisplay();
          oled.setTextSize(1);
          oled.setTextColor(SH110X_WHITE);
          oled.setCursor(0, 0);
          oled.println("Waiting for people");
          oled.println("to leave...");
          oled.display();

          // Serial.println("WAITING...");
        } else {
          resetToDefaultState();
          // Serial.println("Not WAITING ANYMORE");
        }
      }
      break;
  }
}

bool countdown (int countdown, unsigned long currentMillis) {
  static unsigned long lastCountdown = 0;
  if (currentMillis - lastCountdown >= countdown) {
    lastCountdown = currentMillis;
    return true;
  }
  return false;
}

void nonBlockingCountdown(int &countdownCar, int &countdownPedestrian, unsigned long currentMillis) {
  static unsigned long lastUpdateCar = 0;
  if (currentMillis - lastUpdateCar >= interval) {
    lastUpdateCar = currentMillis;
    if (countdownCar > 0 && countdownPedestrian > 0) {
      countdownCar--;
      countdownPedestrian--;
      displayCountdown(countdownCar, countdownPedestrian);
    }
  }
}

void blinkingGreenLightState(unsigned long currentMillis) {
    static unsigned long lastBlinkTime = 0;
    static bool greenState = false;
    if (currentMillis - lastBlinkTime >= interval / 4) {
        greenState = !greenState;
        digitalWrite(greenLedPedestrian, greenState ? HIGH : LOW);
        lastBlinkTime = currentMillis;
    }
}

void displayCountdown(int carCountdown, int pedestrianCountdown) {
  int lineHeight = 32;
  int halfLineHeight = lineHeight/2;
  // Car
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(0, lineHeight*0);
  oled.println("Car Countdown");
  oled.setTextSize(2);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(0, lineHeight*0 + halfLineHeight);
  oled.println(carCountdown);

  // Pedestrian
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(0, lineHeight*1);
  oled.println("Pedestrian Countdown");
  oled.setTextSize(2);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(0, lineHeight*1 + halfLineHeight);
  oled.println(pedestrianCountdown);

  oled.display();
}

const unsigned long ultrasonicInterval = 100;
const int ignoreDistance = 500;
bool isCrosswalkOccupied(unsigned long currentMillis) {
  static unsigned long lastUltrasonicTime = 0;  
  static long lastDistance = 0;
  long duration, distance;
  if (currentMillis - lastUltrasonicTime >= ultrasonicInterval) {
    lastUltrasonicTime = currentMillis;
    // Trigger and read ultrasonic sensor logic
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    // lastDistance = distance;
    if (distance > 100) {
      distance = lastDistance;
    } else {
      lastDistance = distance;
    }
    
    Serial.print(distance);
    Serial.println(" cm");
    
    return (distance < 13 && abs(lastDistance - distance) < 3);
  }
}

// Add this flag to track capture state
bool captureTaken = false;  // New variable to track capture status
const unsigned long debounceDelay = interval; // 1-second debounce delay
void isCarCross(unsigned long currentMillis) {
  int val = digitalRead(IRPin);
  static unsigned long lastCaptureTime = 0; // Store the time of the last capture
  // static bool captureTaken = false;  // New variable to track capture status
  
  // unsigned long currentTime = millis();

  // If a car is detected and debounce period has passed
  if (val == LOW && (currentMillis - lastCaptureTime > debounceDelay) && !captureTaken) {
    lastCaptureTime = currentMillis;
    captureTaken = true;

    // Set values to send
    data.time = localTime();
    data.status = true;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    // localTime(); // Print time
  }

  // Reset the capture flag when the sensor is clear
  if (val == HIGH) {
    captureTaken = false;
  }
}

// Reset state function
void resetToDefaultState() {
  countdownCar = COUNTDOWN_CAR;
  countdownPedestrian = COUNTDOWN_PEDESTRIAN;
  buttonPressed = false;
  myServo.write(servoOpen);  // Raise the barrier
  setCarLight(0);  // Cars green
  setPedestrianLight(2);  // Pedestrians red
  clearScreen();
  state = 0;
  captureTaken = false; 
  // lastCaptureTime = 0;  // Reset the capture time
}

void clearScreen() {
  oled.clearDisplay();
  oled.display();
}

// Set car traffic light
void setCarLight(int colorID) {
  digitalWrite(greenLedCar, colorID == 0 ? HIGH : LOW);
  digitalWrite(yellowLedCar, colorID == 1 ? HIGH : LOW);
  digitalWrite(redLedCar, colorID == 2 ? HIGH : LOW);
}

// Set pedestrian traffic light
void setPedestrianLight(int colorID) {
  digitalWrite(greenLedPedestrian, colorID == 0 ? HIGH : LOW);
  digitalWrite(redLedPedestrian, colorID == 2 ? HIGH : LOW);
}

tm localTime(){
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time. Restarting the board.");
    delay(100);
    ESP.restart();
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  return timeinfo;
}
