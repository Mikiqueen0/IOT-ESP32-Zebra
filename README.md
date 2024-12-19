
# Traffic Light Control System with ESP32 and Camera Module

## Overview
This project implements a traffic light control system using the **ESP32 microcontroller** and **camera module (ESP32-CAM)**. The system is designed to manage car and pedestrian traffic lights with a countdown mechanism, control a servo motor for a barrier, and detect vehicles and pedestrians using IR and ultrasonic sensors. Additionally, the ESP32-CAM is integrated to capture video or stream images, allowing real-time monitoring of the intersection.

## Components Used
### For Traffic Light System:
- **ESP32 Dev Module**: Main microcontroller for managing the traffic light control.
- **OLED Display (SH1106)**: Displays the countdown for the traffic lights.
- **Servo Motor**: Controls a barrier that opens or closes based on the traffic light status.
- **IR Sensor**: Detects vehicles crossing the pedestrian crossing.
- **Ultrasonic Sensor**: Measures distance to detect pedestrian presence.
- **LEDs**: For controlling the traffic lights (Car and Pedestrian).
- **Buzzer**: Provides sound alerts during specific events.
- **Push Buttons**: Used to simulate pedestrian button presses.

### For Camera System:
- **ESP32-CAM Module**: Provides camera functionality for capturing and streaming images.
- **MicroSD Card**: For storing images or video streams if required.
- **Camera Module**: Integrated into the ESP32-CAM for video capture.

## Libraries Used
- **ESP32Servo**: Controls the servo motor.
- **Wire**: Used for I2C communication with the OLED display.
- **LiquidCrystal_I2C**: (Currently unused in the code, could be removed).
- **esp_now**: For ESP-NOW communication between multiple ESP32 devices.
- **WiFi**: For Wi-Fi communication (used for ESP-NOW and camera streaming).
- **Adafruit_GFX**: Core graphics library for displays.
- **Adafruit_SSD1306 / Adafruit_SH110X**: Display libraries for the OLED screen.
- **ESP32CAM**: Library for handling camera functionality on ESP32-CAM.

## Features
### Traffic Light Control (ESP32):
- **Car and Pedestrian Countdown**: Displays countdown timers for both car and pedestrian traffic lights on the OLED screen.
- **Servo Motor Control**: Automatically opens or closes a barrier when the pedestrian light is green or red.
- **ESP-NOW Communication**: Sends data (time and status) to another ESP32 device when a car crosses the IR sensor.
- **Ultrasonic Sensor**: Detects the presence of pedestrians on the crosswalk.
- **Push Button Simulation**: Allows for a simulated pedestrian button press to change the traffic light sequence.
- **LED Traffic Lights**: Control of car and pedestrian traffic lights using LEDs.
- **Buzzer Alerts**: The buzzer sounds when specific events occur (e.g., car detected).

### Camera Stream (ESP32-CAM):
- **Real-Time Video Stream**: Captures real-time images or video of the intersection using the ESP32-CAM module.
- **JPEG Image Capture**: Capture snapshots when required for event logging or monitoring.
- **SD Card Storage**: Store images or video files on a microSD card connected to the ESP32-CAM.
- **Web Server**: Provides a live stream of the camera feed over Wi-Fi for remote monitoring.

## Pin Configuration

### ESP32 Traffic Control System:
- **Servo Motor Pin**: Pin 19
- **IR Sensor Pin**: Pin 26
- **Ultrasonic Sensor**:
  - **Trigger Pin**: Pin 12
  - **Echo Pin**: Pin 14
- **LED Pins**:
  - **Car Green Light**: Pin 18
  - **Car Yellow Light**: Pin 5
  - **Car Red Light**: Pin 17
  - **Pedestrian Green Light**: Pin 16
  - **Pedestrian Red Light**: Pin 4
- **Push Button Pin**: Pin 15
- **Buzzer Pin**: Pin 25
- **On-Board LED Pin**: Pin 2

### ESP32-CAM (Camera):
- **Camera Pins**: Default ESP32-CAM pins for the camera module (no need to configure manually).
- **SD Card Pins**: Pins for SD card (configured in the code depending on your ESP32-CAM variant).

## Setup and Installation

### Requirements
- **Arduino IDE**: Install the latest version of the Arduino IDE from [here](https://www.arduino.cc/en/software).
- **ESP32 Board**: Install the ESP32 board in the Arduino IDE. To do this:
  1. Open the Arduino IDE and go to **File** -> **Preferences**.
  2. In the "Additional Boards Manager URLs" field, add this URL: `https://dl.espressif.com/dl/package_esp32_index.json`.
  3. Go to **Tools** -> **Board** -> **Boards Manager**, search for "esp32", and install the latest version.

### Install Libraries
Install the required libraries via the Arduino Library Manager:
- **ESP32Servo**: Search for it in **Sketch** -> **Include Library** -> **Manage Libraries**.
- **Adafruit GFX**: Search for it in the Library Manager.
- **Adafruit SH110X**: Search for it in the Library Manager.
- **Wire**: This is included by default with the Arduino IDE.
- **esp_now**: Part of the ESP32 core libraries.
- **ESP32CAM**: For handling the camera module functionality.

### Wiring
#### Traffic Light System Wiring:
- Connect the hardware as per the pin configuration mentioned above.
- Ensure proper connections for the servo, sensors, LEDs, and other peripherals.
  
#### Camera Module Wiring:
- The ESP32-CAM typically uses the default camera pins; however, ensure it is correctly connected to the camera module.
- If you're using an SD card for storing images or videos, ensure the SD card is correctly inserted and connected.

### Upload the Code
1. Open the Arduino IDE and load the provided code into a new sketch.
2. Select the correct ESP32 board from **Tools** -> **Board**.
3. Select the correct port from **Tools** -> **Port**.
4. Click **Upload** to compile and upload the sketch to your ESP32.

### Wi-Fi Setup
- Update the following lines in the code with your Wi-Fi credentials for both the traffic light system and the camera stream:

  ```cpp
  const char* ssid = "<Your WiFi SSID>";
  const char* password = "<Your WiFi Password>";
  ```
  
## Operation

### Traffic Light System (ESP32):

1.  **Button Press**: When the pedestrian button is pressed, the system switches to pedestrian mode, where the pedestrian light turns green, and the car light turns red.
2.  **Countdown**: The system displays countdowns for both car and pedestrian lights on the OLED display. It updates every second.
3.  **Car Detection**: The IR sensor detects when a car crosses the line. It sends this information to another ESP32 via ESP-NOW, along with the current time.
4.  **Ultrasonic Sensor**: The system uses the ultrasonic sensor to detect the presence of pedestrians on the crosswalk. If the crosswalk is occupied, the pedestrian light blinks.
5.  **Servo Motor**: The servo motor raises or lowers a barrier based on the state of the traffic light.

### Camera Stream (ESP32-CAM):

1. **Car Image Capture**: Capture image when IR sensor detected car and upload the image to Firebase cloud storage.

## Troubleshooting

-   **Wi-Fi Connection Issues**: Ensure that the Wi-Fi credentials are correctly set in the code and that the ESP32 is within range of the Wi-Fi network.
-   **Sensor Calibration**: You may need to calibrate the ultrasonic sensor based on your environment to ensure it detects pedestrians correctly.
-   **IR Sensor Not Triggering**: Ensure that the IR sensor is aligned correctly and has a clear path to detect vehicles.
