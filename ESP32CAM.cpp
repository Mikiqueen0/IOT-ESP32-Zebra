/*********
  Rui Santos
  Complete instructions at: https://RandomNerdTutorials.com/esp32-cam-save-picture-firebase-storage/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  Based on the example provided by the ESP Firebase Client Library
*********/

#include "Arduino.h"
#include "WiFi.h"
#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <LittleFS.h>
#include <FS.h>
#include <SD.h>
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include <addons/TokenHelper.h>
#include <esp_now.h>
#include "time.h"

typedef struct struct_message {
  tm time;
  bool status;
} struct_message;

// Create a struct_message called myData
struct_message data;

// change to your own wifi
const char* ssid = "";
const char* password = "";

// Insert Firebase project API Key
#define API_KEY "AIzaSyCgCPwravCD0z1yA2-zBn8BhD5tGLQ2Egc"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "iotzebraimage@gmail.com"
#define USER_PASSWORD "1212312121"

// Insert Firebase storage bucket ID e.g bucket-name.appspot.com
#define STORAGE_BUCKET_ID "iot-zebra-a7889.firebasestorage.app"
// For example:
//#define STORAGE_BUCKET_ID "esp-iot-app.appspot.com"

// Remove static file names and use dynamic naming
char dynamicLocalFilePath[40];  // Dynamic local file path for photo
char dynamicRemoteFilePath[40]; // Dynamic remote file path for Firebase

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define ONBOARDLED 33

boolean takeNewPhoto = true;

//Define Firebase Data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig configF;

// configF.rtos.timeout = 60000;
// configF.timeout.socketConnection = 30000; // Set to 30 seconds


void fcsUploadCallback(FCS_UploadStatusInfo info);

bool taskCompleted = false;

char timeBuffer[30];

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  // Format the time structure into a string
  strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d_%H-%M-%S", &data.time);
  Serial.print("Time: ");
  Serial.println(timeBuffer);
  
  Serial.print("Status: ");
  Serial.println(data.status ? "true" : "false");
  Serial.println();
}

void capturePhotoSaveLittleFS() {
  // Dispose first pictures because of bad quality
  camera_fb_t* fb = NULL;
  // Skip first 3 frames (increase/decrease number as needed).
  for (int i = 0; i < 4; i++) {
    fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    fb = NULL;
  }
    
  // Take a new photo
  fb = NULL;  
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }  

  // Create dynamic file name using timeBuffer
  snprintf(dynamicLocalFilePath, sizeof(dynamicLocalFilePath), "/%s.jpg", timeBuffer);  // Dynamic local file name

  // Save the photo to LittleFS
  File file = LittleFS.open(dynamicLocalFilePath, FILE_WRITE);
  // File file = LittleFS.open("/photo.jpg", FILE_WRITE);

  // Insert the data in the photo file
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.print("The picture has been saved in ");
    Serial.print(dynamicLocalFilePath);
    // Serial.print("/photo.jpg");
    Serial.print(" - Size: ");
    Serial.print(fb->len);
    Serial.println(" bytes");
  }
  // Close the file
  file.close();
  esp_camera_fb_return(fb);
}


void initWiFi(){
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(ONBOARDLED, HIGH);
    delay(500);
    digitalWrite(ONBOARDLED, LOW);
    WiFi.reconnect();
    attempts++;
    if (attempts > 20) {  // Timeout after 20 seconds
      Serial.println("\nFailed to connect to WiFi. Restarting...");
      ESP.restart();
    }
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(ONBOARDLED, HIGH);
}


void initLittleFS(){
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("LittleFS mounted successfully");
  }
}

void initCamera(){
 // OV2640 camera module
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 6;
    config.fb_count = 1;
  } 
  // else {
  //   config.frame_size = FRAMESIZE_SVGA;
  //   config.jpeg_quality = 12;
  //   config.fb_count = 1;
  // }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  } 
}

void setup() {
  pinMode(ONBOARDLED, OUTPUT);
  // Serial port for debugging purposes
  Serial.begin(115200);
  initWiFi();
  initLittleFS();
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();

  //Firebase
  // Assign the api key
  configF.api_key = API_KEY;
  //Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  //Assign the callback function for the long running token generation task
  configF.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&configF, &auth);
  Firebase.reconnectWiFi(true);

  //Communitate with another esp board
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("Finished setup()");

  formatLittleFS();

  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.println(file.name());
    file = root.openNextFile();
  }

}

void formatLittleFS() {
  Serial.println("Formatting LittleFS...");
  if (LittleFS.format()) {
    Serial.println("LittleFS formatted successfully!");
  } else {
    Serial.println("Failed to format LittleFS.");
  }
}


#define WIFI_SIGNAL_THRESHOLD -90  // Set this value to your preferred RSSI threshold (in dBm)
#define MAX_RETRIES 5  // Set the maximum number of retries for upload attempts

int retryCount = 0;  // Counter for retry attempts

void loop() {
  // // Check Wi-Fi signal strength
  int rssi = WiFi.RSSI();
  // Serial.print("Wi-Fi RSSI: ");
  // Serial.println(rssi);

  if (rssi < WIFI_SIGNAL_THRESHOLD) {  // If the Wi-Fi signal is weak
    Serial.print("Weak Wi-Fi signal, retrying... Wi-Fi RSSI: ");
    Serial.println(rssi);
    delay(2000);  // Wait a moment before retrying (you can adjust this delay as needed)
    return;  // Skip the upload if the signal is weak and retry later
  }

  if (data.status) {
    capturePhotoSaveLittleFS();  // Capture and save photo
    listFiles(); // See all files
    data.status = false;
    delay(1);

    if (Firebase.ready() && !taskCompleted) {
      taskCompleted = true;
      Serial.print("Uploading picture... ");

      // Upload to Firebase Storage with dynamic file paths
      snprintf(dynamicRemoteFilePath, sizeof(dynamicRemoteFilePath), "/data/%s.jpg", timeBuffer);  // Dynamic remote file name

      // Try uploading to Firebase Storage
      bool uploadSuccess = Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID, dynamicLocalFilePath, mem_storage_type_flash, dynamicRemoteFilePath, "image/jpeg", fcsUploadCallback);

      if (uploadSuccess) {
        Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
        LittleFS.remove(dynamicLocalFilePath);  // Delete file after successful upload
        Serial.println("File deleted to free up space.");
        retryCount = 0;  // Reset retry count after successful upload
        taskCompleted = false;
      } else {
        Serial.println(fbdo.errorReason());  // Error message if upload fails
        retryCount++;  // Increment retry count

        if (retryCount >= MAX_RETRIES) {
          Serial.println("Upload failed after maximum retries. Stopping upload attempts.");
          retryCount = 0;  // Reset the retry count for future attempts
          taskCompleted = false;
        } else {
          Serial.println("Retrying upload...");
        }
      }
    }
  }
}

void listFiles() {
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  Serial.println("File lists");
  while (file) {
    Serial.println(file.name());
    file = root.openNextFile();
  }
  Serial.println("---------");
}
// The Firebase Storage upload callback function
void fcsUploadCallback(FCS_UploadStatusInfo info){
    if (info.status == firebase_fcs_upload_status_init){
        Serial.printf("Uploading file %s (%d) to %s\n", info.localFileName.c_str(), info.fileSize, info.remoteFileName.c_str());
    }
    else if (info.status == firebase_fcs_upload_status_upload)
    {
        Serial.printf("Uploaded %d%s, Elapsed time %d ms\n", (int)info.progress, "%", info.elapsedTime);
    }
    else if (info.status == firebase_fcs_upload_status_complete)
    {
        Serial.println("Upload completed\n");
        FileMetaInfo meta = fbdo.metaData();
        Serial.printf("Name: %s\n", meta.name.c_str());
        Serial.printf("Bucket: %s\n", meta.bucket.c_str());
        Serial.printf("contentType: %s\n", meta.contentType.c_str());
        Serial.printf("Size: %d\n", meta.size);
        Serial.printf("Generation: %lu\n", meta.generation);
        Serial.printf("Metageneration: %lu\n", meta.metageneration);
        Serial.printf("ETag: %s\n", meta.etag.c_str());
        Serial.printf("CRC32: %s\n", meta.crc32.c_str());
        Serial.printf("Tokens: %s\n", meta.downloadTokens.c_str());
        Serial.printf("Download URL: %s\n\n", fbdo.downloadURL().c_str());
    }
    else if (info.status == firebase_fcs_upload_status_error){
        Serial.printf("Upload failed, %s\n", info.errorMsg.c_str());
    }
}
