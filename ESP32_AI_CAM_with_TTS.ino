/*
  Made by Siddharaj Pande
  Tested with - 
  Arduino IDE version 2.3.5
  AI Thinker ESP32-CAM with OV2640(Default cam)and PSRAM
  MAX98357A Amplifier for TTS
  4ohm 3watt woofer
   Features
- Captures image using ESP32-CAM and store them in SD card
- Sends last image to Gemini AI if button 2 pressed
- Converts Gemini's response into speech using Google TTS
  
  Select Board "AI Thinker ESP32-CAM"
  GPIO 0 must be connected to GND to upload a sketch
  After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

  Connections:
  ESP32-CAM AI-THINKER
  FTDI programmer FT232RL connections:
  3v3 - VCC
  GND - GND
  U0R - TX
  U0T - RX
  on cam connect:
  IO0 - GND2 - while uploading then remove it or it will not work

  MAX98357A I2S Amplifier connections:
  VIN - 3.3V
  GND - GND
  14 - BCLK
  4 - LRC
  2 - DIN/DOUT
  GAIN - GND

  Speaker+ - Amplifier +
  Speaker- - Amplifier -

  Connect button one to IO12 and GND it is to click pictures
  connect button two to I013 and GND it is to analyze the last picture 

  You can see the pictures in the SD card by plugging it to a phone or PC

  Note- The ESP32 will reset after a Gemini analysis that is typed in the code and it is to maintain the stability of the ESP32 cam because it has low RAM and PSRAM
*/

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"          // SD Card ESP32
#include "SD_MMC.h"      // SD Card ESP32
#include "soc/soc.h"      // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>      // read and write from flash memory
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Audio.h"
#include <Bounce2.h>     // Button debouncing library
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <rom/rtc.h>

// WiFi credentials
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";
const char* gemini_api_key = "Your_Gemini_API_key";

// Audio pins
#define I2S_DOUT      2
#define I2S_BCLK      14
#define I2S_LRC       4

// Button pins
#define BUTTON_1      12  // First button for taking picture
#define BUTTON_2      13  // Second button for Gemini analysis

// define the number of bytes you want to access
#define EEPROM_SIZE 2

// Pin definition for CAMERA_MODEL_AI_THINKER
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

// Constants
const int WIFI_TIMEOUT = 20000;   // 20 seconds
const int HTTP_TIMEOUT = 10000;   // 10 seconds
const int MAX_RETRIES = 3;

int pictureNumber = 0;
Audio audio;
String lastImagePath = "";
Bounce2::Button button1 = Bounce2::Button();
Bounce2::Button button2 = Bounce2::Button();

RTC_DATA_ATTR int bootCount = 0;
volatile bool geminiAnalysisInProgress = false;
volatile bool pendingResetAfterTTS = false;

bool connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi");
    return false;
  }
}

void setup() {
  bootCount++;
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);

  // Initialize buttons with debouncing
  button1.attach(BUTTON_1, INPUT_PULLUP);
  button2.attach(BUTTON_2, INPUT_PULLUP);
  button1.interval(25);  // debounce interval in milliseconds
  button2.interval(25);

  // Initialize WiFi
  if (!connectToWiFi()) {
    return;
  }

  // Initialize audio
bool i2sStarted = audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
if (!i2sStarted) {
  return;
}
audio.setVolume(100);



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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;//So far VGA worked the best
  config.jpeg_quality = 15;//So far 15 worked the best
  config.fb_count = 1;//So far 1 worked the best
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  int sdRetries = 0;
while (!SD_MMC.begin() && sdRetries < 3) {
  delay(1000);
  sdRetries++;
}
if (!SD_MMC.begin()) {
  Serial.println("SD Card Mount Failed");
  return;
}


  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }

  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize picture number by scanning SD card for existing files
  pictureNumber = 1; // Start from 1
  while (SD_MMC.exists("/picture" + String(pictureNumber) + ".jpg")) {
    pictureNumber++;
  }
  
  // Update EEPROM with the found number
  EEPROM.write(0, pictureNumber & 0xFF);         // lower byte
  EEPROM.write(1, (pictureNumber >> 8) & 0xFF);  // higher byte
  EEPROM.commit();
  
  Serial.print("Next picture number: ");
  Serial.println(pictureNumber);

  if (bootCount > 1) {
    speakAnswer("I am ready again");
  }
}

bool takePicture() {
  camera_fb_t * fb = NULL;
  int retryCount = 0;
  while (true) {
    fb = esp_camera_fb_get();
    if (fb) break;
    retryCount++;
    Serial.print("Camera capture failed, retrying (attempt ");
    Serial.print(retryCount);
    Serial.println(")...");
    delay(500);
  }
  // Save to SD card
  String path = "/picture" + String(pictureNumber) + ".jpg";
  fs::FS &fs = SD_MMC;
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
    esp_camera_fb_return(fb);
    return false;
  }
  else {
    file.write(fb->buf, fb->len);
    Serial.printf("Saved file to path: %s\n", path.c_str());
    lastImagePath = path;  // Store the path for later use
    
    // Increment picture number for next photo
    pictureNumber++;
    
    // Update EEPROM with new number
    EEPROM.write(0, pictureNumber & 0xFF);         // lower byte
    EEPROM.write(1, (pictureNumber >> 8) & 0xFF);  // higher byte
    EEPROM.commit();
    
    Serial.print("Next picture will be number: ");
    Serial.println(pictureNumber);
  }
  file.close();
  esp_camera_fb_return(fb);
  return true;
}

// Helper: Encode image to base64
String encodeImageToBase64(uint8_t* data, size_t length) {
  static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";
  String base64 = "";
  int i = 0, j = 0;
  uint8_t char_array_3[3], char_array_4[4];
  while (length--) {
    char_array_3[i++] = *(data++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;
      for (i = 0; i < 4; i++) base64 += base64_chars[char_array_4[i]];
      i = 0;
    }
  }
  if (i) {
    for (j = i; j < 3; j++) char_array_3[j] = '\0';
    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    for (j = 0; j < i + 1; j++) base64 += base64_chars[char_array_4[j]];
    while (i++ < 3) base64 += '=';
  }
  return base64;
}

// Debug-enabled analyzeWithGemini with retry logic
bool analyzeWithGemini() {
  geminiAnalysisInProgress = true;
  while (true) {
    Serial.println("[Gemini] Starting analyzeWithGemini...");
    Serial.print("[Gemini] Free heap before capture: ");
    Serial.println(ESP.getFreeHeap());
    camera_fb_t * fb = NULL;
    int camRetry = 0;
    while (true) {
      fb = esp_camera_fb_get();
      if (fb) break;
      camRetry++;
      Serial.print("[Gemini] Camera capture failed, retrying (attempt ");
      Serial.print(camRetry);
      Serial.println(")...");
      delay(500);
    }
    Serial.print("[Gemini] Image size: ");
    Serial.println(fb->len);
    Serial.print("[Gemini] Free heap before base64: ");
    Serial.println(ESP.getFreeHeap());
    String base64Image = encodeImageToBase64(fb->buf, fb->len);
    Serial.print("[Gemini] Free heap after base64: ");
    Serial.println(ESP.getFreeHeap());
    esp_camera_fb_return(fb);
    if (base64Image.length() == 0) {
      Serial.println("[Gemini] Failed to encode image, retrying...");
      delay(500);
      continue;
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[Gemini] WiFi not connected, attempting reconnect...");
      if (!connectToWiFi()) {
        Serial.println("[Gemini] WiFi reconnect failed, retrying...");
        delay(500);
        continue;
      }
    }

    Serial.print("[Gemini] Free heap before JSON build: ");
    Serial.println(ESP.getFreeHeap());
    DynamicJsonDocument doc(8192); // Increased buffer size for safety
    JsonArray contents = doc.createNestedArray("contents");
    JsonObject content = contents.createNestedObject();
    JsonArray parts = content.createNestedArray("parts");
    String geminiPrompt = "Analyze this image and describe what you see in detail. IF you see a math problem, solve it step by step. IF you see a famous monument, tell its history, location, and significance. IF you see a famous person, provide their name, profession, and key achievements. IF you see text in any language other than Hindi, Marathi, or English, translate it to English. Be concise but informative. If you cannot identify something clearly, say so rather than guessing and don't tell anything about the quality of Image good or bad or clear or unclear anything just do your job if you can't identify it say so.";//You can modify the prompt accordingly
    JsonObject textPart = parts.createNestedObject();
    textPart["text"] = geminiPrompt;
    JsonObject imagePart = parts.createNestedObject();
    JsonObject inlineData = imagePart.createNestedObject("inlineData");
    inlineData["mimeType"] = "image/jpeg";
    inlineData["data"] = base64Image;
    JsonObject genConfig = doc.createNestedObject("generationConfig");
    genConfig["maxOutputTokens"] = 400;
    String jsonPayload;
    serializeJson(doc, jsonPayload);
    Serial.print("[Gemini] JSON payload size: ");
    Serial.println(jsonPayload.length());
    Serial.print("[Gemini] Free heap before HTTP POST: ");
    Serial.println(ESP.getFreeHeap());

    HTTPClient http;
    String apiUrl = String("https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent?key=") + gemini_api_key;
    http.begin(apiUrl);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(20000);
    Serial.println("[Gemini] Sending HTTP POST to Gemini...");
    int httpResponseCode = http.POST(jsonPayload);
    Serial.print("[Gemini] HTTP POST sent, response code: ");
    Serial.println(httpResponseCode);
    if (httpResponseCode > 0) {
      String result = http.getString();
      Serial.print("[Gemini] HTTP response body: ");
      Serial.println(result);
      http.end();
      DynamicJsonDocument responseDoc(8192);
      DeserializationError error = deserializeJson(responseDoc, result);
      if (!error && responseDoc["candidates"][0]["content"]["parts"][0]["text"]) {
        String answer = responseDoc["candidates"][0]["content"]["parts"][0]["text"].as<String>();
        Serial.print("[Gemini] Parsed answer: ");
        Serial.println(answer);
        speakAnswer(answer); // Only speak the Gemini answer
        geminiAnalysisInProgress = false;
        pendingResetAfterTTS = true; // Set flag to reset after TTS is done
        return true;
      } else {
        Serial.println("[Gemini] Failed to parse response from Gemini, retrying...");
        delay(500);
        continue;
      }
    } else {
      Serial.println("[Gemini] HTTP request failed, code: " + String(httpResponseCode) + ", retrying...");
      http.end();
      delay(500);
      continue;
    }
  }
}

// Debug-enabled speakAnswer
void speakAnswer(String answer) {
  const int chunkSize = 50;
  int len = answer.length();
  int start = 0;
  int chunkNum = 0;
  while (start < len) {
    String chunk;
    if (start + chunkSize >= len) {
      chunk = answer.substring(start);
      start = len;
    } else {
      int end = start + chunkSize;
      while (end > start && !isEndOfSentence(answer.charAt(end))) {
        end--;
      }
      chunk = answer.substring(start, end);
      start = end + 1;
    }
    chunkNum++;
    Serial.print("[TTS] Speaking chunk ");
    Serial.print(chunkNum);
    Serial.print(": ");
    Serial.println(chunk);

    // Check WiFi before TTS
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[TTS] WiFi not connected, skipping TTS chunk.");
      return;
    }

    // Check heap before TTS
    if (ESP.getFreeHeap() < 20000) {
      Serial.println("[TTS] Low heap, skipping TTS chunk.");
      return;
    }

    audio.connecttospeech(chunk.c_str(), "en-GB");
    while(audio.isRunning()) {
      audio.loop();
    }
    Serial.print("[TTS] Finished chunk ");
    Serial.println(chunkNum);
  }
}

bool isEndOfSentence(char c) {
  return c == ' ' || c == '.' || c == '?' || c == '!' || c == ',';
}

void loop() {
  button1.update();
  button2.update();

  if (geminiAnalysisInProgress && (button1.read() == LOW || button2.read() == LOW)) {
    Serial.println("Button pressed during Gemini analysis, resetting ESP32...");
    delay(100);
    ESP.restart();
  }

  if (button1.pressed()) {
    if (takePicture()) {
      Serial.println("Picture taken successfully");
    }
  }

  if (button2.pressed()) {
    Serial.println("Analyzing image with Gemini");
    if (!analyzeWithGemini()) {
      Serial.println("Gemini analysis failed");
    }
  }

  audio.loop();

  // After TTS is done, reset if pending
  if (pendingResetAfterTTS && !audio.isRunning()) {
    Serial.println("TTS finished, resetting ESP32...");
    delay(100);
    ESP.restart();
  }
}

void audio_info(const char *info) {
  Serial.print("audio_info: ");
  Serial.println(info);
}
