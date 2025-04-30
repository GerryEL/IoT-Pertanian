#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFiS3.h>
#include <EEPROM.h>
#include <FspTimer.h>
#include <WDT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ----- Pin Definitions -----
#define SOILPIN A0
#define MQ135PIN A1
#define LDRPIN A2
#define RAINPIN A3
#define DHTPIN 8
#define DHTTYPE DHT11
#define PUMP 2

// ----- Thresholds for Pump Activation-----
#define SOIL_THRESHOLD 200
#define PARTIAL_SOIL_THRESHOLD 400
#define PUMP_TIME_THRESHOLD 180000
#define HEAVY_RAIN_THRESHOLD 800
#define LIGHT_RAIN_THRESHOLD 990

// Thresholds for displaying sensor values
#define SOIL_LOW 200
#define SOIL_MEDIUM 400
#define LIGHT_LOW 750
#define LIGHT_MEDIUM 400
#define RAIN_LOW 990
#define RAIN_MEDIUM 890

// ----- Timing Settings -----
#define LCD_MODE_CHANGE_INTERVAL 5000
#define LCD_READINGS_PER_MODE 5
#define WIFI_RECONNECT_ATTEMPTS 3
#define SERVER_SEND_INTERVAL 300000
#define WATCHDOG_TIMEOUT 8000
#define VALID_YEAR_THRESHOLD 2025
#define ERROR_DISPLAY_TIMEOUT 5000
#define NTP_UPDATE_INTERVAL 86400000
#define HTTP_TIMEOUT 10000

// ----- Time Zone GMT +7 -----
#define GMT_OFFSET_SECONDS 25200

// ----- System States -----
enum SystemState {
  STATE_INIT,
  STATE_READ_SENSORS,
  STATE_EVALUATE_WATERING,
  STATE_WATERING,
  STATE_DISPLAY_DATA,
  STATE_SEND_DATA,
  STATE_ASYNC_SENDING,
  STATE_ERROR
};

// ----- Error Codes -----
enum ErrorCode {
  ERROR_NONE,
  ERROR_DHT,
  ERROR_SOIL,
  ERROR_RAIN,
  ERROR_AIR,
  ERROR_LDR,
  ERROR_PUMP,
  ERROR_WIFI,
  ERROR_SERVER,
  ERROR_TIME
};

// ----- WiFi and Server Settings -----
const char* ssid = "Vivo V23 5G";
const char* password = "2444666668888888";

// HTTP server settings
const char* serverAddress = "yourserver.com";   // Server hostname or IP
const int serverPort = 80;                      // HTTP port (typically 80)
const char* serverPath = "/api/garden/data";    // API endpoint
const char* apiKey = "your_api_key_here";       // API key for authentication

// Create WiFi client for HTTP
WiFiClient httpClient;

// For NTP time synchronization
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", GMT_OFFSET_SECONDS, NTP_UPDATE_INTERVAL);
bool timeInitialized = false;
unsigned long lastNTPUpdateTime = 0;

// ----- Global Variables -----
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// System state and error tracking
SystemState currentState = STATE_INIT;
ErrorCode currentError = ERROR_NONE;
char errorMessage[32] = "";
unsigned long stateStartTime = 0;
unsigned long errorStartTime = 0;

// Flag to track if an error has been displayed
bool errorDisplayed = false;

// LCD display variables
byte displayMode = 0;
byte readingCount = 0;
unsigned long lastLCDUpdateTime = 0;
unsigned long lastLCDModeChangeTime = 0;

// Sensor data structure to organize readings
struct SensorData {
  float temperature;
  float humidity;
  int ldrValue;
  int rainValue;
  int airQuality;
  float airQualityPPM;
  int soilMoisture;
  bool isValid;
  unsigned long timestamp;
} sensorData;

// Server communication tracking
int dataTransmissionErrors = 0;
unsigned long lastServerSendTime = 0;
char serverResponse[128];

// Buffer for formatting strings (avoid String class)
char msgBuffer[150];
char timeStampBuffer[25];

// Global variables for async HTTP transmission
unsigned long asyncSendStartTime = 0;
bool asyncSendInProgress = false;
unsigned long asyncDisplayRefreshTime = 0;
int httpResponseCode = 0;
bool httpRequestSent = false;
#define ASYNC_DISPLAY_REFRESH_INTERVAL 1000

// Function to get status text based on sensor value with inversion parameter
const char* getSensorStatus(int value, int lowThreshold, int mediumThreshold, bool invertLogic = false) {
  if (invertLogic) {
    // For sensors where lower values mean higher intensity (LDR, rain)
    if (value > mediumThreshold) {
      return "LOW";
    } else if (value > lowThreshold) {
      return "MEDIUM";
    } else {
      return "HIGH";
    }
  } else {
    // For sensors where higher values mean higher intensity
    if (value < lowThreshold) {
      return "LOW";
    } else if (value < mediumThreshold) {
      return "MEDIUM";
    } else {
      return "HIGH";
    }
  }
}

float convertToCO2PPM(int analogValue) {
  // Convert analog reading (0-1023) to voltage (0-5V)
  float voltage = analogValue * (5.0 / 1023.0);
  
  // Convert voltage to resistance ratio
  float rs = ((5.0 * 10.0) / voltage) - 10.0; // Assuming 10K load resistor
  
  // Calculate ratio of Rs/R0 where R0 is the resistance in fresh air
  // You need to calibrate R0 - typical value might be around 76.63
  float r0 = 76.63; // This should be calibrated in clean air (400 ppm CO2)
  float ratio = rs / r0;
  
  // Convert to ppm using power regression
  // CO2 PPM = a * (Rs/R0)^b
  // Typical values: a=100, b=-1.53 for CO2
  float ppm = 100.0 * pow(ratio, -1.53);
  
  return ppm;
}

// Function to check and reconnect WiFi if needed
bool checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    snprintf(msgBuffer, sizeof(msgBuffer), "WiFi disconnected. Attempting to reconnect...");
    Serial.println(msgBuffer);
    
    for (int attempt = 0; attempt < WIFI_RECONNECT_ATTEMPTS; attempt++) {
      WiFi.begin(ssid, password);
      unsigned long startTime = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 10000) {
        WDT.refresh();
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("\nReconnected to WiFi!"));
        return true;
      } else {
        snprintf(msgBuffer, sizeof(msgBuffer), "\nAttempt %d failed. Retrying...", attempt + 1);
        Serial.print(msgBuffer);
      }
    }
    
    Serial.println(F("Failed to reconnect to WiFi after multiple attempts."));
    return false;
  }
  return true;
}

// Function to update NTP time
bool updateNTPTime() {
  if (!checkWiFi()) {
    return false;
  }
  
  bool success = timeClient.update();
  if (success) {
    lastNTPUpdateTime = millis();
    timeInitialized = true;
    Serial.println(F("NTP time updated successfully"));
  } else {
    Serial.println(F("Failed to update NTP time"));
  }
  return success;
}

// Function to get current time as formatted string
void getFormattedTime(char* buffer, size_t bufferSize, bool includeSeconds = true) {
  if (!timeInitialized) {
    // If time not initialized, try to update
    if (!updateNTPTime()) {
      snprintf(buffer, bufferSize, "Time not available");
      return;
    }
  }
  
  // Check if we need to refresh NTP time
  if (millis() - lastNTPUpdateTime >= NTP_UPDATE_INTERVAL) {
    updateNTPTime();
  }
  
  unsigned long epochTime = timeClient.getEpochTime();
  time_t rawtime = epochTime;
  struct tm* ti;
  ti = localtime(&rawtime);
  
  if (includeSeconds) {
    snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d", 
            ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday, 
            ti->tm_hour, ti->tm_min, ti->tm_sec);
  } else {
    snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d", 
            ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday, 
            ti->tm_hour, ti->tm_min);
  }
}

// Function to display an error message once
void displayErrorOnce(ErrorCode error) {
  if (!errorDisplayed) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ERROR:"));
    lcd.setCursor(0, 1);
    
    switch (error) {
      case ERROR_DHT:
        lcd.print(F("Temp & Humidity"));
        break;
      case ERROR_SOIL:
        lcd.print(F("Soil Moisture"));
        break;
      case ERROR_RAIN:
        lcd.print(F("Rain"));
        break;
      case ERROR_AIR:
        lcd.print(F("Air Quality"));
        break;
      case ERROR_LDR:
        lcd.print(F("LDR"));
        break;
      case ERROR_PUMP:
        lcd.print(F("Pump Error"));
        break;
      case ERROR_WIFI:
        lcd.print(F("WiFi"));
        break;
      case ERROR_SERVER:
        lcd.print(F("Server"));
        break;
      case ERROR_TIME:
        lcd.print(F("Time Sync Failed"));
        break;
      default:
        lcd.print(F("Unknown"));
        break;
    }
    
    delay(2000);
    errorDisplayed = true;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Enable watchdog timer using the Renesas API
  WDT.begin(WATCHDOG_TIMEOUT);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Starting"));
  
  dht.begin();

  // Set pin modes
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, LOW);
  
  // Initialize analog sensors
  pinMode(SOILPIN, INPUT);
  pinMode(MQ135PIN, INPUT);
  pinMode(LDRPIN, INPUT);
  pinMode(RAINPIN, INPUT);

  // Reset error displayed flag
  errorDisplayed = false;

  // Connect to WiFi
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Connecting WiFi"));
  Serial.print(F("Connecting to WiFi"));
  WiFi.begin(ssid, password);
  
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < 30000)) {
    WDT.refresh();
    delay(1000);
    Serial.print(".");
    lcd.setCursor(0, 1);
    lcd.print(F("Please wait..."));
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("...Connected!"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("WiFi Connected"));
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    
    // Initialize and update NTP client
    timeClient.begin();
    if (updateNTPTime()) {

      getFormattedTime(timeStampBuffer, sizeof(timeStampBuffer));
      Serial.print(F("Date and Time: "));
      Serial.println(timeStampBuffer);
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Time: ")); 
      lcd.print(timeClient.getFormattedTime());
      
      time_t rawtime = timeClient.getEpochTime();
      struct tm* ti = localtime(&rawtime);
      
      lcd.setCursor(0, 1);
      lcd.print(F("Date: ")); 
      if (ti->tm_mon < 9) lcd.print('0');
      lcd.print(ti->tm_mon + 1);
      lcd.print('/'); 
      if (ti->tm_mday < 10) lcd.print('0');
      lcd.print(ti->tm_mday);
      lcd.print('/');
      lcd.print((ti->tm_year + 1900) % 100);
      delay(3000);
    } else {
      Serial.println(F("Failed to sync time with NTP"));
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Time Sync Failed"));
      lcd.setCursor(0, 1);
      lcd.print(F("Using local time..."));
      delay(2000);
      
      // Set time error and display it once
      currentError = ERROR_TIME;
      displayErrorOnce(currentError);
    }
    
    // Test HTTP connection
    if (httpClient.connect(serverAddress, serverPort)) {
      Serial.println(F("HTTP server connection test successful"));
      httpClient.stop();
    } else {
      Serial.println(F("HTTP server connection test failed"));
      currentError = ERROR_SERVER;
      displayErrorOnce(currentError);
    }
    
    delay(2000);
  } else {
    Serial.println(F(" Failed!"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("WiFi Failed!"));
    lcd.setCursor(0, 1);
    lcd.print(F("Check settings"));
    delay(2000);
    
    currentError = ERROR_WIFI;
    displayErrorOnce(currentError);
  }
  
  lastLCDUpdateTime = millis();
  lastLCDModeChangeTime = millis();
  
  currentState = STATE_READ_SENSORS;
  stateStartTime = millis();
}

bool readSensors() {
  WDT.refresh();
  
  bool success = true;
  
  if (timeInitialized) {
    sensorData.timestamp = timeClient.getEpochTime();
  } else {
    sensorData.timestamp = millis() / 1000;
    
    if (millis() - lastNTPUpdateTime >= NTP_UPDATE_INTERVAL) {
      updateNTPTime();
    }
  }
  
  sensorData.temperature = dht.readTemperature();
  sensorData.humidity = dht.readHumidity();
  if (isnan(sensorData.temperature) || isnan(sensorData.humidity)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    sensorData.temperature = 0.0;
    sensorData.humidity = 0.0;
    currentError = ERROR_DHT;
    displayErrorOnce(currentError);
    success = false;
  }
  
  sensorData.ldrValue = analogRead(LDRPIN);
  if (sensorData.ldrValue < 0 || sensorData.ldrValue > 1023) {
    Serial.println(F("LDR sensor reading out of range!"));
    sensorData.ldrValue = 0;
    currentError = ERROR_LDR;
    displayErrorOnce(currentError);
    success = false;
  }

  sensorData.rainValue = analogRead(RAINPIN);
  if (sensorData.rainValue < 0 || sensorData.rainValue > 1023) {
    Serial.println(F("Rain sensor reading out of range!"));
    sensorData.rainValue = 1023;
    currentError = ERROR_RAIN;
    displayErrorOnce(currentError);
    success = false;
  }
  
  sensorData.airQuality = analogRead(MQ135PIN);
  if (sensorData.airQuality < 0 || sensorData.airQuality > 1023) {
    Serial.println(F("Air quality sensor reading out of range!"));
    sensorData.airQuality = 0;
    currentError = ERROR_AIR;
    displayErrorOnce(currentError);
    success = false;
  }

  sensorData.airQualityPPM = convertToCO2PPM(sensorData.airQuality);
  
  sensorData.soilMoisture = analogRead(SOILPIN);
  if (sensorData.soilMoisture < 0 || sensorData.soilMoisture > 1023) {
    Serial.println(F("Soil moisture sensor reading out of range!"));
    sensorData.soilMoisture = 1023;
    currentError = ERROR_SOIL;
    displayErrorOnce(currentError);
    success = false;
  }
  
  sensorData.isValid = success;
  if (success) {
    currentError = ERROR_NONE;
  }
  
  return success;
}

bool controlWatering() {
  WDT.refresh();
  bool pumpActivated = false;
  
  if (sensorData.soilMoisture < SOIL_THRESHOLD) {
    if (sensorData.rainValue < HEAVY_RAIN_THRESHOLD) {
      digitalWrite(PUMP, LOW);
      Serial.println(F("Heavy rain detected; pump remains off."));
    } else if (sensorData.rainValue < LIGHT_RAIN_THRESHOLD) {
      if (sensorData.soilMoisture < PARTIAL_SOIL_THRESHOLD) {
        Serial.println(F("Light rain detected and soil very dry; starting partial watering."));
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Partial Watering"));
        
        unsigned long startTime = millis();
        while (sensorData.soilMoisture < PARTIAL_SOIL_THRESHOLD && (millis() - startTime < 60000)) {
          WDT.refresh();
          digitalWrite(PUMP, HIGH);
          
          pumpActivated = true;
          lcd.setCursor(0, 1);
          lcd.print(F("Soil: ")); 
          lcd.print(getSensorStatus(sensorData.soilMoisture, SOIL_LOW, SOIL_MEDIUM, false));
          delay(1000);
          sensorData.soilMoisture = analogRead(SOILPIN);
          
          snprintf(msgBuffer, sizeof(msgBuffer), "Soil moisture: %d", sensorData.soilMoisture);
          Serial.println(msgBuffer);
        }
        digitalWrite(PUMP, LOW);
        Serial.println(F("Partial watering complete."));
      } else {
        digitalWrite(PUMP, LOW);
        Serial.println(F("Light rain detected and soil partially watered; pump remains off."));
      }
    } else {
      digitalWrite(PUMP, HIGH);
      pumpActivated = true;
      
      Serial.println(F("No rain and soil is dry; pump activated."));
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Watering..."));
      
      unsigned long pumpStartTime = millis();
      while (sensorData.soilMoisture < SOIL_THRESHOLD && (millis() - pumpStartTime < PUMP_TIME_THRESHOLD)) {
        WDT.refresh();
        lcd.setCursor(0, 1);
        lcd.print(F("Soil: ")); 
        lcd.print(getSensorStatus(sensorData.soilMoisture, SOIL_LOW, SOIL_MEDIUM, false));
        delay(1000);
        sensorData.soilMoisture = analogRead(SOILPIN);
        
        snprintf(msgBuffer, sizeof(msgBuffer), "Watering... Soil moisture: %d", sensorData.soilMoisture);
        Serial.println(msgBuffer);
      }
      digitalWrite(PUMP, LOW);
      Serial.println(F("Watering complete or timeout reached."));
    }
  } else {
    digitalWrite(PUMP, LOW);
    Serial.println(F("Soil moisture adequate; pump deactivated."));
  }
  
  return true;
}

void displayData() {
  readSensors();

  getFormattedTime(timeStampBuffer, sizeof(timeStampBuffer));
  Serial.print(F("Timestamp: "));
  Serial.println(timeStampBuffer);
  
  snprintf(msgBuffer, sizeof(msgBuffer), "Temperature  : %.1f°C", sensorData.temperature);
  Serial.println(msgBuffer);
  
  snprintf(msgBuffer, sizeof(msgBuffer), "Humidity     : %.1f%%", sensorData.humidity);
  Serial.println(msgBuffer);
  
  snprintf(msgBuffer, sizeof(msgBuffer), "Light Level  : %d (%s)", 
          sensorData.ldrValue, getSensorStatus(sensorData.ldrValue, LIGHT_LOW, LIGHT_MEDIUM, true));
  Serial.println(msgBuffer);
  
  snprintf(msgBuffer, sizeof(msgBuffer), "Rain Level   : %d (%s)", 
          sensorData.rainValue, getSensorStatus(sensorData.rainValue, RAIN_LOW, RAIN_MEDIUM, true));
  Serial.println(msgBuffer);
  
  snprintf(msgBuffer, sizeof(msgBuffer), "Air Quality  : %0.1f PPM", 
        sensorData.airQualityPPM);
  Serial.println(msgBuffer);
  
  snprintf(msgBuffer, sizeof(msgBuffer), "Soil Moisture: %d (%s)\n", 
          sensorData.soilMoisture, getSensorStatus(sensorData.soilMoisture, SOIL_LOW, SOIL_MEDIUM, false));
  Serial.println(msgBuffer);

  unsigned long currentTime = millis();
  
  if (currentTime - lastLCDModeChangeTime >= LCD_MODE_CHANGE_INTERVAL) {
    displayMode = (displayMode + 1) % 4;
    lastLCDModeChangeTime = currentTime;
    readingCount = 0;

    lcd.clear();
  }
  
  switch (displayMode) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(F("Temp: ")); 
      lcd.print(sensorData.temperature, 1); 
      lcd.print((char)223); 
      lcd.print(F("C    "));
      
      lcd.setCursor(0, 1);
      
      if (currentState == STATE_ASYNC_SENDING) {
        lcd.print(F("Sending Data... "));
      } else {
        lcd.print(F("Humidity: ")); 
        lcd.print(sensorData.humidity, 1); 
        lcd.print(F("%    "));
      }
      break;
      
    case 1:
      lcd.setCursor(0, 0);
      lcd.print(F("Soil: ")); 
      lcd.print(getSensorStatus(sensorData.soilMoisture, SOIL_LOW, SOIL_MEDIUM, false));
      lcd.print(F("    "));
      
      lcd.setCursor(0, 1);
      
      if (currentState == STATE_ASYNC_SENDING) {
        lcd.print(F("Sending Data... "));
      } else {
        lcd.print(F("Rain: ")); 
        lcd.print(getSensorStatus(sensorData.rainValue, RAIN_LOW, RAIN_MEDIUM, true));
        lcd.print(F("    "));
      }
      break;
      
    case 2:
      lcd.setCursor(0, 0);
      lcd.print(F("Light: ")); 
      lcd.print(getSensorStatus(sensorData.ldrValue, LIGHT_LOW, LIGHT_MEDIUM, true));
      lcd.print(F("    "));
      
      lcd.setCursor(0, 1);
      
      if (currentState == STATE_ASYNC_SENDING) {
        lcd.print(F("Sending Data... "));
      } else {
        lcd.print(F("CO2: ")); 
        lcd.print((int)sensorData.airQualityPPM);
        lcd.print(F(" PPM   "));
      }
      break;
      
    case 3:
      if (timeInitialized) {
        lcd.setCursor(0, 0);
        lcd.print(F("Time: ")); 
        lcd.print(timeClient.getFormattedTime());
        lcd.print(F("    "));
        
        lcd.setCursor(0, 1);
        
        if (currentState == STATE_ASYNC_SENDING) {
          lcd.print(F("Sending Data... "));
        } else {
          time_t rawtime = timeClient.getEpochTime();
          struct tm* ti = localtime(&rawtime);
          
          lcd.print(F("Date: ")); 
          if (ti->tm_mon < 9) lcd.print('0');
          lcd.print(ti->tm_mon + 1);
          lcd.print('/'); 
          if (ti->tm_mday < 10) lcd.print('0');
          lcd.print(ti->tm_mday);
          lcd.print('/');
          lcd.print((ti->tm_year + 1900) % 100);
          lcd.print(F("    "));
        }
      } else {
        lcd.setCursor(0, 0);
        lcd.print(F("Time not sync'd"));
        lcd.setCursor(0, 1);
        lcd.print(F("Check WiFi/NTP"));
      }
      break;
  }
  
  lastLCDUpdateTime = currentTime;
}

bool sendDataToServer() {
  WDT.refresh();
  
  if (!checkWiFi()) {
    currentError = ERROR_WIFI;
    displayErrorOnce(currentError);
    dataTransmissionErrors++;

    if (dataTransmissionErrors >= 3) {
      currentState = STATE_ERROR;
      currentError = ERROR_WIFI;
    } else {
      currentState = STATE_DISPLAY_DATA;
    }
    return false;
  }
  
  // Connect to the server
  if (httpClient.connect(serverAddress, serverPort)) {
    getFormattedTime(timeStampBuffer, sizeof(timeStampBuffer));
    
    // Create JSON payload
    char payload[256];
    snprintf(payload, sizeof(payload), 
            "{\"temp\":%.1f,\"hum\":%.1f,\"light\":%d,\"rain\":%d,\"air\":%d,\"soil\":%d,\"time\":\"%s\"}",
            sensorData.temperature, sensorData.humidity, sensorData.ldrValue, 
            sensorData.rainValue, sensorData.airQualityPPM, sensorData.soilMoisture,
            timeStampBuffer);
    
    // Calculate content length for the header
    int contentLength = strlen(payload);
    
    // Send HTTP POST request
    httpClient.print(F("POST "));
    httpClient.print(serverPath);
    httpClient.println(F(" HTTP/1.1"));
    httpClient.print(F("Host: "));
    httpClient.println(serverAddress);
    httpClient.println(F("Content-Type: application/json"));
    httpClient.print(F("Content-Length: "));
    httpClient.println(contentLength);
    httpClient.print(F("X-API-Key: "));
    httpClient.println(apiKey);
    httpClient.println(F("Connection: close"));
    httpClient.println();
    httpClient.println(payload);
    
    Serial.print(F("Sending HTTP POST: "));
    Serial.println(payload);
    
    asyncSendStartTime = millis();
    asyncSendInProgress = true;
    httpRequestSent = true;
    asyncDisplayRefreshTime = asyncSendStartTime;
    
    currentState = STATE_ASYNC_SENDING;
    
    return true;
  } else {
    Serial.println(F("HTTP server connection failed"));
    currentError = ERROR_SERVER;
    displayErrorOnce(currentError);
    dataTransmissionErrors++;
    
    if (dataTransmissionErrors >= 3) {
      currentState = STATE_ERROR;
      currentError = ERROR_SERVER;
      return false;
    }
    
    currentState = STATE_DISPLAY_DATA;
    return false;
  }
}

void handleAsyncSending() {
  WDT.refresh();
  
  unsigned long currentTime = millis();
  
  // Periodically refresh the display during async sending
  if (currentTime - asyncDisplayRefreshTime >= ASYNC_DISPLAY_REFRESH_INTERVAL) {
    displayData();
    asyncDisplayRefreshTime = currentTime;
  }
  
  // Check for HTTP response or timeout
  if (httpRequestSent && httpClient.available()) {
    // Read the HTTP status line
    String statusLine = httpClient.readStringUntil('\n');
    Serial.print(F("HTTP Response: "));
    Serial.println(statusLine);
    
    // Extract response code
    if (statusLine.indexOf("HTTP/1.1") >= 0) {
      httpResponseCode = statusLine.substring(9, 12).toInt();
      Serial.print(F("Response code: "));
      Serial.println(httpResponseCode);
    }
    
    // Skip HTTP headers
    while (httpClient.available() && httpClient.findUntil("\r\n\r\n", "")) {
      // Skip headers
    }
    
    // Read response body
    int responseIndex = 0;
    while (httpClient.available() && responseIndex < sizeof(serverResponse) - 1) {
      serverResponse[responseIndex++] = httpClient.read();
    }
    serverResponse[responseIndex] = '\0';
    
    Serial.print(F("Server response: "));
    Serial.println(serverResponse);
    
    httpClient.stop();
    asyncSendInProgress = false;
    httpRequestSent = false;
    
    // Check if response was successful
    if (httpResponseCode >= 200 && httpResponseCode < 300) {
      Serial.println(F("Data transmission successful"));
      dataTransmissionErrors = 0;
      lastServerSendTime = currentTime;
      currentState = STATE_DISPLAY_DATA;
    } else {
      Serial.print(F("Data transmission failed with code: "));
      Serial.println(httpResponseCode);
      dataTransmissionErrors++;
      
      if (dataTransmissionErrors >= 3) {
        currentState = STATE_ERROR;
        currentError = ERROR_SERVER;
      } else {
        currentState = STATE_DISPLAY_DATA;
      }
    }
  } 
  // Check for timeout
  else if (currentTime - asyncSendStartTime >= HTTP_TIMEOUT) {
    Serial.println(F("HTTP request timed out"));
    httpClient.stop();
    asyncSendInProgress = false;
    httpRequestSent = false;
    dataTransmissionErrors++;
    
    if (dataTransmissionErrors >= 3) {
      currentState = STATE_ERROR;
      currentError = ERROR_SERVER;
    } else {
      currentState = STATE_DISPLAY_DATA;
    }
  }
}

void handleError() {
  WDT.refresh();
  
  // Clear the LCD and display error message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("ERROR: "));
  
  switch (currentError) {
    case ERROR_DHT:
      lcd.print(F("DHT Sensor"));
      break;
    case ERROR_SOIL:
      lcd.print(F("Soil Sensor"));
      break;
    case ERROR_RAIN:
      lcd.print(F("Rain Sensor"));
      break;
    case ERROR_AIR:
      lcd.print(F("Air Sensor"));
      break;
    case ERROR_LDR:
      lcd.print(F("Light Sensor"));
      break;
    case ERROR_PUMP:
      lcd.print(F("Pump Failure"));
      break;
    case ERROR_WIFI:
      lcd.print(F("WiFi Connection"));
      break;
    case ERROR_SERVER:
      lcd.print(F("Server Connection"));
      break;
    case ERROR_TIME:
      lcd.print(F("Time Sync"));
      break;
    default:
      lcd.print(F("Unknown"));
      break;
  }
  
  lcd.setCursor(0, 1);
  lcd.print(F("Trying to recover"));
  
  delay(3000);
  
  // Try to recover from errors
  switch (currentError) {
    case ERROR_WIFI:
      // Attempt to reconnect to WiFi
      if (checkWiFi()) {
        currentError = ERROR_NONE;
        currentState = STATE_READ_SENSORS;
        dataTransmissionErrors = 0;
      }
      break;
      
    case ERROR_SERVER:
      // Try to reconnect to server
      if (checkWiFi() && httpClient.connect(serverAddress, serverPort)) {
        httpClient.stop();
        currentError = ERROR_NONE;
        currentState = STATE_READ_SENSORS;
        dataTransmissionErrors = 0;
      }
      break;
      
    case ERROR_TIME:
      // Try to sync time again
      if (updateNTPTime()) {
        currentError = ERROR_NONE;
        currentState = STATE_READ_SENSORS;
      }
      break;
      
    default:
      // For sensor errors, just try to read sensors again
      currentState = STATE_READ_SENSORS;
      break;
  }
  
  // If still in error state, wait before retrying
  if (currentState == STATE_ERROR) {
    delay(5000);
  }
}

void loop() {
  WDT.refresh();
  
  unsigned long currentTime = millis();
  
  switch (currentState) {
    case STATE_INIT:
      // Already handled in setup()
      currentState = STATE_READ_SENSORS;
      break;
      
    case STATE_READ_SENSORS:
      Serial.println(F("\n--- Reading Sensors ---"));
      if (readSensors()) {
        currentState = STATE_EVALUATE_WATERING;
      } else {
        // If sensor reading failed, display error and try again
        errorStartTime = currentTime;
        errorDisplayed = false;
        
        // Continue to display data with the last valid readings
        currentState = STATE_DISPLAY_DATA;
      }
      break;
      
    case STATE_EVALUATE_WATERING:
      Serial.println(F("\n--- Evaluating Watering Needs ---"));
      if (sensorData.soilMoisture < SOIL_THRESHOLD) {
        currentState = STATE_WATERING;
      } else {
        currentState = STATE_DISPLAY_DATA;
      }
      break;
      
    case STATE_WATERING:
      Serial.println(F("\n--- Watering Process ---"));
      if (controlWatering()) {
        currentState = STATE_DISPLAY_DATA; 
      } else {
        currentError = ERROR_PUMP;
        currentState = STATE_ERROR;
      }
      break;
      
    case STATE_DISPLAY_DATA:
      // Update display with sensor data
      displayData();
      
      // Reset error displayed flag
      errorDisplayed = false;
      
      // Check if it's time to send data to server
      if (currentTime - lastServerSendTime >= SERVER_SEND_INTERVAL) {
        currentState = STATE_SEND_DATA;
      } else {
        // Read sensors again after a delay
        delay(1000);
        currentState = STATE_READ_SENSORS;
      }
      break;
      
    case STATE_SEND_DATA:
      Serial.println(F("\n--- Sending Data to Server ---"));
      sendDataToServer();
      // State transition handled in sendDataToServer()
      break;
      
    case STATE_ASYNC_SENDING:
      handleAsyncSending();
      // State transition handled in handleAsyncSending()
      break;
      
    case STATE_ERROR:
      handleError();
      // State transition handled in handleError()
      break;
      
    default:
      // If we somehow end up in an invalid state, reset to reading sensors
      Serial.println(F("Invalid state detected, resetting to sensor reading"));
      currentState = STATE_READ_SENSORS;
      break;
  }
  
  // Check if we need to resync NTP time
  if (timeInitialized && (currentTime - lastNTPUpdateTime >= NTP_UPDATE_INTERVAL)) {
    updateNTPTime();
  }
}