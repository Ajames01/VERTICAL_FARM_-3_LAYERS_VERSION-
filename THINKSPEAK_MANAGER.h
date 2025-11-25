#ifndef THINGSPEAK_MANAGER_H
#define THINGSPEAK_MANAGER_H

#include <HardwareSerial.h>

// ---------------------- State Variables ----------------------
enum TSState {
  TS_POWER_ON,
  TS_CHECK_AT,
  TS_CHECK_SIM,
  TS_WAIT_NETWORK,
  TS_CHECK_GPRS,
  TS_SETUP_BEARER_PROFILE,
  TS_OPEN_BEARER,
  TS_QUERY_BEARER,
  TS_INIT_HTTP,
  TS_SET_HTTP_PARAMS,
  TS_READ_SENSOR,
  TS_SET_URL,
  TS_HTTP_GET,
  TS_WAIT_NEXT,
  TS_ERROR_STATE
};

class ThingSpeakManager {
private:
  HardwareSerial* sim800;
  const char* apiKey;
  const char* apn;
  
  TSState currentState;
  unsigned long stateStartTime;
  int networkAttempts;
  int httpRetries;
  int retryDelay;
  
  // Connection state tracking
  bool moduleInitialized;
  bool gprsConnected;
  bool httpInitialized;
  
  // Sensor data buffers
  static const int SAMPLES_BEFORE_SEND = 3;
  float tempBuffer[SAMPLES_BEFORE_SEND];
  float humBuffer[SAMPLES_BEFORE_SEND];
  int soilMoisture1Buffer[SAMPLES_BEFORE_SEND];
  int soilMoisture2Buffer[SAMPLES_BEFORE_SEND];
  int soilMoisture3Buffer[SAMPLES_BEFORE_SEND];
  bool pumpStateBuffer[SAMPLES_BEFORE_SEND];  // ADDED: Pump state buffer
  int sampleCount;
  
  // Averaged values for ThingSpeak
  float temperature;
  float humidity;
  int soilMoisture1Percent;
  int soilMoisture2Percent;
  int soilMoisture3Percent;
  int soilMoistureAverage;
  int pumpStateValue;  // ADDED: 0 or 1 for ThingSpeak
  
  unsigned long lastUploadTime;
  unsigned long uploadInterval;
  
  // Helper functions
  void changeState(TSState newState);
  void clearBuffer();
  String readResponse(unsigned long timeout);
  bool sendCommandAndWait(const char* cmd, const char* expected, unsigned long timeout);
  
  // State handlers
  void handlePowerOn();
  void handleCheckAT();
  void handleCheckSIM();
  void handleWaitNetwork();
  void handleCheckGPRS();
  void handleSetupBearer();
  void handleOpenBearer();
  void handleQueryBearer();
  void handleInitHTTP();
  void handleSetHTTPParams();
  void handleReadSensor();
  void handleSetURL();
  void handleHTTPGet();
  void handleWaitNext();
  void handleError();

public:
  ThingSpeakManager(HardwareSerial* serial, const char* key, const char* apnName, unsigned long interval = 15000);
  
  void begin(int simPowerPin);
  void update();  // Call this in loop()
  
  // UPDATED: Add sensor data from main program (now includes pump state)
  void addSensorData(float temp, float hum, int soil1, int soil2, int soil3, bool pumpOn);
  
  // Get connection status
  bool isConnected() { return gprsConnected && httpInitialized; }
  bool isModuleInitialized() { return moduleInitialized; }
  
  // Get current state for debugging
  String getStateString();
};

// Constructor
ThingSpeakManager::ThingSpeakManager(HardwareSerial* serial, const char* key, const char* apnName, unsigned long interval) {
  sim800 = serial;
  apiKey = key;
  apn = apnName;
  uploadInterval = interval;
  
  currentState = TS_POWER_ON;
  stateStartTime = 0;
  networkAttempts = 0;
  httpRetries = 0;
  retryDelay = 1000;
  
  moduleInitialized = false;
  gprsConnected = false;
  httpInitialized = false;
  
  sampleCount = 0;
  temperature = 25.0;
  humidity = 50.0;
  soilMoisture1Percent = 50;
  soilMoisture2Percent = 50;
  soilMoisture3Percent = 50;
  soilMoistureAverage = 50;
  pumpStateValue = 0;  // ADDED: Initialize pump state
  
  lastUploadTime = 0;
}

// Initialize
void ThingSpeakManager::begin(int simPowerPin) {
  pinMode(simPowerPin, OUTPUT);
  digitalWrite(simPowerPin, HIGH);
  
  sim800->begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  stateStartTime = millis();
  Serial.println("[ThingSpeak] Manager initialized");
}

// Main update function - call this in loop()
void ThingSpeakManager::update() {
  switch (currentState) {
    case TS_POWER_ON:
      handlePowerOn();
      break;
    case TS_CHECK_AT:
      handleCheckAT();
      break;
    case TS_CHECK_SIM:
      handleCheckSIM();
      break;
    case TS_WAIT_NETWORK:
      handleWaitNetwork();
      break;
    case TS_CHECK_GPRS:
      handleCheckGPRS();
      break;
    case TS_SETUP_BEARER_PROFILE:
      handleSetupBearer();
      break;
    case TS_OPEN_BEARER:
      handleOpenBearer();
      break;
    case TS_QUERY_BEARER:
      handleQueryBearer();
      break;
    case TS_INIT_HTTP:
      handleInitHTTP();
      break;
    case TS_SET_HTTP_PARAMS:
      handleSetHTTPParams();
      break;
    case TS_READ_SENSOR:
      handleReadSensor();
      break;
    case TS_SET_URL:
      handleSetURL();
      break;
    case TS_HTTP_GET:
      handleHTTPGet();
      break;
    case TS_WAIT_NEXT:
      handleWaitNext();
      break;
    case TS_ERROR_STATE:
      handleError();
      break;
  }
}

// UPDATED: Add sensor data from main program (now includes pump state)
void ThingSpeakManager::addSensorData(float temp, float hum, int soil1, int soil2, int soil3, bool pumpOn) {
  if (sampleCount < SAMPLES_BEFORE_SEND) {
    tempBuffer[sampleCount] = temp;
    humBuffer[sampleCount] = hum;
    soilMoisture1Buffer[sampleCount] = soil1;
    soilMoisture2Buffer[sampleCount] = soil2;
    soilMoisture3Buffer[sampleCount] = soil3;
    pumpStateBuffer[sampleCount] = pumpOn;  // ADDED: Store pump state
    sampleCount++;
  }
}

// Helper functions
void ThingSpeakManager::changeState(TSState newState) {
  currentState = newState;
  stateStartTime = millis();
  clearBuffer();
}

void ThingSpeakManager::clearBuffer() {
  while (sim800->available()) {
    sim800->read();
  }
}

String ThingSpeakManager::readResponse(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    while (sim800->available()) {
      char c = sim800->read();
      response += c;
      Serial.write(c);
    }
    delay(10);
  }
  
  return response;
}

bool ThingSpeakManager::sendCommandAndWait(const char* cmd, const char* expected, unsigned long timeout) {
  Serial.print("[TS] Sending: ");
  Serial.println(cmd);
  
  clearBuffer();
  sim800->println(cmd);
  
  String response = readResponse(timeout);
  bool success = response.indexOf(expected) >= 0;
  
  if (success) {
    Serial.println("[TS] ✓ SUCCESS");
  } else {
    Serial.print("[TS] ✗ FAILED - Expected: ");
    Serial.println(expected);
  }
  
  return success;
}

String ThingSpeakManager::getStateString() {
  switch (currentState) {
    case TS_POWER_ON: return "PowerOn";
    case TS_CHECK_AT: return "CheckAT";
    case TS_CHECK_SIM: return "CheckSIM";
    case TS_WAIT_NETWORK: return "WaitNet";
    case TS_CHECK_GPRS: return "CheckGPRS";
    case TS_SETUP_BEARER_PROFILE: return "SetupBearer";
    case TS_OPEN_BEARER: return "OpenBearer";
    case TS_QUERY_BEARER: return "QueryBearer";
    case TS_INIT_HTTP: return "InitHTTP";
    case TS_SET_HTTP_PARAMS: return "SetHTTPParams";
    case TS_READ_SENSOR: return "ReadSensor";
    case TS_SET_URL: return "SetURL";
    case TS_HTTP_GET: return "HTTPGet";
    case TS_WAIT_NEXT: return "WaitNext";
    case TS_ERROR_STATE: return "Error";
    default: return "Unknown";
  }
}

// State handlers
void ThingSpeakManager::handlePowerOn() {
  if (moduleInitialized) {
    if (gprsConnected && httpInitialized) {
      changeState(TS_READ_SENSOR);
    } else {
      changeState(TS_WAIT_NETWORK);
    }
    return;
  }
  
  if (millis() - stateStartTime > 3000) {
    Serial.println("[TS] Module powered on");
    moduleInitialized = true;
    changeState(TS_CHECK_AT);
  }
}

void ThingSpeakManager::handleCheckAT() {
  if (sendCommandAndWait("AT", "OK", 2000)) {
    changeState(TS_CHECK_SIM);
  } else {
    delay(2000);
  }
}

void ThingSpeakManager::handleCheckSIM() {
  if (sendCommandAndWait("AT+CPIN?", "READY", 3000)) {
    networkAttempts = 0;
    changeState(TS_WAIT_NETWORK);
  } else {
    changeState(TS_ERROR_STATE);
  }
}

void ThingSpeakManager::handleWaitNetwork() {
  clearBuffer();
  sim800->println("AT+CREG?");
  String response = readResponse(2000);
  
  if (response.indexOf("+CREG: 0,1") >= 0 || response.indexOf("+CREG: 0,5") >= 0) {
    Serial.println("[TS] ✓ Registered on network");
    networkAttempts = 0;
    changeState(TS_CHECK_GPRS);
  } else {
    networkAttempts++;
    if (networkAttempts >= 20) {
      gprsConnected = false;
      changeState(TS_ERROR_STATE);
    } else {
      delay(2000);
    }
  }
}

void ThingSpeakManager::handleCheckGPRS() {
  clearBuffer();
  sim800->println("AT+CGATT?");
  String response = readResponse(3000);
  
  if (response.indexOf("+CGATT: 1") >= 0) {
    Serial.println("[TS] ✓ GPRS attached");
    changeState(TS_SETUP_BEARER_PROFILE);
  } else {
    if (sendCommandAndWait("AT+CGATT=1", "OK", 5000)) {
      delay(2000);
      changeState(TS_SETUP_BEARER_PROFILE);
    } else {
      gprsConnected = false;
      changeState(TS_ERROR_STATE);
    }
  }
}

void ThingSpeakManager::handleSetupBearer() {
  clearBuffer();
  sim800->println("AT+SAPBR=0,1");
  delay(2000);
  clearBuffer();
  
  if (!sendCommandAndWait("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000)) {
    gprsConnected = false;
    changeState(TS_ERROR_STATE);
    return;
  }
  
  delay(100);
  
  char cmd[100];
  snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", apn);
  
  if (!sendCommandAndWait(cmd, "OK", 2000)) {
    gprsConnected = false;
    changeState(TS_ERROR_STATE);
    return;
  }
  
  Serial.println("[TS] ✓ Bearer configured");
  delay(100);
  changeState(TS_OPEN_BEARER);
}

void ThingSpeakManager::handleOpenBearer() {
  clearBuffer();
  sim800->println("AT+SAPBR=1,1");
  String response = readResponse(15000);
  
  if (response.indexOf("OK") >= 0 || response.indexOf("ALREADY") >= 0) {
    Serial.println("[TS] ✓ Bearer opened");
    gprsConnected = true;
    delay(1000);
    changeState(TS_QUERY_BEARER);
  } else {
    gprsConnected = false;
    changeState(TS_ERROR_STATE);
  }
}

void ThingSpeakManager::handleQueryBearer() {
  clearBuffer();
  sim800->println("AT+SAPBR=2,1");
  String response = readResponse(3000);
  
  if (response.indexOf("+SAPBR: 1,1") >= 0) {
    Serial.println("[TS] ✓ Bearer active");
    gprsConnected = true;
    changeState(TS_INIT_HTTP);
  } else {
    gprsConnected = false;
    changeState(TS_ERROR_STATE);
  }
}

void ThingSpeakManager::handleInitHTTP() {
  if (httpInitialized) {
    changeState(TS_READ_SENSOR);
    return;
  }
  
  clearBuffer();
  sim800->println("AT+HTTPTERM");
  delay(1000);
  clearBuffer();
  
  if (sendCommandAndWait("AT+HTTPINIT", "OK", 3000)) {
    httpInitialized = true;
    delay(100);
    changeState(TS_SET_HTTP_PARAMS);
  } else {
    httpRetries++;
    httpInitialized = false;
    if (httpRetries > 3) {
      httpRetries = 0;
      changeState(TS_ERROR_STATE);
    } else {
      delay(2000);
    }
  }
}

void ThingSpeakManager::handleSetHTTPParams() {
  if (!sendCommandAndWait("AT+HTTPPARA=\"CID\",1", "OK", 2000)) {
    httpInitialized = false;
    changeState(TS_ERROR_STATE);
    return;
  }
  
  Serial.println("[TS] ✓ HTTP initialized");
  delay(100);
  
  sampleCount = 0;
  changeState(TS_READ_SENSOR);
}

void ThingSpeakManager::handleReadSensor() {
  // Wait for samples to be collected from main program
  if (sampleCount >= SAMPLES_BEFORE_SEND) {
    // Calculate averages
    temperature = 0;
    humidity = 0;
    int soilMoisture1Sum = 0;
    int soilMoisture2Sum = 0;
    int soilMoisture3Sum = 0;
    
    for (int i = 0; i < SAMPLES_BEFORE_SEND; i++) {
      temperature += tempBuffer[i];
      humidity += humBuffer[i];
      soilMoisture1Sum += soilMoisture1Buffer[i];
      soilMoisture2Sum += soilMoisture2Buffer[i];
      soilMoisture3Sum += soilMoisture3Buffer[i];
    }
    
    temperature /= SAMPLES_BEFORE_SEND;
    humidity /= SAMPLES_BEFORE_SEND;
    soilMoisture1Percent = soilMoisture1Sum / SAMPLES_BEFORE_SEND;
    soilMoisture2Percent = soilMoisture2Sum / SAMPLES_BEFORE_SEND;
    soilMoisture3Percent = soilMoisture3Sum / SAMPLES_BEFORE_SEND;
    soilMoistureAverage = (soilMoisture1Percent + soilMoisture2Percent + soilMoisture3Percent) / 3;
    
    // ADDED: Calculate pump state (if pump was on in any sample, report as 1)
    pumpStateValue = 0;
    for (int i = 0; i < SAMPLES_BEFORE_SEND; i++) {
      if (pumpStateBuffer[i]) {
        pumpStateValue = 1;
        break;
      }
    }
    
    Serial.print("[TS] Averaged data ready: Temp=");
    Serial.print(temperature, 1);
    Serial.print("°C, Hum=");
    Serial.print(humidity, 1);
    Serial.print("%, SoilAvg=");
    Serial.print(soilMoistureAverage);
    Serial.print("%, Pump=");
    Serial.println(pumpStateValue);
    
    sampleCount = 0;
    changeState(TS_SET_URL);
  } else {
    // Stay in this state until samples are collected
    delay(100);
  }
}

void ThingSpeakManager::handleSetURL() {
  // UPDATED: Added field7 for pump state
  char url[350];
  snprintf(url, sizeof(url), 
           "http://api.thingspeak.com/update?api_key=%s&field1=%.2f&field2=%.2f&field3=%d&field4=%d&field5=%d&field6=%d&field7=%d",
           apiKey, temperature, humidity, soilMoisture1Percent, soilMoisture2Percent, soilMoisture3Percent, soilMoistureAverage, pumpStateValue);
  
  Serial.println("[TS] URL:");
  Serial.println(url);
  
  char cmd[400];
  snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  
  clearBuffer();
  sim800->println(cmd);
  String response = readResponse(3000);
  
  if (response.indexOf("OK") >= 0) {
    Serial.println("[TS] ✓ URL set");
    delay(100);
    changeState(TS_HTTP_GET);
  } else {
    httpRetries++;
    
    if (httpRetries > 3) {
      httpInitialized = false;
      httpRetries = 0;
      changeState(TS_INIT_HTTP);
    } else {
      delay(1000);
      changeState(TS_SET_URL);
    }
  }
}

void ThingSpeakManager::handleHTTPGet() {
  clearBuffer();
  sim800->println("AT+HTTPACTION=0");
  
  String response = "";
  unsigned long start = millis();
  bool gotOK = false;
  
  while (millis() - start < 3000) {
    while (sim800->available()) {
      char c = sim800->read();
      response += c;
    }
    if (response.indexOf("OK") >= 0) {
      gotOK = true;
      break;
    }
    delay(10);
  }
  
  if (!gotOK) {
    httpRetries++;
    if (httpRetries > 3) {
      httpInitialized = false;
      httpRetries = 0;
      changeState(TS_INIT_HTTP);
    } else {
      delay(retryDelay);
      changeState(TS_SET_URL);
    }
    return;
  }
  
  response = "";
  start = millis();
  bool gotAction = false;
  
  while (millis() - start < 25000) {
    while (sim800->available()) {
      char c = sim800->read();
      response += c;
    }
    
    if (response.indexOf("+HTTPACTION:") >= 0) {
      gotAction = true;
      break;
    }
    
    delay(100);
  }
  
  if (gotAction) {
    if (response.indexOf("+HTTPACTION: 0,200") >= 0 || 
        response.indexOf("+HTTPACTION:0,200") >= 0) {
      Serial.println("[TS] ✓✓✓ DATA SENT SUCCESSFULLY!");
      httpRetries = 0;
      retryDelay = 1000;
      lastUploadTime = millis();
      changeState(TS_WAIT_NEXT);
      
    } else if (response.indexOf(",601") >= 0) {
      gprsConnected = false;
      httpInitialized = false;
      changeState(TS_ERROR_STATE);
      
    } else if (response.indexOf(",603") >= 0) {
      httpRetries++;
      delay(retryDelay);
      retryDelay = min(retryDelay * 2, 30000);
      changeState(TS_SET_URL);
      
    } else if (response.indexOf(",604") >= 0) {
      httpRetries++;
      delay(3000);
      changeState(TS_SET_URL);
      
    } else {
      httpRetries++;
      if (httpRetries > 5) {
        httpInitialized = false;
        changeState(TS_INIT_HTTP);
      } else {
        delay(retryDelay);
        changeState(TS_SET_URL);
      }
    }
  } else {
    httpRetries++;
    if (httpRetries > 3) {
      httpInitialized = false;
      httpRetries = 0;
      changeState(TS_INIT_HTTP);
    } else {
      delay(retryDelay);
      retryDelay = min(retryDelay * 2, 30000);
      changeState(TS_SET_URL);
    }
  }
}

void ThingSpeakManager::handleWaitNext() {
  // Check if enough time has passed
  if (millis() - lastUploadTime >= uploadInterval) {
    if (gprsConnected && httpInitialized && httpRetries == 0) {
      changeState(TS_READ_SENSOR);
    } else if (httpRetries > 0) {
      httpRetries = 0;
      changeState(TS_WAIT_NETWORK);
    } else {
      changeState(TS_READ_SENSOR);
    }
  }
  // Otherwise stay in this state
}

void ThingSpeakManager::handleError() {
  Serial.println("[TS] !!! ERROR STATE !!!");
  
  if (httpInitialized) {
    clearBuffer();
    sim800->println("AT+HTTPTERM");
    delay(1000);
    clearBuffer();
    httpInitialized = false;
  }
  
  if (gprsConnected) {
    clearBuffer();
    sim800->println("AT+SAPBR=0,1");
    delay(3000);
    clearBuffer();
    gprsConnected = false;
  }
  
  delay(retryDelay);
  retryDelay = min(retryDelay * 2, 60000);
  
  networkAttempts = 0;
  httpRetries = 0;
  sampleCount = 0;
  
  changeState(TS_WAIT_NETWORK);
}

#endif