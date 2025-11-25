/*
 * ESP32 Smart Irrigation System
 * Main Program: LCD Display + Automatic Pump Control
 * 
 * Features:
 * - Real-time sensor display on 20x4 LCD
 * - Automatic pump control based on soil moisture
 * - Independent background ThingSpeak logging
 * - Pump state tracking to ThingSpeak
 * 
 * Hardware Connections:
 * - LCD (I2C): SDA=GPIO21, SCL=GPIO22
 * - Relay: Signal=GPIO26
 * - DHT11: Data=GPIO13
 * - Soil Sensors: GPIO34, GPIO35, GPIO32
 * - SIM800L: RX=GPIO16, TX=GPIO17, PWR=GPIO25
 */

#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include "ThingSpeakManager.h"

// ---------------------- Pin Definitions ----------------------
#define SIM_PWR 25
#define DHTPIN 13
#define DHTTYPE DHT11
#define SOIL_MOISTURE_PIN_1 34
#define SOIL_MOISTURE_PIN_2 35
#define SOIL_MOISTURE_PIN_3 33
#define RELAY_PIN 5

// ---------------------- Soil Moisture Calibration ----------------------
#define SOIL_WET_VALUE 2550
#define SOIL_DRY_VALUE 4095

// ---------------------- Pump Control Thresholds ----------------------
#define PUMP_ON_THRESHOLD 30    // Turn pump ON when moisture < 30%
#define PUMP_OFF_THRESHOLD 70   // Turn pump OFF when moisture > 70%

// ---------------------- Timing Intervals ----------------------
#define SENSOR_READ_INTERVAL 2000   // Read sensors every 2 seconds
#define LCD_UPDATE_INTERVAL 1000    // Update LCD every 1 second
#define THINGSPEAK_INTERVAL 15000   // Upload to ThingSpeak every 15 seconds

// ---------------------- Objects ----------------------
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);  // I2C address 0x27, 20 columns, 4 rows
HardwareSerial sim800Serial(2);
ThingSpeakManager tsManager(&sim800Serial, "7CB22MIXCVZS1ZU2", "web.gprs.mtnnigeria.net", THINGSPEAK_INTERVAL);

// ---------------------- Global Variables ----------------------
// Current sensor readings
float currentTemperature = 25.0;
float currentHumidity = 50.0;
int currentSoilMoisture1 = 50;
int currentSoilMoisture2 = 50;
int currentSoilMoisture3 = 50;
int currentSoilAverage = 50;

// Pump state
bool pumpState = false;

// Timing
unsigned long lastSensorRead = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastThingSpeakSample = 0;

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== Vertical Farm Monitoring System ===");
  Serial.println("Main Program: LCD + Pump Control");
  Serial.println("Background: ThingSpeak Logging\n");
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  VERTICAL FARM");
  lcd.setCursor(0, 1);
  lcd.print("     MONITORING SYSTEM");
  lcd.setCursor(0, 2);
  lcd.print("  Initializing...");
  delay(2000);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Configure soil moisture pins
  pinMode(SOIL_MOISTURE_PIN_1, INPUT);
  pinMode(SOIL_MOISTURE_PIN_2, INPUT);
  pinMode(SOIL_MOISTURE_PIN_3, INPUT);
  
  // Configure relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Start with pump OFF
  
  // Initialize ThingSpeak manager (runs in background)
  tsManager.begin(SIM_PWR);
  
  lcd.setCursor(0, 3);
  lcd.print("Ready!");
  delay(1000);
  
  Serial.println("=== System Ready ===\n");
}

// ---------------------- Main Loop ----------------------
void loop() {
  // Background task: Update ThingSpeak manager
  tsManager.update();
  
  // Read sensors at regular intervals
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readSensors();
    controlPump();
    lastSensorRead = millis();
    
    // Also send sample to ThingSpeak manager (including pump state)
    if (millis() - lastThingSpeakSample >= 5000) {  // Send every 5 seconds
      tsManager.addSensorData(currentTemperature, currentHumidity, 
                              currentSoilMoisture1, currentSoilMoisture2, 
                              currentSoilMoisture3, pumpState);
      lastThingSpeakSample = millis();
    }
  }
  
  // Update LCD at regular intervals
  if (millis() - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastLcdUpdate = millis();
  }
}

// ---------------------- Sensor Functions ----------------------
void readSensors() {
  // Read DHT11 sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (!isnan(h) && !isnan(t)) {
    currentTemperature = t;
    currentHumidity = h;
  }
  
  // Read all three soil moisture sensors
  currentSoilMoisture1 = readSoilMoisture(SOIL_MOISTURE_PIN_1);
  currentSoilMoisture2 = readSoilMoisture(SOIL_MOISTURE_PIN_2);
  currentSoilMoisture3 = readSoilMoisture(SOIL_MOISTURE_PIN_3);
  
  // Calculate average
  currentSoilAverage = (currentSoilMoisture1 + currentSoilMoisture2 + currentSoilMoisture3) / 3;
  
  // Print to Serial for debugging
  Serial.print("Sensors: T=");
  Serial.print(currentTemperature, 1);
  Serial.print("°C, H=");
  Serial.print(currentHumidity, 0);
  Serial.print("%, S1=");
  Serial.print(currentSoilMoisture1);
  Serial.print("%, S2=");
  Serial.print(currentSoilMoisture2);
  Serial.print("%, S3=");
  Serial.print(currentSoilMoisture3);
  Serial.print("%, Avg=");
  Serial.print(currentSoilAverage);
  Serial.print("%, Pump=");
  Serial.println(pumpState ? "ON" : "OFF");
}

int readSoilMoisture(int pin) {
  // Quick reading with 5 samples for fast response
  long sum = 0;
  const int numReadings = 5;
  
  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  
  int avgValue = sum / numReadings;
  
  // Convert to percentage (0% = dry, 100% = wet)
  int moisture = map(avgValue, SOIL_DRY_VALUE, SOIL_WET_VALUE, 0, 100);
  moisture = constrain(moisture, 0, 100);
  
  return moisture;
}

// ---------------------- Pump Control ----------------------
void controlPump() {
  if (!pumpState && currentSoilAverage < PUMP_ON_THRESHOLD) {
    // Turn pump ON
    digitalWrite(RELAY_PIN, HIGH);
    pumpState = true;
    Serial.println("\n>>> PUMP TURNED ON <<<");
    Serial.print("Reason: Soil moisture ");
    Serial.print(currentSoilAverage);
    Serial.print("% < ");
    Serial.print(PUMP_ON_THRESHOLD);
    Serial.println("%\n");
  } 
  else if (pumpState && currentSoilAverage > PUMP_OFF_THRESHOLD) {
    // Turn pump OFF
    digitalWrite(RELAY_PIN, LOW);
    pumpState = false;
    Serial.println("\n>>> PUMP TURNED OFF <<<");
    Serial.print("Reason: Soil moisture ");
    Serial.print(currentSoilAverage);
    Serial.print("% > ");
    Serial.print(PUMP_OFF_THRESHOLD);
    Serial.println("%\n");
  }
}

// ---------------------- LCD Display ----------------------
void updateLCD() {
  lcd.clear();
  
  // Line 1: Temperature and Humidity
  lcd.setCursor(0, 0);
  lcd.print("T:");
  if (currentTemperature < 10) lcd.print(" ");
  lcd.print(currentTemperature, 1);
  lcd.print("C ");
  
  lcd.print("H:");
  if (currentHumidity < 10) lcd.print(" ");
  if (currentHumidity < 100) lcd.print(" ");
  lcd.print(currentHumidity, 0);
  lcd.print("%");
  
  // Line 2: Soil Moisture Sensors 1 & 2
  lcd.setCursor(0, 1);
  lcd.print("S1:");
  if (currentSoilMoisture1 < 10) lcd.print(" ");
  if (currentSoilMoisture1 < 100) lcd.print(" ");
  lcd.print(currentSoilMoisture1);
  lcd.print("% ");
  
  lcd.print("S2:");
  if (currentSoilMoisture2 < 10) lcd.print(" ");
  if (currentSoilMoisture2 < 100) lcd.print(" ");
  lcd.print(currentSoilMoisture2);
  lcd.print("%");
  
  // Line 3: Soil Sensor 3 and Average
  lcd.setCursor(0, 2);
  lcd.print("S3:");
  if (currentSoilMoisture3 < 10) lcd.print(" ");
  if (currentSoilMoisture3 < 100) lcd.print(" ");
  lcd.print(currentSoilMoisture3);
  lcd.print("% ");
  
  lcd.print("Av:");
  if (currentSoilAverage < 10) lcd.print(" ");
  if (currentSoilAverage < 100) lcd.print(" ");
  lcd.print(currentSoilAverage);
  lcd.print("%");
  
  // Line 4: Pump Status and Connection Status
  lcd.setCursor(0, 3);
  lcd.print("Pump:");
  if (pumpState) {
    lcd.print("ON ");
  } else {
    lcd.print("OFF");
  }
  
  // Show ThingSpeak connection status
  lcd.setCursor(10, 3);
  if (tsManager.isConnected()) {
    lcd.print("Cloud:OK");
  } else if (tsManager.isModuleInitialized()) {
    lcd.print("Cloud:..");
  } else {
    lcd.print("Cloud:--");
  }
}