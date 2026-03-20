#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== LCD Configuration =====
LiquidCrystal_I2C lcd(0x27, 20, 4);  // 20x4 LCD with I2C address 0x27

// ===== Pin Definitions =====
#define BUZZER_PIN 2
#define TEMP_SENSOR_PIN 4        // DS18B20 OneWire
#define MOSFET_PIN 5             // PWM control for MOSFET (0-50A)
#define FAN_PIN 3                // PWM control for cooling fan
#define YELLOW_LED_PIN 6         // Warning LED (70°C)
#define RED_LED_PIN 7            // Shutdown LED (90°C)
#define POT_PIN A0               // Potentiometer for current setting
#define CURRENT_SENSE_PIN A1     // Current sensor (LM358)
#define VOLTAGE_SENSE_PIN A2     // Voltage sensor

// ===== OneWire & Temperature Sensor =====
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

// ===== Calibration Constants =====
const float REF_VOLTAGE = 5.0;
const int ADC_RESOLUTION = 1024;
const float SHUNT_RESISTOR = 0.1;          // R20: 0.1 Ohm
const float VOLTAGE_DIVIDER_RATIO = 5.0;   // (R11 100k + R7 25k) / R7 = 5.0
const float MAX_CURRENT = 50.0;             // 0-50A DC load
const float MAX_VOLTAGE = 25.0;             // 25V maximum
const float OVERVOLTAGE_THRESHOLD = 26.0;   // OVP trigger at 26V

// ===== Temperature Thresholds =====
const float TEMP_FAN_START = 45.0;         // Fan starts
const float TEMP_WARNING = 70.0;           // Yellow LED blink threshold
const float TEMP_CRITICAL = 90.0;          // Red LED + Auto-shutdown threshold

// ===== System Variables =====
float measuredVoltage = 0.0;
float measuredCurrent = 0.0;
float measuredTemp = 0.0;
int mosfetPWM = 0;
int fanPWM = 0;
boolean systemShutdown = false;
boolean overvoltageTriggered = false;
boolean blinkState = false;
unsigned long blinkTimer = 0;
unsigned long sensorReadTimer = 0;

// ===== Status Strings =====
const char *statusMessages[] = {"OK      ", "WARNING ", "CRITICAL"};
const char *ledStatus[] = {"OFF", "BLK", "ON "};  // OFF, BLinking, ON
const char *temperatureStatus[] = {"OK  ", "WARM", "CRIT"};  // Temperature status
const char *modeMessages[] = {"NORMAL  ", "FAN_ON  ", "SHUTDOWN"};

// ===== Enum Definitions =====
enum SystemStatus { STATUS_OK, STATUS_WARN, STATUS_CRITICAL };
enum LEDState { LED_OFF, LED_BLINK, LED_ON };
enum SystemMode { MODE_NORMAL, MODE_FAN_ON, MODE_SHUTDOWN };
enum TemperatureStatus { TEMP_OK, TEMP_WARM, TEMP_CRIT };

// ===== State Variables =====
SystemStatus currentStatus = STATUS_OK;
LEDState yellowLEDState = LED_OFF;
LEDState redLEDState = LED_OFF;
SystemMode currentMode = MODE_NORMAL;
TemperatureStatus tempStatus = TEMP_OK;

void setup() {
  // Pin setup
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(CURRENT_SENSE_PIN, INPUT);
  pinMode(VOLTAGE_SENSE_PIN, INPUT);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Display splash screen with credits
  displaySplashScreen();
  delay(3000);
  
  // Display second splash screen - Features
  displaySplashScreenPage2();
  delay(3000);
  
  lcd.clear();

  // Initialize temperature sensors
  sensors.begin();

  // Serial for debugging
  Serial.begin(9600);
  Serial.println("\n=====================================");
  Serial.println("  DC LOAD TEST V2");
  Serial.println("  BY SPEKTRA KOMUNIKASI");
  Serial.println("=====================================");
  Serial.println("Max: 50A @ 25V with Temperature & Fan Control");
  Serial.println("Voltage Range: 0-25V (Protected @ 26V+)");
  Serial.println("Fan Control: Progressive speed 45-90°C");
  Serial.println("Temperature Warning: 70°C");
  Serial.println("Temperature Critical: 90°C");
  Serial.println("Overvoltage Protection: YES (Zener 5V1)");
  Serial.println("=====================================\n");

  // Turn off all outputs initially
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWrite(MOSFET_PIN, 0);
  analogWrite(FAN_PIN, 0);

  delay(1000);
}

void loop() {
  // Read sensors every 500ms
  if (millis() - sensorReadTimer > 500) {
    readAllSensors();
    sensorReadTimer = millis();
  }

  // Update system status based on temperature and voltage
  updateSystemStatus();

  // Detect overvoltage condition
  detectOvervoltage();

  // Update temperature status for display
  updateTemperatureStatus();

  // Update system mode
  updateSystemMode();

  // Control LEDs and buzzer based on status
  handleTemperatureIndicators();

  // Control cooling fan based on temperature
  controlCoolingFan();

  // Update LCD display with all info
  updateLCDDisplay();

  // Control MOSFET based on status
  controlMOSFET();

  // Debug serial output
  debugSerialOutput();

  delay(100);
}

// ===== Read all sensors =====
void readAllSensors() {
  // Read potentiometer (set current/load)
  int potValue = analogRead(POT_PIN);
  mosfetPWM = map(potValue, 0, 1023, 0, 255);

  // Read current sense
  int currentRaw = analogRead(CURRENT_SENSE_PIN);
  float senseVoltage = (currentRaw * REF_VOLTAGE) / ADC_RESOLUTION;
  measuredCurrent = senseVoltage / SHUNT_RESISTOR;
  
  // Limit to max 50A
  if (measuredCurrent > MAX_CURRENT) {
    measuredCurrent = MAX_CURRENT;
  }
  if (measuredCurrent < 0.0) {
    measuredCurrent = 0.0;
  }

  // Read voltage sense with Zener protection
  int voltageRaw = analogRead(VOLTAGE_SENSE_PIN);
  float voltageADC = (voltageRaw * REF_VOLTAGE) / ADC_RESOLUTION;
  
  // Zener 5V1 clamps to ~5.1V, so cap at 5V for safety
  if (voltageADC > 5.0) {
    voltageADC = 5.0;
  }
  
  measuredVoltage = voltageADC * VOLTAGE_DIVIDER_RATIO;
  
  // Safety limit
  if (measuredVoltage > MAX_VOLTAGE + 2.0) {
    measuredVoltage = MAX_VOLTAGE + 2.0;
  }
  if (measuredVoltage < 0.0) {
    measuredVoltage = 0.0;
  }

  // Read temperature from DS18B20
  sensors.requestTemperatures();
  measuredTemp = sensors.getTempCByIndex(0);
}

// ===== Detect Overvoltage =====
void detectOvervoltage() {
  if (measuredVoltage > OVERVOLTAGE_THRESHOLD) {
    overvoltageTriggered = true;
  } else if (measuredVoltage < (OVERVOLTAGE_THRESHOLD - 1.0)) {
    overvoltageTriggered = false;  // Reset when below threshold - 1V hysteresis
  }
}

// ===== Update Temperature Status =====
void updateTemperatureStatus() {
  if (measuredTemp < TEMP_WARNING) {
    tempStatus = TEMP_OK;
  } else if (measuredTemp < TEMP_CRITICAL) {
    tempStatus = TEMP_WARM;
  } else {
    tempStatus = TEMP_CRIT;
  }
}

// ===== Update System Mode =====
void updateSystemMode() {
  if (systemShutdown) {
    currentMode = MODE_SHUTDOWN;
  } else if (fanPWM > 0) {
    currentMode = MODE_FAN_ON;
  } else {
    currentMode = MODE_NORMAL;
  }
}

// ===== Update System Status =====
void updateSystemStatus() {
  if (systemShutdown) {
    currentStatus = STATUS_CRITICAL;
    redLEDState = LED_ON;
    yellowLEDState = LED_OFF;
  } else if (measuredTemp >= TEMP_WARNING && measuredTemp < TEMP_CRITICAL) {
    currentStatus = STATUS_WARN;
    yellowLEDState = LED_BLINK;
    redLEDState = LED_OFF;
  } else if (measuredTemp >= TEMP_CRITICAL) {
    systemShutdown = true;
    currentStatus = STATUS_CRITICAL;
    redLEDState = LED_ON;
    yellowLEDState = LED_OFF;
  } else {
    currentStatus = STATUS_OK;
    yellowLEDState = LED_OFF;
    redLEDState = LED_OFF;
  }
}

// ===== Handle Temperature Indicators (LEDs & Buzzer) =====
void handleTemperatureIndicators() {
  unsigned long blinkInterval = 500; // 500ms blink

  // Yellow LED (Warning)
  if (yellowLEDState == LED_BLINK) {
    if (millis() - blinkTimer > blinkInterval) {
      blinkState = !blinkState;
      blinkTimer = millis();
    }
    digitalWrite(YELLOW_LED_PIN, blinkState ? HIGH : LOW);
  } else if (yellowLEDState == LED_ON) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else {
    digitalWrite(YELLOW_LED_PIN, LOW);
  }

  // Red LED (Critical)
  if (redLEDState == LED_ON) {
    digitalWrite(RED_LED_PIN, HIGH);
  } else if (redLEDState == LED_BLINK) {
    if (millis() - blinkTimer > blinkInterval) {
      blinkState = !blinkState;
      blinkTimer = millis();
    }
    digitalWrite(RED_LED_PIN, blinkState ? HIGH : LOW);
  } else {
    digitalWrite(RED_LED_PIN, LOW);
  }

  // Buzzer (only during critical shutdown)
  if (currentStatus == STATUS_CRITICAL) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ===== Control Cooling Fan =====
void controlCoolingFan() {
  // Fan speed proportional to temperature
  if (measuredTemp < TEMP_FAN_START) {
    fanPWM = 0;  // Fan off
  } else if (measuredTemp < TEMP_WARNING) {
    // Linear increase from 45°C to 70°C (0 to 50% speed)
    fanPWM = map(measuredTemp * 10, TEMP_FAN_START * 10, TEMP_WARNING * 10, 0, 128);
  } else if (measuredTemp < TEMP_CRITICAL) {
    // Increase from 70°C to 90°C (50% to 100% speed)
    fanPWM = map(measuredTemp * 10, TEMP_WARNING * 10, TEMP_CRITICAL * 10, 128, 255);
  } else {
    // Full speed during critical
    fanPWM = 255;
  }

  analogWrite(FAN_PIN, fanPWM);
}

// ===== Control MOSFET =====
void controlMOSFET() {
  if (systemShutdown || overvoltageTriggered) {
    analogWrite(MOSFET_PIN, 0); // Turn off MOSFET on shutdown or overvoltage
  } else {
    analogWrite(MOSFET_PIN, mosfetPWM);
  }
}

// ===== Display Splash Screen Page 1 - Title & Credits =====
void displaySplashScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  DC LOAD TEST V2   ");
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print("BY SPEKTRA KOMUNIKASI");
  lcd.setCursor(0, 3);
  lcd.print("    Loading...      ");
}

// ===== Display Splash Screen Page 2 - Features =====
void displaySplashScreenPage2() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("50A / 25V / Temp OK ");
  lcd.setCursor(0, 1);
  lcd.print("Fan Control Ready   ");
  lcd.setCursor(0, 2);
  lcd.print("OVP: YES - Zener 5V1");
  lcd.setCursor(0, 3);
  lcd.print("System Ready!       ");
}

// ===== Update LCD Display - COMPLETE FUNCTION =====
void updateLCDDisplay() {
  // Calculate power (W = V × I)
  float powerWatts = measuredVoltage * measuredCurrent;
  
  // Calculate fan percentage
  int fanPercent = (fanPWM / 255.0) * 100;
  
  // ===== ROW 1: Voltage, Current, Fan Speed (20 chars) =====
  // Format: "V:12.5V I:15.3A F:50%"
  lcd.setCursor(0, 0);
  
  // Voltage display
  lcd.print("V:");
  if (measuredVoltage < 10) lcd.print(" ");
  lcd.print(measuredVoltage, 1);
  lcd.print("V ");
  
  // Current display
  lcd.print("I:");
  if (measuredCurrent < 10) lcd.print(" ");
  lcd.print(measuredCurrent, 1);
  lcd.print("A ");
  
  // Fan speed display
  lcd.print("F:");
  if (fanPercent < 10) lcd.print(" ");
  if (fanPercent < 100) lcd.print(" ");
  lcd.print(fanPercent);
  lcd.print("%");
  
  // ===== ROW 2: Temperature, Power (20 chars) =====
  // Format: "Temp:65.2C  Pow:192W"
  lcd.setCursor(0, 1);
  
  // Temperature display
  lcd.print("Temp:");
  if (measuredTemp < 10) lcd.print(" ");
  if (measuredTemp < 100) lcd.print(" ");
  lcd.print(measuredTemp, 1);
  lcd.print("C ");
  
  // Power display
  lcd.print("Pow:");
  if (powerWatts < 100) lcd.print("  ");
  if (powerWatts < 1000) lcd.print(" ");
  lcd.print((int)powerWatts);
  lcd.print("W");
  
  // ===== ROW 3: Status, LED Indicators (20 chars) =====
  // Format: "Stat:OK    Y:OFF R:OFF"
  lcd.setCursor(0, 2);
  
  // Status display
  lcd.print("Stat:");
  lcd.print(statusMessages[currentStatus]);  // "OK      " / "WARNING " / "CRITICAL"
  
  // Yellow LED status
  lcd.print(" Y:");
  lcd.print(ledStatus[yellowLEDState]);      // "OFF" / "BLK" / "ON "
  
  // Red LED status
  lcd.print(" R:");
  lcd.print(ledStatus[redLEDState]);         // "OFF" / "BLK" / "ON "
  
  // ===== ROW 4: Temperature Warning, PWM, OVP, Mode (20 chars) =====
  // Format: "T:OK PWM:128 OVP:OFF"
  lcd.setCursor(0, 3);
  
  // Temperature status (OK / WARM / CRIT)
  lcd.print("T:");
  lcd.print(temperatureStatus[tempStatus]);
  
  // PWM value display
  lcd.print(" PWM:");
  if (mosfetPWM < 10) lcd.print("  ");
  if (mosfetPWM < 100) lcd.print(" ");
  lcd.print(mosfetPWM);
  
  // Overvoltage Protection status
  lcd.print(" OVP:");
  if (overvoltageTriggered) {
    lcd.print("ON!");  // Show ON with warning symbol
  } else {
    lcd.print("OFF");
  }
}

// ===== Debug Serial Output =====
void debugSerialOutput() {
  // Print every 2 seconds for debugging
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    
    Serial.print("V:");
    Serial.print(measuredVoltage, 2);
    Serial.print("V ");
    
    Serial.print("I:");
    Serial.print(measuredCurrent, 2);
    Serial.print("A ");
    
    Serial.print("T:");
    Serial.print(measuredTemp, 2);
    Serial.print("C ");
    
    Serial.print("PWM:");
    Serial.print(mosfetPWM);
    Serial.print(" ");
    
    Serial.print("FAN:");
    Serial.print((fanPWM / 255.0) * 100, 0);
    Serial.print("% ");
    
    Serial.print("Stat:");
    Serial.print(statusMessages[currentStatus]);
    Serial.print(" ");
    
    Serial.print("TempSta:");
    Serial.print(temperatureStatus[tempStatus]);
    Serial.print(" ");
    
    Serial.print("OVP:");
    Serial.print(overvoltageTriggered ? "ON!" : "OFF");
    Serial.print(" ");
    
    Serial.print("Mode:");
    Serial.println(modeMessages[currentMode]);
  }
}
