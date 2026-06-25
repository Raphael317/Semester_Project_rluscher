#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Stepper pinout
const int dirPin = 2;
const int stepPin = 3;
const int ms2Pin = 4;
const int ms1Pin = 5;
const int enPin = 6;

const int buttonPin = 10;

const int encA = 11;
const int encB = 12;

// Pump encoder/button pinout
const int pumpButtonPin = A2;
const int pumpEncA = 7;
const int pumpEncB = 8;

// P9813 / RGB LED driver pins
const int pumpDataPin = 9;   // DIN / DATA
const int pumpClockPin = A1; // CIN / CLOCK

LiquidCrystal_I2C lcd(0x27, 16, 2);

// RPM control
float targetRPM = 3.0;

const float rpmStep = 0.1;
const float minRPM = 0.5;
const float maxRPM = 20.0;

// Pump load control
// 0% = completely OFF
// 5% = old 95% output
// 100% = full output
int pumpLoadPercent = 50;

const int pumpStep = 5;
const int minPumpLoad = 0;
const int maxPumpLoad = 100;

const int minRunningPumpLoad = 5;

// Direct P9813 output scaling
// 242 is about 95% of 255.
// Displayed 5-100% maps to output value 242-255.
const int pumpRealMinValue = 242;
const int pumpRealMaxValue = 255;

// Calibration
const float calibrationFactor = 1.10;

// Fixed microstepping
const int microstep = 16;

// Encoder
int lastEncA;
int lastPumpEncA;

// Driver state
bool driverOn = false;
bool pumpOn = false;

// Stepper button debounce
bool lastButtonReading = HIGH;
bool buttonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Pump button debounce
bool lastPumpButtonReading = HIGH;
bool pumpButtonState = HIGH;

unsigned long lastPumpDebounceTime = 0;

// Step timing
unsigned long lastStepTime = 0;
bool stepState = LOW;

// Last value sent to RGB board
int lastSentPumpValue = -1;

void p9813SendBit(bool bitValue) {
  digitalWrite(pumpDataPin, bitValue);
  digitalWrite(pumpClockPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(pumpClockPin, LOW);
  delayMicroseconds(1);
}

void p9813SendByte(byte value) {
  for (int i = 7; i >= 0; i--) {
    p9813SendBit((value >> i) & 0x01);
  }
}

void p9813SetColor(byte red, byte green, byte blue) {

  // Start frame
  p9813SendByte(0x00);
  p9813SendByte(0x00);
  p9813SendByte(0x00);
  p9813SendByte(0x00);

  // Prefix byte for P9813
  byte prefix = 0xC0;

  if ((blue & 0x80) == 0)  prefix |= 0x20;
  if ((blue & 0x40) == 0)  prefix |= 0x10;
  if ((green & 0x80) == 0) prefix |= 0x08;
  if ((green & 0x40) == 0) prefix |= 0x04;
  if ((red & 0x80) == 0)   prefix |= 0x02;
  if ((red & 0x40) == 0)   prefix |= 0x01;

  p9813SendByte(prefix);
  p9813SendByte(blue);
  p9813SendByte(green);
  p9813SendByte(red);

  // End frame
  p9813SendByte(0x00);
  p9813SendByte(0x00);
  p9813SendByte(0x00);
  p9813SendByte(0x00);

  delayMicroseconds(500);
}

void sendPumpValue(int value) {

  if (value < 0) value = 0;
  if (value > 255) value = 255;

  if (value != lastSentPumpValue) {

    // Pump is connected to R channel
    p9813SetColor(value, 0, 0);

    lastSentPumpValue = value;
  }
}

unsigned long calculateHalfPeriod() {

  float stepsPerRev = 200.0 * microstep;

  float calibratedRPM = targetRPM * calibrationFactor;

  float halfPeriod =
    60000000.0 / (2.0 * calibratedRPM * stepsPerRev);

  return (unsigned long)halfPeriod;
}

void updatePumpOutput() {

  bool pumpShouldRun = pumpOn && pumpLoadPercent >= minRunningPumpLoad;

  // P:0% or pump button OFF = completely OFF
  if (!pumpShouldRun) {
    sendPumpValue(0);
    return;
  }

  // Displayed 5-100% maps to old 95-100% output
  int pumpValue =
    map(pumpLoadPercent,
        minRunningPumpLoad,
        maxPumpLoad,
        pumpRealMinValue,
        pumpRealMaxValue);

  sendPumpValue(pumpValue);
}

void setup() {

  Serial.begin(9600);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(pumpEncA, INPUT_PULLUP);
  pinMode(pumpEncB, INPUT_PULLUP);
  pinMode(pumpButtonPin, INPUT_PULLUP);

  pinMode(pumpDataPin, OUTPUT);
  pinMode(pumpClockPin, OUTPUT);

  digitalWrite(pumpDataPin, LOW);
  digitalWrite(pumpClockPin, LOW);

  digitalWrite(dirPin, HIGH);

  // Fixed 1/16 microstepping
  digitalWrite(ms2Pin, HIGH);
  digitalWrite(ms1Pin, HIGH);

  // Stepper driver OFF at startup
  digitalWrite(enPin, HIGH);

  // Pump OFF at startup
  sendPumpValue(0);

  lastEncA = digitalRead(encA);
  lastPumpEncA = digitalRead(pumpEncA);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("P:OFF M:OFF");

  lcd.setCursor(0, 1);
  lcd.print("P:50% M:3.0RPM");
}

void loop() {

  // Encoder changes RPM
  int currentEncA = digitalRead(encA);

  if (currentEncA != lastEncA && currentEncA == LOW) {

    if (digitalRead(encB) != currentEncA) {
      targetRPM += rpmStep;
    }
    else {
      targetRPM -= rpmStep;
    }

    if (targetRPM < minRPM) targetRPM = minRPM;
    if (targetRPM > maxRPM) targetRPM = maxRPM;
  }

  lastEncA = currentEncA;

  // Pump encoder changes pump load
  int currentPumpEncA = digitalRead(pumpEncA);

  if (currentPumpEncA != lastPumpEncA && currentPumpEncA == LOW) {

    if (digitalRead(pumpEncB) != currentPumpEncA) {
      pumpLoadPercent += pumpStep;
    }
    else {
      pumpLoadPercent -= pumpStep;
    }

    if (pumpLoadPercent < minPumpLoad) pumpLoadPercent = minPumpLoad;
    if (pumpLoadPercent > maxPumpLoad) pumpLoadPercent = maxPumpLoad;
  }

  lastPumpEncA = currentPumpEncA;

  // Button toggles stepper driver ON/OFF
  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != buttonState) {

      buttonState = reading;

      if (buttonState == LOW) {

        driverOn = !driverOn;

        if (driverOn) {
          digitalWrite(enPin, LOW);
        }
        else {
          digitalWrite(enPin, HIGH);
          digitalWrite(stepPin, LOW);
          stepState = LOW;
        }
      }
    }
  }

  lastButtonReading = reading;

  // Pump button toggles pump ON/OFF
  bool pumpReading = digitalRead(pumpButtonPin);

  if (pumpReading != lastPumpButtonReading) {
    lastPumpDebounceTime = millis();
  }

  if ((millis() - lastPumpDebounceTime) > debounceDelay) {

    if (pumpReading != pumpButtonState) {

      pumpButtonState = pumpReading;

      if (pumpButtonState == LOW) {
        pumpOn = !pumpOn;
      }
    }
  }

  lastPumpButtonReading = pumpReading;

  // Update pump output
  updatePumpOutput();

  // LCD update only when values change
  static float lastShownRPM = -1.0;
  static bool lastShownDriverOn = false;
  static int lastShownPumpLoad = -1;
  static bool lastShownPumpRunning = false;

  bool pumpRunning = pumpOn && pumpLoadPercent >= minRunningPumpLoad;

  if (abs(targetRPM - lastShownRPM) >= 0.01 ||
      driverOn != lastShownDriverOn ||
      pumpLoadPercent != lastShownPumpLoad ||
      pumpRunning != lastShownPumpRunning) {

    // First row: pump and motor ON/OFF
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);

    lcd.print("P:");
    if (pumpRunning) {
      lcd.print("ON ");
    }
    else {
      lcd.print("OFF");
    }

    lcd.print(" M:");
    if (driverOn) {
      lcd.print("ON ");
    }
    else {
      lcd.print("OFF");
    }

    // Second row: pump load and motor RPM
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);

    lcd.print("P:");
    lcd.print(pumpLoadPercent);
    lcd.print("%");

    lcd.print(" M:");
    lcd.print(targetRPM, 1);
    lcd.print("RPM");

    lastShownRPM = targetRPM;
    lastShownDriverOn = driverOn;
    lastShownPumpLoad = pumpLoadPercent;
    lastShownPumpRunning = pumpRunning;
  }

  // Non-blocking step generation
  if (driverOn) {

    unsigned long halfPeriod = calculateHalfPeriod();

    if (micros() - lastStepTime >= halfPeriod) {

      lastStepTime = micros();

      stepState = !stepState;
      digitalWrite(stepPin, stepState);
    }
  }
}