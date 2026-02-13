/*
Version: v10 
Author: Leon Nguyen
System: CVT

Description:
- Live Data on I2C LCD
- Hot-Swappable SD Card Logic (Re-inits on button press)
- Memory Optimized (F-Strings)
- SEPARATED logging logic from measuring logic:
  - Screen updates ALWAYS (Live Data)
  - SD Card saves ONLY when button is pressed
- Pin Change Interrupts for 3rd RPM sensor
*/

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>

// -------- Hardware Objects --------
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4); // Set screen at address 0x27 or 0x3F [NEW]

// -------- Pins --------
const int SD_CS         = 10;
const int hallPrimary   = 2;
const int hallSecondary = 3;
const int hallBrake     = 4;   
const int ledPin        = 8;
const int buttonPin     = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS   = 50;  // 20Hz Logging
const unsigned long SCREEN_MS   = 250; // 4Hz Screen Update to prevent flickering [NEW]
const uint8_t       PPR         = 1;
const unsigned long DEBOUNCE_MS = 50;

// -------- Counters / State --------
volatile unsigned long cntP = 0, cntS = 0, cntB = 0;
bool logging     = false;
bool lastStable  = HIGH; 
bool debouncing  = false;
unsigned long tDebounce = 0;

bool sdOk  = false;
bool rtcOk = false;
unsigned long t0_ms = 0;
char filename[16]; // Reduced size for memory

// Global RPM variables [NEW]
float rpmP = 0;
float rpmS = 0;
float rpmB = 0;

// --- ISRs ---
void isrP() { cntP++; }
void isrS() { cntS++; }

// ISR for Pin 4 (Brake RPM)
ISR(PCINT2_vect) {
  static bool lastBrakeState = HIGH;
  bool state = (PIND & (1 << PIND4));
  if (lastBrakeState == HIGH && state == LOW) {
    cntB++;
  }
  lastBrakeState = state;
}

// --- Helper Functions ---
void buildNewRunFilename() {
  int run = 1;
  do {
    // Optimized string formatting [NEW]
    sprintf(filename, "RUN%03d.CSV", run);
    run++;
  } while (SD.exists(filename));
}

void writeHeader() {
  File localFile = SD.open(filename, FILE_WRITE);
  if (localFile) {
    // F() macro saves RAM by storing strings in Flash memory [NEW]
    if (rtcOk) {
      localFile.println(F("Date,Time,P_RPM,S_RPM,B_RPM"));
    } else {
      localFile.println(F("Time_s,P_RPM,S_RPM,B_RPM"));
    }
    localFile.close();
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize Screen [NEW]
  lcd.init();      
  lcd.backlight(); 
  lcd.setCursor(0,0);
  lcd.print(F("CSUF Baja SAE")); // F() macro
  lcd.setCursor(0,1);
  lcd.print(F("DAQ v10 Init..."));
  delay(1000); 
  lcd.clear();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  
  pinMode(hallPrimary, INPUT_PULLUP);
  pinMode(hallSecondary, INPUT_PULLUP);
  pinMode(hallBrake, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(hallPrimary), isrP, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), isrS, FALLING);
  
  // Enable PCINT for D4
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);

  Serial.println(F("Initializing DAQ..."));

  // Check RTC
  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  t0_ms = millis();
}

void loop() {
  unsigned long nowMs = millis();

  // --- 1. Button Logic (Start/Stop Logging) --- [NEW]
  bool raw = digitalRead(buttonPin);
  
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      
      // BUTTON PRESS EVENT
      if (lastStable == LOW && raw2 == HIGH) { // Released
        if (!logging) {
          // --- START LOGGING (HOT SWAP LOGIC) ---
          
          // Force SD restart EVERY time logging begins
          sdOk = SD.begin(SD_CS); 
          
          if (sdOk) {
            buildNewRunFilename();
            writeHeader();
            logging = true;
            digitalWrite(ledPin, HIGH);
            Serial.print(F("Log Start: ")); Serial.println(filename);
          } else {
            // SD Failed
            Serial.println(F("SD Fail"));
            // Blink LED to indicate error
            for(int i=0; i<3; i++) {
               digitalWrite(ledPin, HIGH); delay(100);
               digitalWrite(ledPin, LOW);  delay(100);
            }
          }
        } else {
          // --- STOP LOGGING ---
          logging = false;
          digitalWrite(ledPin, LOW);
          Serial.println(F("Log Stop"));
        }
      }
      lastStable = raw2;
    }
    debouncing = false;
  }

  // --- 2. Sampling Logic (Runs ALWAYS for live view) ---
  static unsigned long lastSample = 0;
  // Removed "if (logging)" check so this runs always {UPDATED}
  if (nowMs - lastSample >= SAMPLE_MS) {
    lastSample = nowMs;

    // Atomic Read
    noInterrupts();
    unsigned long nP = cntP;
    unsigned long nS = cntS;
    unsigned long nB = cntB;
    cntP = 0; cntS = 0; cntB = 0;
    interrupts();

    // Calculate RPM
    rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;
    rpmB = (nB * (60000.0 / SAMPLE_MS)) / PPR;

    // --- 3. Data Logging (Only if active) ---
     // SD write is now isolated inside this check [UPDATED]
    if (logging) {
       File localFile = SD.open(filename, FILE_WRITE);
       if (localFile) {
          if (rtcOk) {
            DateTime dt = rtc.now();
            char buf[20];
            sprintf(buf, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
            localFile.print(dt.year()); localFile.print('/'); 
            localFile.print(dt.month()); localFile.print('/');
            localFile.print(dt.day()); localFile.print(',');
            localFile.print(buf); localFile.print(',');
          } else {
            localFile.print((millis() - t0_ms) / 1000.0, 3);
            localFile.print(',');
          }
          localFile.print(rpmP, 0); localFile.print(',');
          localFile.print(rpmS, 0); localFile.print(',');
          localFile.println(rpmB, 0);
          localFile.close();
       } else {
         // If write fails, stop logging to avoid data corruption
         logging = false;
         digitalWrite(ledPin, LOW);
         sdOk = false; // Mark SD as failed so screen updates
       }
    }
  }

  // --- 4. Screen Update --- [NEW]
  static unsigned long lastScreen = 0;
  if (nowMs - lastScreen >= SCREEN_MS) {
    lastScreen = nowMs;

    // Line 1: Status
    lcd.setCursor(0, 0);
    if (logging) {
      lcd.print(F("REC  "));
    } else {
      lcd.print(F("IDLE "));
    }
    // Show Filename or SD Status
    lcd.print(sdOk ? F("SD:OK ") : F("NO SD "));

    // Line 2: Primary RPM
    lcd.setCursor(0, 1);
    lcd.print(F("P:")); lcd.print((int)rpmP); lcd.print(F("   "));

    // Line 3: Secondary RPM
    lcd.setCursor(0, 2);
    lcd.print(F("S:")); lcd.print((int)rpmS); lcd.print(F("   "));

    // Line 4: Brake RPM
    lcd.setCursor(0, 3);
    lcd.print(F("B:")); lcd.print((int)rpmB); lcd.print(F("   "));
  }
}