/*
Author: Brody Lister / Leon Nguyen
Version: v9
System: DAQ

Description:
- Implemented MPU-6050 IMU.
- Updated SD Log to include Accel and Gyro columns (x,y,z).
- RTC fixes (including updating time if power was lost).
- Optimized memory (F-Strings) to support additional libraries.
- Updated button logic to enable hot-swapping.
- Removed signs of Leon's vibe coding. -_- - Leon

Sensors Included:
- Hall Effect (Primary & Secondary RPM) [Foundation]
- DS3231 RTC (Real Time Clock) [Foundation]
- MPU-6050 (Accelerometer & Gyroscope) [NEW]
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;


// -------- Pins --------
const int SD_CS        = 10;
const int hallPrimary  = 2;
const int hallSecondary = 3;
const int ledPin       = 8;
const int buttonPin    = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS   = 50;  // 20Hz Sampling
const uint8_t       PPR         = 1;   // Pulses Per Rev
const unsigned long DEBOUNCE_MS = 50;  

// -------- Counters / State --------
volatile unsigned long cntP = 0, cntS = 0;

bool logging     = false;
bool lastStable  = HIGH; 
bool debouncing  = false;
unsigned long tDebounce = 0;

bool sdOk  = false;
bool rtcOk = false;
bool mpuOk = false;

unsigned long t0_ms = 0; 
char filename[16]; 

// --- ISRs ---
void isrP() { cntP++; }
void isrS() { cntS++; }

// --- Build File Name ---
void buildNewRunFilename() {
  int run = 1;
  do {
    // 8.3 format: RUN001.CSV
    sprintf(filename, "RUN%03d.CSV", run); 
    run++;
  } while (SD.exists(filename));
}

// --- Write Header ---
void writeHeader() {
  File localFile = SD.open(filename, FILE_WRITE);
  if (localFile) {

    if (rtcOk) {
      localFile.println(F("Date,Time,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ"));
    } else {
      localFile.println(F("Time_s,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ"));
    }
    
    localFile.close();
    Serial.println(F("Header written"));
  } else {
    Serial.println(F("Header write fail"));
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(hallPrimary, INPUT_PULLUP);
  pinMode(hallSecondary, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(hallPrimary), isrP, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), isrS, FALLING);

  Serial.println(F("Initializing DAQ v9..."));

  // --- SD ---
  sdOk = SD.begin(SD_CS);
  Serial.print(F("SD: ")); Serial.println(sdOk ? F("OK") : F("FAIL"));

  // --- RTC ---
  rtcOk = rtc.begin();
  Serial.print(F("RTC: ")); Serial.println(rtcOk ? F("OK") : F("FAIL"));

  // rtcOk reads false at compile-time. Just check if rtc.lostPower() to adjust clock. [NEW]
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Try to find MPU at address 0x69 first (AD0 = HIGH) to avoid RTC conflict
  if (!mpu.begin(0x69)) {
    Serial.println(F("MPU6050 not found at 0x69! Check AD0 pin (Connect to 3.3V)."));
    mpuOk = false;
  } else {
    Serial.println(F("MPU6050 OK"));
    mpuOk = true;
    
    // Configure settings for Baja vehicle dynamics
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 8G range (Bumps are >2G)
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // Standard rotation speed
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Low pass filter (removes engine vibration)
  }

  t0_ms = millis();
  Serial.println(F("Ready. Press Button."));
}

void loop() {
  unsigned long nowMs = millis();

// --- Button Logic (Toggle) --- [NEW]
  bool raw = digitalRead(buttonPin);
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      
      // BUTTON RELEASED -> TOGGLE LOGGING
      if (lastStable == LOW && raw2 == HIGH) { 
        if (!logging) {
          // --- START LOGGING ---
          
          // Force SD card restart every time, which should allow ability to swap cards without resetting the Arduino.
          sdOk = SD.begin(SD_CS); 
          
          if (sdOk) {
            buildNewRunFilename(); // Scans card and finds next available number
            writeHeader();
            logging = true;
            digitalWrite(ledPin, HIGH);
            Serial.print(F("LOG ON -> ")); Serial.println(filename);
          } else {
            // If card is missing or broken
            Serial.println(F("SD Error - Insert Card"));
            // Blink LED as an indicator
            for(int i=0; i<5; i++) { 
              digitalWrite(ledPin, HIGH); delay(50); 
              digitalWrite(ledPin, LOW); delay(50); 
            }
          }
        } else {
          // --- STOP LOGGING ---
          logging = false;
          digitalWrite(ledPin, LOW);
          Serial.println(F("LOG OFF"));
        }
      }
      lastStable = raw2;
    }
    debouncing = false;
  }

  // --- Sampling Loop ---
  static unsigned long lastSample = 0;
  if (logging && (nowMs - lastSample >= SAMPLE_MS)) {
    lastSample = nowMs;

    // 1. Grab RPM Counts (Foundation)
    noInterrupts();
    unsigned long nP = cntP;
    unsigned long nS = cntS;
    cntP = 0; cntS = 0;
    interrupts();

    // READ SENSOR EVENTS
    sensors_event_t a, g, temp;
    if (mpuOk) {
      mpu.getEvent(&a, &g, &temp);
    }

    // 3. Calculation (Foundation)
    float rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    float rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;

    // 4. Write to SD
    File localFile = SD.open(filename, FILE_WRITE);
    if (localFile) {
      // Time (Foundation)
      if (rtcOk) {
        DateTime dt = rtc.now();
        localFile.print(dt.year()); localFile.print('/');
        localFile.print(dt.month()); localFile.print('/');
        localFile.print(dt.day()); localFile.print(',');
        localFile.print(dt.hour()); localFile.print(':');
        localFile.print(dt.minute()); localFile.print(':');
        localFile.print(dt.second()); localFile.print(',');
      } else {
        localFile.print((millis() - t0_ms) / 1000.0, 3);
        localFile.print(',');
      }

      // RPM (Foundation)
      localFile.print(rpmP, 0); localFile.print(',');
      localFile.print(rpmS, 0); localFile.print(',');

      // WRITE IMU DATA TO SD
      if (mpuOk) {
        // Accelerometer (X, Y, Z)
        localFile.print(a.acceleration.x, 2); localFile.print(',');
        localFile.print(a.acceleration.y, 2); localFile.print(',');
        localFile.print(a.acceleration.z, 2); localFile.print(',');
        // Gyroscope (X, Y, Z)
        localFile.print(g.gyro.x, 2); localFile.print(',');
        localFile.print(g.gyro.y, 2); localFile.print(',');
        localFile.println(g.gyro.z, 2); // println ends the CSV row
      } else {
        // Fill with zeros if sensor failed so CSV columns don't break
        localFile.println(F("0,0,0,0,0,0")); 
      }

      localFile.close();
    } else {
      Serial.println(F("File Open Fail"));
    }
  }
}