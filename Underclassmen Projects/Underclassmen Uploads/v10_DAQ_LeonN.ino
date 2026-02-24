/*
Version: v10
Author: Leon Nguyen
System: DAQ

Description:
- Ported from Arduino Uno to Teensy 4.1 (ALL PINS ARE STRICTLY 3.3V ONLY, NO 5V!!!).
- Upgraded SD logging to use Teensy 4.1 BUILTIN_SDCARD slot.
- Implemented GPS code and moved GPS from SoftwareSerial to Hardware Serial (Serial1).
- Optimized SD Writes: File now opens once on start, and uses .flush() to prevent SD latency and wear.
- Increased I2C bus speed to 400kHz (Fast Mode) for rapid IMU sensor reads, as supported by IMU and RTC.
- Buffered SD writes using snprintf for single-line, high-speed logging -- improving efficiency.
- Replaced custom string parser with TinyGPSPlus for a strong checksum validation and EMI noise immunity.

Sensors Included:
- Hall Effect Sensor (Primary RPM)
- Hall Effect Sensor (Secondary RPM)
- DS3231 RTC (Real Time Clock)
- MPU-6050 (Accelerometer & Gyroscope)
- GPS Module (NMEA 9600 baud) [NEW]
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

// -------- Pins --------
const int hallPrimary   = 2;
const int hallSecondary = 3;
const int ledPin        = 8;
const int buttonPin     = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS   = 50;   // 20Hz Sampling
const uint8_t       PPR         = 1;    // Pulses Per Rev
const unsigned long DEBOUNCE_MS = 50;  
const unsigned long FLUSH_MS    = 1000; // Flush data to SD every 1 second [NEW]

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
char filename[32]; // 16 -> 32 to silence compiler warnings [NEW] 

File logFile; // Global file object to avoid opening/closing it 20 times a second [NEW]

// --- ISRs ---
void isrP() { cntP++; }
void isrS() { cntS++; }

// --- GPS Parsing (TinyGPSPlus Optimized) --- [NEW]
void updateGPS() {
  while (Serial1.available() > 0) { // Using Teensy Hardware Serial1
    gps.encode(Serial1.read()); // Feeds raw serial data to TinyGPSPlus for checksum validation [NEW]
  }
}

// --- Build File Name ---
void buildNewRunFilename() {
  int run = 1;
  do {
    sprintf(filename, "RUN%03d.CSV", run); 
    run++;
  } while (SD.exists(filename));
}

// --- Write Header ---
void writeHeader() {
  if (rtcOk) {
    logFile.println(F("Date,Time,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Lat,Lon"));
  } else {
    logFile.println(F("Time_s,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Lat,Lon"));
  }
  logFile.flush(); // Immediately push header to SD card [NEW]
  Serial.println(F("Header written"));
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast I2C Mode for rapid MPU6050 reads [NEW]
  
  Serial1.begin(9600); // Start Hardware GPS Serial (RX1=Pin 0, TX1=Pin 1) [NEW]

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(hallPrimary, INPUT_PULLUP);
  pinMode(hallSecondary, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(hallPrimary), isrP, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), isrS, FALLING);

  Serial.println(F("Initializing Teensy 4.1 DAQ..."));
  
  // Initialize Teensy 4.1 built-in SD slot [NEW]
  sdOk = SD.begin(BUILTIN_SDCARD);
  Serial.print(F("SD: ")); Serial.println(sdOk ? F("OK") : F("FAIL"));

  rtcOk = rtc.begin();
  Serial.print(F("RTC: ")); Serial.println(rtcOk ? F("OK") : F("FAIL"));
  
  if (rtcOk && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!mpu.begin(0x69)) {
    Serial.println(F("MPU6050 not found at 0x69! Check AD0 pin on IMU."));
    mpuOk = false;
  } else {
    Serial.println(F("MPU6050 OK"));
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);      
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  t0_ms = millis();
  Serial.println(F("Ready. Press Button."));
}

void loop() {
  unsigned long nowMs = millis();
  
  updateGPS();

  // --- Button Logic ---
  bool raw = digitalRead(buttonPin);
  
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      if (lastStable == LOW && raw2 == HIGH) { 
        if (!logging) {
          
          sdOk = SD.begin(BUILTIN_SDCARD);
          if (sdOk) {
            buildNewRunFilename();
            logFile = SD.open(filename, FILE_WRITE); // Open file ONCE at start [NEW]
            
            if (logFile) {
              writeHeader();
              logging = true;
              digitalWrite(ledPin, HIGH);
              Serial.print(F("LOG ON -> ")); Serial.println(filename);
            }
          } else {
            Serial.println(F("SD Error - Insert Card"));
            for(int i=0; i<5; i++) { 
              digitalWrite(ledPin, HIGH); delay(50); 
              digitalWrite(ledPin, LOW); delay(50); 
            }
          }
        } else {
          logging = false;
          digitalWrite(ledPin, LOW);
          if (logFile) logFile.close(); // Securely close file at end of run [NEW]
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

    noInterrupts();
    unsigned long nP = cntP;
    unsigned long nS = cntS;
    cntP = 0; cntS = 0;
    interrupts();

    sensors_event_t a, g, temp;
    if (mpuOk) {
      mpu.getEvent(&a, &g, &temp);
    }

    float rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    float rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;

    if (logFile) {
      char dataLine[150]; // Buffer to hold the entire CSV row [NEW]
      char timeStr[25];
      char imuStr[50];

      // Format Time [NEW]
      if (rtcOk) {
        DateTime dt = rtc.now();
        snprintf(timeStr, sizeof(timeStr), "%04d/%02d/%02d,%02d:%02d:%02d", 
                 dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
      } else {
        snprintf(timeStr, sizeof(timeStr), "%.3f", (nowMs - t0_ms) / 1000.0);
      }

      // Format IMU [NEW]
      if (mpuOk) {
        snprintf(imuStr, sizeof(imuStr), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
                 a.acceleration.x, a.acceleration.y, a.acceleration.z, 
                 g.gyro.x, g.gyro.y, g.gyro.z);
      } else {
        snprintf(imuStr, sizeof(imuStr), "0,0,0,0,0,0");
      }

      // Compile the entire line and write it in one single command [NEW]
      if (gps.location.isValid()) { // [NEW]
         // Extended to 6 decimal places for highly accurate map plotting [NEW]
         snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%s,%.6f,%.6f", 
                  timeStr, rpmP, rpmS, imuStr, gps.location.lat(), gps.location.lng()); // [NEW]
      } else {
         snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%s,0.000000,0.000000", 
                  timeStr, rpmP, rpmS, imuStr);
      }
      
      logFile.println(dataLine);

      // Periodically flush data to SD without closing the file [NEW]
      static unsigned long lastFlush = 0;
      if (nowMs - lastFlush >= FLUSH_MS) {
        lastFlush = nowMs;
        logFile.flush(); 
      }
      
    } else {
      Serial.println(F("File Write Fail"));
      logging = false; // Failsafe
      digitalWrite(ledPin, LOW);
    }
  }
}