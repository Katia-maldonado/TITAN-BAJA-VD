/*
------------------------------------------------------------
Version: v9
Author: Katia Maldonado
Team: CSUF Baja SAE – Titan Racing
System: CVT / Vehicle Dynamics DAQ

Description:
- Added brake disk RPM measurement using Hall-effect sensor on D4
- Implemented pin-change interrupt (PCINT) for third RPM channel
- Logs primary, secondary, and brake RPM to SD with RTC timestamp

Sensors Included:
- Hall-effect sensor (Primary CVT RPM) – D2
- Hall-effect sensor (Secondary CVT RPM) – D3
- Hall-effect sensor (Brake disk RPM) – D4 (PCINT)
- DS3231 Real-Time Clock (I2C)
- Pushbutton (Log Start/Stop)
- Status LED
------------------------------------------------------------
*/

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

RTC_DS3231 rtc;

// -------- Pins --------
const int SD_CS         = 10;
const int hallPrimary   = 2;
const int hallSecondary = 3;
const int hallBrake     = 4;   // ✅ NEW: Brake Hall Sensor on D4 (PCINT)
const int ledPin        = 8;
const int buttonPin     = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS   = 50;  // sample window in ms
const uint8_t       PPR         = 1;    // pulses per rev (magnets per rev)
const unsigned long DEBOUNCE_MS = 50;   // button debounce

// -------- Counters / State --------
volatile unsigned long cntP = 0, cntS = 0;
volatile unsigned long cntB = 0;        // ✅ NEW: Brake pulse counter

bool logging     = false;
bool lastStable  = HIGH;   // button idle (INPUT_PULLUP)
bool debouncing  = false;
unsigned long tDebounce = 0;

bool sdOk  = false;
bool rtcOk = false;
unsigned long t0_ms = 0;   // millis at power-up (for fallback time)
char filename[32];

// --- ISRs (Hall Sensors on D2/D3) ---
void isrP() { cntP++; }
void isrS() { cntS++; }

// ✅ NEW: Pin Change Interrupt for D4 (Brake Hall)
// D4 is PORTD bit 4 => PCINT20 => vector PCINT2_vect (covers D0–D7)
ISR(PCINT2_vect) {
  static bool lastBrakeState = HIGH;
  bool state = (PIND & (1 << PIND4));   // fast read of D4

  // count falling edges only (HIGH -> LOW), like FALLING mode
  if (lastBrakeState == HIGH && state == LOW) {
    cntB++;
  }
  lastBrakeState = state;
}

// --- Build new file name ---
// Use simple 8.3 format: RUN001.CSV, RUN002.CSV, etc.
void buildNewRunFilename() {
  int run = 1;
  do {
    snprintf(filename, sizeof(filename), "RUN%03d.CSV", run);
    run++;
  } while (SD.exists(filename));
}

// --- Write CSV header row ---
void writeHeader() {
  File localFile = SD.open(filename, FILE_WRITE);
  if (localFile) {
    if (rtcOk) {
      localFile.println("Date,Time,Primary_RPM,Secondary_RPM,Brake_RPM");   // ✅ UPDATED
    } else {
      localFile.println("Time_s,Primary_RPM,Secondary_RPM,Brake_RPM");      // ✅ UPDATED
    }
    localFile.close();
    Serial.println("✅ Header written");
  } else {
    Serial.println("❌ Header write failed (open)");
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
  pinMode(hallBrake, INPUT_PULLUP);   // ✅ NEW

  attachInterrupt(digitalPinToInterrupt(hallPrimary), isrP, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), isrS, FALLING);

  // ✅ NEW: Enable Pin Change Interrupt for D4
  // Enable PCINT group for PORTD (D0–D7)
  PCICR  |= (1 << PCIE2);
  // Enable PCINT20 specifically (D4)
  PCMSK2 |= (1 << PCINT20);

  Serial.println("Initializing DAQ...");

  // --- SD card ---
  sdOk = SD.begin(SD_CS);
  Serial.print("SD: ");
  Serial.println(sdOk ? "OK" : "FAIL");

  // --- RTC ---
  rtcOk = rtc.begin();
  Serial.print("RTC: ");
  Serial.println(rtcOk ? "OK" : "FAIL");

  // only resets the RTC time if the battery dies or it loses power
  if (rtcOk && rtc.lostPower()) {
    Serial.println("RTC lostPower -> setting compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  t0_ms = millis();
  Serial.println("DAQ ready. Press button to START/STOP logging.");
}

void loop() {
  unsigned long nowMs = millis();

  // --- Debounced pushbutton toggle (on RELEASE) ---
  bool raw = digitalRead(buttonPin);  // HIGH idle, LOW pressed
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      // toggle on release: LOW -> HIGH
      if (lastStable == LOW && raw2 == HIGH) {
        if (!logging) {
          // Turning logging ON
          if (!sdOk) {
            sdOk = SD.begin(SD_CS);
            Serial.print("Retry SD: ");
            Serial.println(sdOk ? "OK" : "FAIL");
          }
          if (sdOk) {
            buildNewRunFilename();
            writeHeader();
            logging = true;
            digitalWrite(ledPin, HIGH);
            Serial.print("▶ LOG ON -> ");
            Serial.println(filename);
          } else {
            Serial.println("⛔ Cannot start logging (SD not OK).");
          }
        } else {
          // Turning logging OFF
          logging = false;
          digitalWrite(ledPin, LOW);
          Serial.println("■ LOG OFF");
        }
      }
      lastStable = raw2;
    }
    debouncing = false;
  }

  // --- Periodic sample & write ---
  static unsigned long lastSample = 0;
  if (logging && (nowMs - lastSample >= SAMPLE_MS)) {
    lastSample = nowMs;

    // Copy and reset counts safely
    noInterrupts();
    unsigned long nP = cntP;
    unsigned long nS = cntS;
    unsigned long nB = cntB;   // ✅ NEW
    cntP = 0;
    cntS = 0;
    cntB = 0;                  // ✅ NEW
    interrupts();

    // RPM calculations
    float rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    float rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;
    float rpmB = (nB * (60000.0 / SAMPLE_MS)) / PPR;   // ✅ NEW

    File localFile = SD.open(filename, FILE_WRITE);
    if (localFile) {
      // --- Time/Date Column(s) ---
      if (rtcOk) {
        DateTime dt = rtc.now();
        // Date: YYYY/MM/DD
        localFile.print(dt.year());
        localFile.print('/');
        if (dt.month() < 10) localFile.print('0');
        localFile.print(dt.month());
        localFile.print('/');
        if (dt.day() < 10) localFile.print('0');
        localFile.print(dt.day());
        localFile.print(',');

        // Time: HH:MM:SS
        if (dt.hour() < 10) localFile.print('0');
        localFile.print(dt.hour());
        localFile.print(':');
        if (dt.minute() < 10) localFile.print('0');
        localFile.print(dt.minute());
        localFile.print(':');
        if (dt.second() < 10) localFile.print('0');
        localFile.print(dt.second());
        localFile.print(',');
      } else {
        // Fallback: time since power-up (seconds)
        float tsec = (millis() - t0_ms) / 1000.0;
        localFile.print(tsec, 3);
        localFile.print(',');
      }

      // --- RPM Data Columns ---
      localFile.print(rpmP, 2);
      localFile.print(',');
      localFile.print(rpmS, 2);
      localFile.print(',');
      localFile.println(rpmB, 2);  // ✅ NEW

      localFile.close();
      Serial.println("💾 write ok");
    } else {
      Serial.println("❌ file open fail");
    }
  }
}
