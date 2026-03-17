## System Overview

The DAQ system is responsible for collecting sensor data during testing and competition runs, logging it to an SD card. The on vehicle DAQ system is built around an **Arduino** paired with an **SD card reader shield**. Sensor inputs are recorded during each run and saved to an SD card for later analysis. This data is used to evaluate vehicle performance, driver behavior, and drivetrain response under real off road conditions.

Power for the DAQ system is supplied by the vehicle’s **12V battery**. A **buck converter** is used to step the voltage down from **12V to 5V** for sensors and components that require regulated 5V power, ensuring stable operation and protecting sensitive electronics.

---

## Sensors & Components

### Core DAQ Hardware
- Arduino  
- SD card reader shield  
- Buck converter (12V → 5V)

### Sensors / Inputs
- Brake Pressure Sensor (BPS)  
- Hall effect sensor – Secondary RPM  
- Spark plug RPM pickup – Primary RPM  
- IMU (gyroscope + accelerometer)  
- IR sensor (temperature monitoring)  
- GPS module (position / speed tracking)  
- Real Time Clock (RTC)  
- Push button (test control / marker)  
- LED indicators (system status)

---

## Data Logging & Output

During a run, sensor signals are routed into the Arduino, time stamped using the RTC, and logged to the SD card. Data is then transferred for processing and analysis in **MATLAB**, where it can be used to evaluate:

- Vehicle speed and distance trends  
- Engine and CVT RPM behavior  
- Brake pressure bias and braking consistency  
- Vehicle dynamics (roll, pitch, yaw, and acceleration)  
- Thermal behavior from IR sensor readings  
- GPS-based track mapping and run comparison  

---

## Live Transmission

In addition to SD logging, the DAQ system supports **live transmission** to allow real time monitoring during testing without removing the SD card.

### On-Vehicle Hardware (Transmitter)
- **XBee Pro 900HP – DMST**
- XBee Xplorer shield
- Antenna

### Base Station Hardware (Receiver)
- **XBee Pro 900HP – DMUT**
- Antenna

Live telemetry is primarily used during testing sessions to monitor system health and vehicle behavior in real time.

---

## To Do List

### Code  
- [x] SD card reader shield (data logging)  
- [x] Real-Time Clock (RTC) integration  
- [x] Brake Pressure Sensor (BPS)  
- [x] Hall effect sensor – Secondary RPM  
- [x] Spark plug RPM pickup – Primary RPM  
- [x] IMU (gyroscope + accelerometer)  
- [x] IR sensor (temperature monitoring)  
- [x] GPS module  
- [x] Push button (test control / marker)  
- [x] LED indicators (system status)  

### Live Transmission (Telemetry)
- [ ] XBee Pro 900HP – DMST (on-vehicle transmitter)  
- [ ] XBee Xplorer shield  
- [ ] Antenna (vehicle side)  
- [ ] XBee Pro 900HP – DMUT (base station receiver)  
- [ ] Antenna (base station)  
