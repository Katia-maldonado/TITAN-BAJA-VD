# Live Transmission (Telemetry System)

This folder documents the **Live Transmission (Telemetry) system** used by the CSUF Titan Racing Baja SAE team. The purpose of this system is to wirelessly transmit selected vehicle data in real time during testing, allowing engineers to monitor system health and vehicle behavior without removing the SD card after each run.

This system is designed primarily for **testing and tuning**, especially in off-road and forested environments where visibility and access to the vehicle may be limited.

---

## System Overview

The live transmission system is built around **XBee Pro 900HP radios**, which operate in the 900 MHz band and are well-suited for long-range communication in obstructed terrain such as dirt tracks and wooded areas.

The system consists of:
- An **on-vehicle transmitter**, connected to the DAQ system
- A **base station receiver**, used by the team during testing

The live transmission system operates alongside SD card logging and does **not replace onboard data storage**. SD logging remains the primary data source, while telemetry provides immediate feedback.

---

## On-Vehicle Hardware (Transmitter)

The following components are mounted on the vehicle and integrated into the DAQ system:

- **XBee Pro 900HP – DMST**
- XBee Xplorer shield
- Antenna

The transmitter receives data from the Arduino DAQ system and sends selected data packets wirelessly to the base station in real time.

---

## Base Station Hardware (Receiver)

The base station setup is used by the team during testing sessions and includes:

- **XBee Pro 900HP – DMUT**
- Antenna

The base station receives telemetry data and allows engineers to observe vehicle behavior without stopping the run.

---

## Data Being Transmitted

Live transmission is intended to send **key parameters only**, rather than the full high-rate dataset stored on the SD card. Typical transmitted values include:

- Engine RPM  
- Secondary (CVT) RPM  
- Vehicle speed  
- Brake pressure  
- System status indicators  

Limiting the transmitted data helps maintain a stable wireless connection and reduces packet loss.

---

## Purpose of Live Telemetry

The primary goals of the live transmission system are to:

- Monitor vehicle performance in real time  
- Verify sensor operation during testing  
- Detect issues such as signal dropouts or abnormal readings  
- Reduce downtime between test runs  
- Support CVT tuning and drivetrain testing  

This capability allows the team to make quicker, data-informed decisions while testing, instead of relying solely on post-run analysis.

---

## Environmental Considerations

The telemetry system is designed with **off-road conditions** in mind, including:

- Operation in forested terrain  
- Long-range communication with obstructions  
- Resistance to vibration and electrical noise  

Antenna selection and placement play a critical role in system performance and are considered during testing.

---

## To Do List

### On-Vehicle (Transmitter Side)
- [ ] XBee Pro 900HP – DMST setup  
- [ ] XBee Xplorer shield integration  
- [ ] ANT-916-CW-RCS antenna  
- [ ] Power supply integration (from DAQ system)  
- [ ] Arduino → XBee serial communication    

### Base Station (Receiver Side)
- [ ] XBee Pro 900HP – DMUT setup
- [ ] UFL -> SMA Cable
- [ ] LMR 400 Coax Cable
- [ ] YA9-11 antenna 
- [ ] Base station power configuration  
- [ ] Data reception and parsing  
- [ ] Live data display / monitoring  
