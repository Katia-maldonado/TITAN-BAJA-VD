## Baja SAE – Titan Racing Baja
Titan Racing Baja SAE is a mechanical engineering club for students to design, build and race a custom off-road vehicle for the annual collegiate competition. Designing and building a car requires us to apply engineering problems to the real world. We gain invaluable skills in design, manufacturing, and project management that prepare us for professional careers. At **CSUF**, our Baja SAE team consists of 14 members, and our goal is to compete in the BAJA SAE Competion that is in Washougal, Washington in May 2026. We are divided into four subteams:

- Chassis / Ergonomics 
- Drivetrain  
- Suspension  
- Vehicle Dynamics / Data Acquisition  

---

## Vehicle Dynamics & Data Acquisition
For the **Vehicle Dynamics / Data Acquisition (VD/DAQ)** subteam, this focuses heavily on the electrical, sensor data, and data analysis work on the car.

We currently run **8 sensors** on the Baja car, all connected to an Arduino based DAQ system to record performance data during testing and competition runs. The system logs:

- Vehicle speed  
- Distance traveled  
- Time  
- Engine RPM  
- Secondary (CVT) RPM  
- Gyroscope data (roll, pitch, yaw)  
- 3-axis acceleration  
- Brake pressure bias  

All sensor data is routed through a **Teensy**, allowing each run to be logged directly to an microSD card. The recorded data is then processed and analyzed in **MATLAB** to evaluate vehicle performance, CVT behavior, braking balance, and driver inputs.
