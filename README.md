# STM32 One-Axis Brushless Motor Balance Controller

A one-axis self-balancing control system built with an STM32 Nucleo-F411RE, MPU6050 IMU, brushless motor, ESC, and a custom-built wooden test rig.

The firmware was written entirely in C and implements a complete embedded feedback loop: reading IMU data over I2C, estimating arm angle using a complementary filter, computing PID control corrections, and transmitting throttle commands to a brushless ESC using the DShot150 digital protocol with DMA.


## Hardware

| Component | Description |
|-----------|-------------|
| Microcontroller | STM32 Nucleo-F411RE |
| IMU | GY-521 MPU6050 6-DOF Accelerometer/Gyroscope |
| ESC | HGLRC 30A BLHeli_S ESC |
| Motor | AKK RS2205 2300KV Brushless Motor |
| Battery | OVONIC 3S 1300mAh 50C LiPo |
| Test Rig | Custom-built wooden one-axis balancing platform |

<p align="center">
  <img src="images/full_rig.jpg" width="700">
</p>

<p align="center">
<i>Completed one-axis balancing test rig.</i>
</p>

## System Architecture

The controller operates as a closed-loop feedback system. The STM32 continuously samples the IMU, estimates the current arm angle, computes a PID correction, and commands the brushless motor through the ESC using the DShot150 digital protocol.

```text
                    +------------------+
                    |   MPU6050 IMU    |
                    | (Accel + Gyro)   |
                    +--------+---------+
                             |
                         I2C Communication
                             |
                             ▼
                  +----------------------+
                  | STM32 Nucleo-F411RE  |
                  |                      |
                  |  • Gyro Calibration  |
                  |  • Low-Pass Filter   |
                  |  • Complementary     |
                  |    Filter            |
                  |  • PID Controller    |
                  |  • DShot150 Driver   |
                  +----------+-----------+
                             |
                    DMA + Timer PWM
                             |
                             ▼
                  +----------------------+
                  |    BLHeli_S ESC      |
                  +----------+-----------+
                             |
                    Brushless Motor
                             |
                             ▼
                     Generates Thrust
                             |
                             ▼
                  Rotates Balance Arm
                             ▲
                             |
                 MPU6050 Measures Angle
```



## Software Architecture

The firmware is organized into independent modules to separate hardware interfaces from the control algorithm. Each module is responsible for a single subsystem, making the project easier to debug, maintain, and extend.

```text
Core
├── Inc
│   ├── Controller.h
│   ├── DShot.h
│   ├── MPU6050.h
│   └── main.h
│
└── Src
    ├── Controller.c
    ├── DShot.c
    ├── MPU6050.c
    └── main.c
```

### main.c

The main application initializes all peripherals, calibrates the IMU during startup, starts the timer interrupts, and executes the main control loop.

Responsibilities:
- System initialization
- Peripheral configuration
- IMU calibration
- Timer management
- Control loop execution

---

### MPU6050 Driver

Interfaces with the MPU6050 over I2C to acquire accelerometer and gyroscope measurements.

Responsibilities:
- Sensor initialization
- Raw accelerometer and gyroscope acquisition
- Gyroscope bias calibration

---

### Controller

Processes the IMU measurements to estimate the current arm angle and compute the required motor command.

Responsibilities:
- Gyroscope low-pass filtering
- Complementary filter
- Angle estimation
- PID feedback controller
- Throttle calculation

---

### DShot Driver

Generates the DShot150 digital motor control protocol entirely in software using STM32 timers and DMA.

Responsibilities:
- DShot packet generation
- Checksum calculation
- DMA waveform transmission
- ESC communication



## Control Algorithm

The controller operates as a real-time closed-loop feedback system running at **100 Hz**. During every control cycle, the STM32 acquires new IMU measurements, estimates the current arm angle, calculates the required motor correction, and transmits a new DShot150 command to the ESC.

```text
          MPU6050
      (Accel + Gyro)
             │
             ▼
    Gyroscope Calibration
             │
             ▼
     Gyro Low-Pass Filter
             │
             ▼
     Complementary Filter
             │
             ▼
    Current Arm Angle
             │
             ▼
       PID Controller
             │
             ▼
     Throttle Correction
             │
             ▼
      DShot150 Driver
             │
             ▼
       ESC + Motor
             │
             ▼
      Physical Balance Arm
             │
             └───────────────┐
                             │
                MPU6050 Measures Angle
```

### Sensor Processing

Each control cycle begins by reading the accelerometer and gyroscope from the MPU6050 over I2C.

The gyroscope is first calibrated to remove startup bias before a low-pass filter reduces measurement noise. Accelerometer and gyroscope measurements are fused using a complementary filter, providing a fast and stable estimate of the arm angle while minimizing long-term drift.

### Closed-Loop Control

The estimated angle is continuously compared to a target angle to determine the current control error.

A PID controller computes the required throttle correction using:

- **Proportional (P):** Responds to the current angle error.
- **Integral (I):** Compensates for small steady-state errors caused by gravity and changing system conditions.
- **Derivative (D):** Uses filtered gyroscope measurements to damp oscillations and improve stability.

The controller applies different proportional and derivative gains depending on the direction of motion to better match the asymmetric dynamics of the balance arm.

### Motor Command Generation

The final controller output is combined with an experimentally determined base throttle before being converted into a DShot150 packet.

The packet is transmitted using an STM32 timer and DMA, allowing the ESC to receive precisely timed digital throttle commands while the CPU continues executing the control loop.
### Angle Estimation

The MPU6050 provides both accelerometer and gyroscope measurements.

The gyroscope provides smooth short-term angular velocity measurements but accumulates drift over time. The accelerometer provides an absolute estimate of the arm angle but is significantly noisier.

A complementary filter combines both sensors, using the gyroscope for short-term response while continuously correcting long-term drift using the accelerometer.

This approach provides a stable angle estimate suitable for real-time feedback control.

### PID Controller

The estimated arm angle is compared to the desired target angle to calculate the control error.

The controller computes three independent control terms:

- **Proportional (P):** Generates the primary restoring force based on the current angle error.
- **Integral (I):** Removes steady-state error by accumulating small persistent errors over time.
- **Derivative (D):** Uses the filtered gyroscope rate to damp oscillations and improve system stability.

The three terms are combined to produce the final throttle correction applied to the brushless motor.

To improve stability, separate proportional and derivative gains are used depending on whether the arm is moving above or below the target angle, allowing the controller to better compensate for the asymmetric dynamics of the physical system.



## DShot150 Digital Motor Control

Instead of using traditional PWM or OneShot protocols, this project communicates with the ESC using the DShot150 digital motor control protocol.

DShot provides deterministic digital communication between the microcontroller and the ESC, eliminating calibration requirements while improving reliability and timing accuracy.

### Packet Generation

Each throttle command is encoded into a 16-bit DShot packet containing:

- 11-bit throttle value
- 1 telemetry request bit
- 4-bit checksum

The checksum is calculated using the three packet nibbles before transmission, allowing the ESC to detect corrupted packets.

### DMA-Based Transmission

Generating DShot waveforms entirely in software requires precise timing that is difficult to maintain while simultaneously executing the control algorithm.

To solve this, the STM32 timer generates the PWM waveform while DMA continuously updates the timer compare register with the pulse widths corresponding to each DShot bit.

This approach allows packet transmission with minimal CPU overhead while maintaining consistent timing.

### Transmission Sequence

```text
Throttle Command
        │
        ▼
Build 16-bit DShot Packet
        │
        ▼
Calculate Checksum
        │
        ▼
Convert Bits to PWM Pulse Widths
        │
        ▼
Load DMA Buffer
        │
        ▼
Timer + DMA Generate Waveform
        │
        ▼
ESC Receives Packet
        │
        ▼
Motor Speed Updated
```

## Results

The completed controller successfully maintains the balance arm near the target angle while recovering from external disturbances.

Key outcomes include:

- Stable closed-loop balancing at a 100 Hz control rate
- Reliable DShot150 communication generated entirely in firmware
- Consistent angle estimation using complementary filtering
- Successful recovery from manual disturbances
- Modular firmware architecture separating sensor, controller, and motor control logic




## Development Challenges

Building a reliable closed-loop control system required solving several hardware and firmware challenges throughout development.

### Implementing the DShot Protocol

Rather than using a standard PWM interface, the project communicates with the ESC using the DShot150 digital protocol. This required implementing packet generation, checksum calculation, and waveform transmission entirely in firmware.

To achieve the required timing accuracy, STM32 timers and DMA were used to generate the waveform with minimal CPU overhead.

---

### Sensor Calibration and Angle Estimation

Raw gyroscope measurements contain bias and drift, while accelerometer measurements are inherently noisy.

To obtain a stable angle estimate, the controller:

- Calibrates the gyroscope during startup
- Applies a low-pass filter to gyroscope measurements
- Uses a complementary filter to combine accelerometer and gyroscope data

This provides fast short-term response while maintaining long-term stability.

---

### Closed-Loop PID Tuning

The physical dynamics of the balance arm required iterative tuning of the PID controller.

Multiple rounds of testing were performed to determine:

- Base throttle required to counter gravity
- Stable proportional and derivative gains
- Integral limits to reduce steady-state error
- Different controller gains depending on the direction of motion

The final controller is capable of recovering from external disturbances while maintaining the arm near the target angle.

---

### Hardware Validation

Several debugging tools and techniques were used throughout development, including:

- UART serial debugging
- Oscilloscope measurements
- Logic analyzer verification of DShot timing
- Incremental subsystem testing before integrating the complete controller

These tools were essential for verifying correct communication between the STM32, IMU, and ESC during development.




## Lessons Learned

This project reinforced several important concepts about embedded systems and real-world control design:

- Breaking complex systems into smaller, independently testable modules makes debugging significantly more manageable.
- Real-world control systems require careful consideration of factors that are often ignored in theory, including sensor noise, actuator response, mechanical friction, and battery voltage sag.
- Verifying individual subsystems before integrating them into the complete controller dramatically reduced debugging time and made isolating problems much easier.
- Building a robust embedded system requires both software and hardware debugging, with tools such as serial output, oscilloscopes, and logic analyzers playing an equally important role.
