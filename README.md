# Self-driving Car Project

This project simulates a car airbag controller during a collision. The airbag control system is implemented on the STM32F407 Discovery microcontroller, and data is sent to a central MCU (ESP32) for display and analysis.

## Hardware Requirements
- STM32F407 Discovery microcontroller
- ESP32 microcontroller
- Weight sensor
- Accelerometer sensor
- Speed sensor
- CAN Module (TJA1050)
- Servo motors
- L298 DC motor driver

## Project Requirements

### CAN Protocol Development
- Implement the CAN protocol for data transmission and reception between:
  - Airbag Control Unit (ACU) microcontroller (STM32F407)
  - Central MCU (ESP32)

### Airbag Control Unit (ACU) Microcontroller Tasks
- Manage sensor data from the weight, accelerometer, and speed sensors.
- Control servo motors for airbag deployment based on collision detection.
- Interface with the L298 DC motor driver for additional control tasks.

### Central MCU (ESP32) Tasks
- Receive data from the ACU via the CAN protocol.
- Display and analyze airbag deployment data.
- Implement real-time collision analysis algorithms.

### Design and Architecture
- Design software layers and modules for modularity and scalability.
- Ensure efficient data flow between the ACU and Central MCU.
## Results
The simulation results can be found in the `/Result/` directory. This includes:
- **Test data**: Data captured from sensor inputs during simulated collisions.
- **Analysis**: Results of airbag deployment and real-time data analysis by the Central MCU.
- **Visualization**: Graphs and charts representing the collision event and airbag activation.

## How to Build and Run the Project
1. Clone this repository to your local machine.
2. Set up the required hardware components as listed above.
3. Flash the STM32F407 microcontroller with the ACU code.
4. Flash the ESP32 with the Central MCU code for data analysis.
5. Connect the sensors, servo motors, and CAN module to the microcontrollers.
6. Run the simulation to observe airbag deployment during collision detection.

