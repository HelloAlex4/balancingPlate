# Ball balancing Plate System
PUT PICTURE HERE

## 1. Project overview
this project presents the development of a **ball-balancing plate system** using three stepper motors to precisely and dinamically control the tilt and vertical position of a platform. The system is able to balance a ball atop of the platform and move it to any specified point on the plate. In addition to that the the System is able to launch the ball in the air and catch it again on the Plate.

## 2. Introduction
The goal of this project was to build a plate that balances a ball on top of it and is able to preciselly controll the position of the ball. The system requires a wide range of skills to complete. On one hand it requires expertise in precise robot building and experience with stepper motors. On the other Hand it is a great challenge in computer programming and real world modeling.

## 3. System Overview
The system utilizes three stepper motors to balance the plate while a camera tracks the position and velocity of the ball to dynamically adjust the plate’s angle.

### 3.1 The Systems

#### 1. Ball Tracking
- A camera captures the ball's position on the plate at 30 FPS.
- The position is output as (x, y) coordinates, which are processed for velocity estimation.

#### 2. Control System
- A Raspberry Pi serves as the central controller.
- A PID-like control system calculates the required plate tilt to maintain balance.
- A closed-loop feedback system ensures smooth and accurate corrections in real time.

#### 3. Motor Movement
- Three stepper motors adjust the plate’s tilt angles along two axes.
- The motors operate in sync to provide precise multi-axis movement.
- Three A4988 stepper motor drivers are used to control the motors, configured for quarter stepping.

### 3.2 System Diagram
<img width="471" alt="Screenshot 2025-01-31 at 21 55 48" src="https://github.com/user-attachments/assets/9a61cfe5-c9bf-478c-8ec0-f624e320902d" />


### 3.3 System Details
- Camera updates: 30 FPS
- Motor precision: 800 steps per revolution (0.45 degrees per step)
- Motor speed: 0.0005 seconds delay per step → 900 degrees per second maximum speed

### 3.4 Why This Design?
A three-stepper-motor design was chosen instead of a two-motor configuration with a central pivot due to the following advantages:

- **Higher precision:** Independent motor control allows for more accurate plate tilting.
- **Expanded motion range:** The system can achieve greater tilt angles than a two-motor design.
- **Vertical axis control:** Enables additional functions, such as making the ball jump by dynamically adjusting the plate’s vertical motion.

## 4 Hardware design
This section details the mechanical and electronic components used to construct the ball-balancing plate, including the plate structure, stepper motors, motor drivers, microcontroller, and camera setup.

For further information on exact construction see attached pictures in picture folder

### 4.1 Plate Structure
The plate is a rigid, lightweight platform that tilts along two axes. The design considerations include:
- **Material:** light weight plastic
- **Size:** octagonal with 11.5cm edges
- **Mounting:** Fixed to 3d-printed mounting system that actually connects to the motors with adhesive
- **Degree of Motion:** The plate can tilt up to 20 degrees in all directions.

### 4.2 Stepper Motors and Mounting
The system uses **three stepper motors**, each responsible for tilting the plate at an independent point.  
- **Motor Model:** Nema 17
- **Stepping Configuration:** 1/4 stepps -> 0.45 degrees per step
- **Mounting Method:** The motors are mounted on a 3d-printed bracket 
- **Coupling:** Each motor is connected to the plate using ball joints to allow for motion in all directions

### 4.3 Motor Drivers
To control the stepper motors, three **A4988** stepper motor drivers are used.  
- **Current Limiting:** Set to 0.4A per motor.  
- **Microstepping:** Configured for quarter-step mode to increase precision.  
- **Wiring:** Connected to the microcontroller via step/direction pins and power through a breadboard.

### 4.4 Microcontroller
The system is controlled by a Raspberry Pi 5.  

### 4.5 Camera and Ball Tracking
A Raspberry Pi Camera v2 is used to track the ball’s position.  
- **Resolution:** 1000x1000  
- **Frame Rate:** 30 FPS for real-time tracking.  
- **Mounting Position:** The camera is positioned overhead the plate with a 3d-printed mounting bracket

### 4.6 Power Supply and Wiring
The system requires a stable power source to operate the motors and microcontroller.  
- **Power Requirements:** 12V 3A for motors, 5V for logic 
- **Voltage Regulation:** motor power connected over an external powersupply, microcontroller is powered through raspberry pi 5v output

### 4.7 Mechanical Constraints and Considerations
- The stepper motors must be mounted securely to prevent vibration-induced inaccuracies.
- The plate’s weight and balance are critical to ensure smooth and stable movement.

## 5. Software Impelementation
The software consists of 3 main parts:
- Tracking functions (functions used for tracking the ball)
- movement functions (functions used for moving the plate)
- head controll loop (loop to calculate position of plate with a PID function)

### 5.1 tracking functions

### 5.2 movement functions

### 5.3 head controll loop









