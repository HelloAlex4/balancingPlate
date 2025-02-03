# Ball balancing Plate System using custom PID function
![IMG_4699](https://github.com/user-attachments/assets/f19e2a78-e578-413c-9565-57dd583de12f)

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
- head controll loop (loop to calculate position of plate with a custom PID function)

These 3 system all work in parallel to eachother in different threads to make sure they dotn hinder eachother from running.

### 5.1 tracking functions
The tracking functions are responsible for tracking the ball and calculating its exact velocity vector in real time. This data is then output back to the head file. The tracking function runns continuously in an external thread to not conflict with other functions.

### 5.1.1 Position detector
The system uses a Camera to capture live image frames of the plate at 30FPS then extracts the balls x, y position using by using a color filter.

### 5.1.2 Velocity calculation
The velocity is calculated by calculating the average x and y delta over 5 frames.
The velocity vector is passed to the control system for prediction-based balancing.

### 5.2.1 movement functions
Movement functions are responsible for moving the motors based on an two input angles.

### 5.2.2 Angle to Motor Height Conversion

To achieve precise plate tilting, the system must compute how much **each of the three motors must move vertically** to match the required plate angles **(θₓ, θᵧ)**. This is done using **inverse kinematics**.

Each of the **three motors** supports a corner of the plate. Tilting the plate along the **X and Y axes** changes the required height of each motor, which is calculated using the following formulas.

---

#### **Motor Height Calculation**  

The required height changes for each motor **(h₁, h₂, h₃)** can be determined using the following equations:

$$
h_1 = L \cdot \tan(\theta_x) + W \cdot \tan(\theta_y)
$$

$$
h_2 = L \cdot \tan(\theta_x) - W \cdot \tan(\theta_y)
$$

$$
h_3 = -L \cdot \tan(\theta_x)
$$

Where:  
- **$h_1, h_2, h_3$** = The required vertical movement of each motor.  
- **$\theta_x, \theta_y$** = The desired tilt angles along the X and Y axes.  
- **$L$** = Distance from the plate’s center to the motor along the X-axis.  
- **$W$** = Distance from the plate’s center to the motor along the Y-axis.  

---

#### **Defining L and W Using Plate Geometry**  

The **three-motor system** follows a **triangular configuration**, where motors are positioned symmetrically around the center.  

Using **trigonometry**, we define **$L$ and $W$** in terms of the total **radial distance ($D$)** from the center to each motor:

$$
L = D \cdot \sin(30^\circ)
$$

$$
W = D \cdot \cos(30^\circ)
$$

Where:  
- **$D$** = The radial distance from the plate’s center to each motor.  
- **$30^\circ$** = The angle between the motors in an **equilateral triangular layout**.

---

#### **Final Height Equations**  

By substituting the **$L$ and $W$** equations into the original height equations, we obtain the final motor height calculations:

$$
h_1 = D \cdot \sin(30^\circ) \cdot \tan(\theta_x) - D \cdot \cos(30^\circ) \cdot \tan(\theta_y)
$$

$$
h_2 = D \cdot \sin(30^\circ) \cdot \tan(\theta_x) + D \cdot \cos(30^\circ) \cdot \tan(\theta_y)
$$

$$
h_3 = -D \cdot \tan(\theta_x)
$$

Where:  
- **$D$** = Distance from the plate’s center to the motor.  
- **$\theta_x, \theta_y$** = Desired tilt angles along the X and Y axes.  
- **$h_1, h_2, h_3$** = The required vertical movement of each motor.  

---

### **Why This Calculation is Important**
- **Precision Control:** Ensures the plate tilts correctly by calculating exact motor height changes.  
- **Mechanical Stability:** Prevents excessive force on any single motor.  
- **Smooth Motion Translation:** Bridges the gap between control inputs and physical movements.  

This section forms the foundation of the motor control system, as the next steps will convert height changes into stepper motor angles.


### 5.2.3 Height to angle calculation
To calculate the motor angle in relation to the motor endpoint height and the other way arround the following 2 fomulas are used.

**height**

$$
h(\theta) = L_1 \cdot \sin(\theta) + L_2 \cdot \sin\left(\cos^{-1}\left(\frac{x_2 - L_1 \cdot \cos(\theta)}{L_2}\right)\right)
$$

where:
- L_1  and  L_2  are the lengths of the first and second arms, respectively.
- x_2  is the fixed horizontal position of the endpoint.
- \theta  is the angle of the first arm relative to the horizontal.

**angle:**

To calculate the required angle  \theta  for a given desired height  h_{\text{desired}} , the system uses a combination of inverse kinematics and a numerical root-finding algorithm. The formula used for calculating the angles is the same as the one for calculating the height mentioned above, except that the fsolve method is used to iterativly solve for the angle by minimizing the difference between h(\theta) and h_{\text{desired}}.
This is necessary since the relationship between h(\theta) and \theta is nonlinear.

This approach ensures that the angle calculation accounts for the geometric constraints of the system while maintaining high precision.

### 5.2.4 Motor movement
Motors are moved linearly and directly by setting the direction pin and toggling the step pin from the Raspberry Pi. For smooth and precise movement, the motors operate in 1/4 steps with a 0.0001-second delay between each tick. The current motor position is tracked by counting the ticks each motor has moved since the start of the program. This is possible because all motors start at the same low position during initialization and only move upward after the program begins.

### 5.3 head controll loop
The head controll loop uses a custom PID function to calculate the desired angles based on the velocity vector and the balls position.

### 5.3.1 custom PID function
A custom PID function is used in comparison to the traditional function to allow for more precise controll. The custom function stil consists of the 3 PID parts.

- **proportional** - The proportional term moves the ball to the goal position based on the current error
- **Integral** - The integral term accumulates past errors and corrects any error that remains after proportional correction
- **Derivative** - The derivative term corrects for rapid changes in the balls position

Since a custom PID implementation is being used more details will be elaborated on in the following paragraphs.

### 5.3.2 Proportional term
The proportional term calculates the recommended angle based on the ball’s current position relative to the target. However, the system does not move the plate proportionally to the ball’s position. Instead, it uses a root function to create a non-linear response, which smooths movements for small deviations while applying stronger corrections for larger deviations.

The proportional term is defined as:

$$
\text{Angle} = -4 \cdot \left( \frac{\text{coords} - \text{goal}}{450} \right)^{0.8}
$$

Where:
- \text{coords} : The current X,Y-coordinate of the ball.
- \text{goal} : The target X,Y-coordinate for the ball.
- 450 : A scaling factor to normalize the positional difference.
- 0.8 : The exponent applies a non-linear smoothing, reducing the impact of large deviations.
- -4 : A proportional scaling factor that determines the angle’s magnitude.

For negative positional differences ( \text{coords}_y < \text{goalY} ), the system adjusts the sign of the correction to maintain proper direction:

$$
\text{Angle} = -4 \cdot \text{sgn}(\Delta) \cdot |\Delta|^{0.8}, \quad \text{where } \Delta = \frac{\text{coords} - \text{goal}}{450}.
$$

This approach ensures that the plate responds to the ball’s position non-linearly:
- Small deviations result in subtle corrections.
- Large deviations trigger stronger corrections, but the root function prevents overly aggressive tilts.

By using this non-linear method, the system achieves more stable and intuitive control over the plate.


### 5.3.3 Integral Term
The integral term does not correct proportionally to the balls position but uses a fixed correction based on the motors minimum turning tick, being 0.45 degrees in this case.
The integral Term only comes into effect when the balls average velocity per frame is bellow 2. When this case comes into effect and the balls error from the target is above 100 the plates angle is adjusted towards the goal position by 0.45 degrees. This only happens at a maximum of every 15 frames to allow the ball to start moving.

### 5.3.4 Derivative Term
The derivative term calculates the recommended angle adjustments based on the ball’s velocity along both the X and Y axes. Unlike the proportional term, which reacts to the ball’s position, the derivative term accounts for the rate of change in position (velocity), helping to stabilize the system by damping overshoot or oscillations along both axes.

The derivative terms for the X and Y axes are defined as:

$$
\text{Angle}_{x,\text{derivative}} = \frac{-3}{100} \cdot \left( |v_x|^{1.4} \cdot \text{sgn}(v_x) \right)
$$

$$
\text{Angle}_{y,\text{derivative}} = \frac{-3}{100} \cdot \left( |v_y|^{1.4} \cdot \text{sgn}(v_y) \right)
$$

Where:
v_x  and  v_y : The ball’s velocities along the X and Y axes, respectively.
|v_x|^{1.4}  and  |v_y|^{1.4} : Apply a non-linear power function to scale the velocities, increasing the damping effect for higher speeds.
\text{sgn}(v_x)  and  \text{sgn}(v_y) : Preserve the direction of the velocities (positive or negative).
\frac{-3}{100} : Scales the output to ensure the corrections remain smooth and subtle.

Non-Linear Behavior

The derivative term uses an exponent of 1.4 to create a non-linear damping effect:
- Small velocities lead to smaller corrections, ensuring smooth plate movement.
- Large velocities are damped more aggressively, reducing the risk of overshooting or destabilizing the system.

Role in the Control System

The derivative term is crucial for stabilizing the ball by:
	1.	Counteracting momentum: Slowing the ball down as it approaches the target.
	2.	Reducing oscillations: Preventing the ball from overshooting the goal position.

By applying the derivative term to both axes, the system ensures that the plate responds dynamically to stabilize the ball’s movement in all directions, resulting in a smoother and more stable control response.

### 5.3.5 Terms
All 3 of these terms recommend their angle adjustion which then gets summed together to get the final angle the plate will go to.
All 3 parts of this system run in parallel ande different threads to make sure they work smoothly together.


## 6 Demonstration and Results

https://github.com/user-attachments/assets/8e9c09fc-2c4e-402d-9fc7-06fbf60173ed

**Demonstration of the ball being pushed and setteling again.
More videos and pictures can be found within the reposotory folder**


https://github.com/user-attachments/assets/7c7f4390-5ed0-4abd-bd12-84e088d4dc7a

**Demonstration parallel to the plate for a better vision of the System in action while the ball is being pushed**


