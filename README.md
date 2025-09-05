# WRO-2025-Future-Engineers

<img width="309" height="309" alt="Maverick Logo" src="https://github.com/user-attachments/assets/e8a05800-0034-40bf-95e7-3ad27c5d8f7a" />


[![WRO - Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-2e52af)](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
[![YouTube - Opening Race](https://img.shields.io/badge/YouTube-▶️%20Opening_Race-df3e3e?logo=youtube)](To add link )
[![YouTube - Obstacle Race](https://img.shields.io/badge/YouTube-▶️%20Obstacle_Race-df3e3e?logo=youtube)](To add link)

**This is the github repository for team MAVERICK for WRO 2025 INDIA. You'll find our documentation in this readme**

We are Team Maverick – Shaik Sameer, Jagadeswar, and Rino Shajan, guided by Ms. N. Seenu. Competing for the first time at WRO India, GMR Hyderabad, we bring creativity, problem-solving, and technical skills. Driven by passion and dedication to win, we aim to learn, innovate, and make our debut unforgettable.

## Contents

- [Mobility and Hardware Design](#Mobility-Management)
- [Power and Sense Management](#Power-and-Sense-Management)
  - [Power Management](#Power-Management)
  - [Sense Management](#Sense-Management)
  - [Wiring Diagram](#Wiring-Diagram)
  - [Bill of Materials](#Bill-of-Materials)
- [Obstacle Management](#Obstacle-Management)
- [Photos](#Photos)
- [Videos](#Videos)
- [Enabling Reproducibility](#Enabling-Reproducibility)

<!-- Mobility management discussion should cover how the vehicle movements are managed. What motors are selected, how they are selected and implemented.
A brief discussion regarding the vehicle chassis design /selection can be provided as well as the mounting of all components to the vehicle chassis/structure. The discussion may include engineering principles such as speed, torque, power etc. usage. Building or assembly instructions can be provided together with 3D CAD files to 3D print parts. -->

## Mobility Management

The robot is built on a custom lightweight chassis, designed by our team and fabricated using a combination of 3D-printed parts and lightweight structural materials. The chassis is optimized for stability, weight balance, and ease of component integration. All critical electronics and power components are mounted on two separate layers for accessibility — the lower deck houses the drivetrain and power distribution, while the upper deck mounts the Raspberry Pi, camera, and sensor modules.

The drivetrain uses a single 12V 1000 RPM geared DC motor, which drives the rear two wheels through a common axle and gear system. This setup eliminates the need for complex differential drive while ensuring both rear wheels receive equal rotational power. The motor is positioned as low as possible to maintain a low center of gravity for improved stability at higher speeds.

Front-wheel steering is achieved using an SG90 servo motor, which is connected to the steering linkage via custom 3D-printed brackets and lightweight gears. The servo is capable of providing precise angular control for smooth turning, which is crucial for navigating the competition track. The steering geometry is tuned to balance turn radius and stability.

The robot uses 65mm rubber wheels for optimal grip on the WRO competition mat. Gear ratios are selected to provide a good balance between speed and torque, ensuring smooth acceleration and reliable obstacle avoidance.

To mount the motor, servo, and electronics securely, we designed custom 3D-printed couplings and brackets. These are fastened using standard M3 hardware, with nut slots embedded in the print for secure assembly without support structures. The Raspberry Pi Camera Module 3 NoIR is mounted at the front on an adjustable bracket to fine-tune the field of view for vision-based detection tasks.

The design emphasizes:
Weight reduction for faster acceleration
Stability via low center of gravity
Modular construction for easy maintenance and upgrades

<!-- Power and Sense management discussion should cover the power source for the vehicle as well as the sensors required to provide the vehicle with information to negotiate the different challenges. The discussion can include the reasons for selecting various sensors and how they are being used on the vehicle together with power consumption. The discussion could include a wiring diagram with BOM for the vehicle that includes all aspects of professional wiring diagrams. -->

## Power and Sense Management

### Power Management

We utilize a 3S LiPo battery (11.1V, 1000mAh) as the primary power source for the entire robot. This lightweight battery was chosen to meet the WRO weight constraints while providing sufficient energy for both the drive system and control electronics.

The drive motor — a 12V 1000 RPM geared DC motor — is powered through a Cytron MD20A motor driver, which can handle the required stall currents while offering smooth PWM control. The L298n receives full battery voltage directly from the LiPo for maximum efficiency. Steering is controlled by an SG90 servo, powered via the regulated 5V output from the buck converter.

To power the Raspberry Pi 4 and Arduino Nano, the LiPo is connected to a DC-DC buck converter, which steps down the 11.1V to a stable 5V. The Pi 4 is the main onboard computer responsible for camera processing, sensor fusion, and high-level navigation decisions. The Arduino Nano is connected to the Pi via USB serial and handles low-level motor and servo control, ensuring real-time responsiveness.

Sensor Suite:
4 Ultrasonic Sensors – Primary obstacle detection for front and side clearance.
2 VL53L0X ToF Sensors – Short-range high-precision distance measurement.
2 TCRT5000 Infrared Sensors – Line and edge detection.
MPU-6050 IMU – Orientation and stability feedback.

All I2C-based sensors (VL53L0X and MPU-6050) are connected through a shared bus, with address management to avoid conflicts. The ultrasonic and IR sensors are wired to the Arduino Nano’s digital pins for rapid polling without burdening the Pi.

Power consumption was carefully considered:
Raspberry Pi 4 – ~1.5–2.0A peak during vision processing
SG90 servo – up to 1A during peak load
motor – ~0.5–1.5A average, up to 3A in brief stalls

The buck converter is rated to handle peak current demands, ensuring stable operation even under load. By isolating high-current motor operations on the MD20A, we protect the control electronics from voltage dips or electrical noise.

This setup ensures:
Stable power delivery to sensitive electronics
High current capacity for drivetrain needs
Efficient separation of control and power stages for reliability

### Sense Management

For our WRO 2025 robot, we use a Raspberry Pi Camera Module 3 NoIR as the primary vision sensor. This camera was selected for its high resolution, low-light capability, and ability to integrate directly with the Raspberry Pi 4’s CSI interface, ensuring low-latency video streaming for real-time processing. The wide dynamic range of the NoIR variant allows for better detection of color-based markers and objects under varying lighting conditions in the competition arena.

The camera connects directly to the Pi 4 and is powered through the CSI interface, eliminating the need for additional power lines. To optimize detection accuracy, the camera is mounted at the front-center of the chassis on an adjustable 3D-printed bracket, allowing us to fine-tune the tilt and height during calibration. The mount is designed to maintain a stable field of view, minimizing vibrations from the drivetrain.

Unlike setups that rely solely on vision, our robot uses the camera in combination with a multi-sensor array:
4 Ultrasonic Sensors – For obstacle detection and front/side distance measurements.
2 VL53L0X ToF Sensors – For precise short-range distance measurement where ultrasonic accuracy drops.
2 TCRT5000 IR Sensors – For line and edge detection.
MPU-6050 IMU – For orientation tracking and stability corrections.

This hybrid approach ensures that even if lighting conditions or arena textures vary, the robot can still navigate reliably using redundant sensing. The camera handles color zone detection, marker recognition, and path tracking, while the ultrasonic and ToF sensors provide robust physical distance measurement to complement visual estimates.

The Pi 4 processes the camera feed in real time using OpenCV-based algorithms. This enables:
Color segmentation for identifying parking zones and markers
Contour detection for wall and obstacle recognition
Hybrid fusion with distance sensor data for improved accuracy

By combining computer vision with hardware-based sensors, we achieve a balance between perception flexibility and measurement reliability, ensuring consistent performance in the unpredictable competition environment.

### Wiring Diagram

![Wiring Diagram]()

### Bill of Materials

 | Component | Quantity | Function |
|-----------|----------|----------|
| Raspberry Pi 4 | 1 | Main onboard computer (**Host Controller**) responsible for running the OS, handling computer vision, high-level navigation logic, and communicating with the Arduino Nano. |
| Raspberry Pi Cooling Fan | 1 | Prevents thermal throttling of the Raspberry Pi 4 during high-load vision processing. |
| Arduino Uno | 1 | Handles low-level control tasks such as motor driver signals, servo steering, and sensor data acquisition, offloading the Pi. |
| Raspberry Pi Camera Module 3 NoIR | 1 | Primary vision system for color detection, path tracking, and marker recognition. |
| MPU-6050 IMU | 1 | Measures acceleration and angular velocity for orientation tracking and stability correction. |
| VL53L0X ToF Sensor | 2 | Provides high-precision short-range distance measurements for obstacle detection in tight areas. |
| Ultrasonic Sensor | 4 | Measures mid-range distances for obstacle avoidance and wall detection. |
| TCRT5000 IR Sensor | 2 | Detects lines, edges, and surface transitions. |
| SG90 Servo Motor | 1 | Controls the front steering mechanism for accurate directional changes. |
| Rhino 1000 RPM Geared DC Motor | 1 | Rear axle drive motor that provides forward and reverse propulsion via a bevel gear linkage. |
| L298n Motor Driver | 1 | Controls the  motor with PWM speed control and direction switching, supporting higher currents than standard H-bridges. |
| 65 mm Rubber Wheels | 4 | Provide traction and stability on the WRO arena surface. |
| Gears & Axle Linkage | — | Transmit power from the  motor to the rear wheels. |
| 3S LiPo Battery 11.1V 1000mAh | 1 | Lightweight, high-discharge battery that powers both the drive system and electronics. |
| Buck Converter | 1 | Steps down LiPo voltage to stable 5V for the Raspberry Pi and Arduino. |
| Power Distribution Board (PDB) | 1 | Distributes power from the battery to multiple subsystems. |
| XT60 / JST Connectors & Wires | — | Ensure reliable and safe electrical connections between components. |
| Fuse | 1 | Protects electronics from overcurrent damage. |
| Power Rocker Switch | 2 | Allows independent power control for motor system and electronics. |

<!-- Obstacle management discussion should include the strategy for the vehicle to negotiate the obstacle course for all the challenges. This could include flow diagrams, pseudo code and source code with detailed comments. -->

All files for the 3D printed parts can be found in the [3D-Printed-Parts](/cad) folder. All parts can be printed without supports at 0.2mm layer height.
The wiring diagram can be found in the [Wiring Diagram](/media/wiringdiagram.jpg) file. The instructions for the LEGO chassis can be found in the []().

## Obstacle Management

### Opening Race

The robot’s autonomous behavior is implemented using a Behavior Tree model for flexibility and modularity. The core logic is handled by the StateMachine class, which manages current states, state transitions, and future-scheduled actions. This scheduling feature is particularly useful for timed maneuvers, such as delaying a steering adjustment to avoid clipping an inner wall.

Core States for WRO 2025:
STARTING – Initial acceleration and alignment based on lane detection.

PD-CENTER – Continuous path-following using PD control from vision and sensor fusion.

TURNING-L/R – Controlled Ackermann-style turns triggered by marker or corner detection.

DONE – Stop sequence upon completion of the final lap.

**Perception Pipeline**

**Camera Processing**

The Raspberry Pi Camera Module 3 NoIR captures live frames at a calibrated angle and height.
The top portion of each frame is cropped to ignore the floor and focus on wall & obstacle detection.
Black pixel regions are isolated using OpenCV’s cv2.inRange filter.
Wall edges are extracted via cv2.Canny edge detection.
np.argmax finds the top edge positions of walls per column; np.diff identifies sudden wall height changes to detect inner wall entrances.

**Sensor Fusion**

Ultrasonic & ToF sensors validate wall distances from vision to prevent false positives.
IR sensors detect arena boundary lines for edge protection.
IMU feedback ensures smooth heading control during turns and recovers from drift.

**Driving & Control**

A PD Controller manages steering based on the offset between the detected outer wall and a pre-calibrated reference position. Only the outer wall is followed to reduce collision risks, especially if inner wall spacing is randomized.
The servo motor applies steering corrections.
The N20 motor maintains a speed tuned for sensor reaction time and safe maneuverability.

**Marker & Turn Detection**

A central region of interest in the camera feed is used to detect blue and orange arena lines.
The frame is converted to HSV color space for robust detection under varying lighting.

On detecting a line:
Blue line → trigger left turn sequence.
Orange line → trigger right turn sequence.
Each turn decrements a corner counter, ensuring the robot stops exactly at the end of the required lap.

This approach blends high-speed camera vision with real-time distance sensing, resulting in a robust navigation system that adapts to both static arena structures and randomized obstacle gaps.

![Wall Detection](/media/walls masking.jpg)

(about the image)

### Obstacle Race

**Pillar Detection & Avoidance**

In addition to generating a binary black-and-white map for wall and lane detection, we also process the cropped RGB frame from the Raspberry Pi Camera Module 3 NoIR by converting it to HSV color space. The HSV representation allows for more robust color segmentation under varying lighting conditions, especially for red and green pillar detection, which is a key obstacle in the WRO 2025 arena.

Once converted to HSV:

Color Masking
Green Detection: A single hue range is sufficient.
Red Detection: Special handling is required due to the hue value wrap-around at 0°/360°.

We create two separate masks:
Lower Red Range: e.g., 0°–10°
Upper Red Range: e.g., 350°–360° (in OpenCV’s scale, near 170–180)
These masks are combined using a bitwise OR operation to capture all shades of red accurately.

Edge & Contour Extraction
After applying the color mask, cv2.Canny is used to detect edges.
cv2.findContours identifies pillar outlines.
For each detected pillar, we compute its centroid, width, and height.
These parameters are passed to the behavior tree for decision-making.

Behavior Tree Integration
Two new states are introduced:
TRACKING-PILLAR – Maintains position while approaching a detected pillar.
AVOIDING-PILLAR-R/G – Executes avoidance maneuvers depending on pillar color.
If the detected pillar is red, the avoidance path is pre-programmed to one side. If green, it is routed to the opposite side.

Sensor Fusion Support
Ultrasonic and ToF sensors validate the detected object’s distance to ensure it’s within avoidance range.
IMU readings keep the robot’s steering corrections smooth during quick obstacle bypasses.

This combined HSV-based detection and sensor-assisted avoidance ensures pillars are handled with both color accuracy and precise spatial awareness, minimizing false triggers and maintaining speed through the course.

![Wall Detection]()

()

<!-- Insert image with behavior tree here -->

![Behavior Tree]()

<!-- Pictures of the team and robot must be provided. The pictures of the robot must cover all sides of the robot, must be clear, in focus and show aspects of the mobility, power and sense, and obstacle management. Reference in the discussion sections 1, 2 and 3 can be made to these pictures. Team photo is necessary for judges to relate and identify the team during the local and international competitions. -->

### Parking Strategy
Upon completing the three laps, the vehicle initiates a Parking Sequence, beginning with the detection of parking boundaries using ultrasonic and ToF sensor measurements. These sensors assess the available space and help align the vehicle with the detected parking area. If sufficient space is confirmed, the vehicle executes a controlled parallel parking maneuver with steering guided by the servo motor. Following this, the system verifies the vehicle’s final position through sensor feedback to ensure proper alignment, at which point it halts. In cases where space is insufficient, the vehicle adjusts its orientation and resumes scanning with the sensors to locate an appropriate parking area, repeating the process until successful parking is achieved.

<p align = "center">
  <img src="">
</p>

## Photos

| ![Front](/V-Photos/Front.jpg)                 | ![Back](/V-Photos/back.jpg)     |
| -------------------------- | ---------------------------- |
| ![Left](/V-Photos/left.jpg)                  | ![Right](/V-Photos/Right.jpg)   |
| ![Top](/V-Photos/Top.jpg)                   | ![Bottom](/V-Photos/Bottom.jpg) |

![Team]()
![Team]()

**A fummy pic 😎🤪**

<!-- The performance videos must demonstrate the performance of the vehicle from start to finish for each challenge. The videos could include an overlay of commentary, titles or animations. The video could also include aspects of section 1, 2 or 3 -->

## Videos

<!-- https://michaelcurrin.github.io/badge-generator/#/generic -->

[![YouTube - Opening Race](https://img.shields.io/badge/YouTube-▶️%20Opening_Race-df3e3e?logo=youtube)]()

[![YouTube - Obstacle Race](https://img.shields.io/badge/YouTube-▶️%20Obstacle_Race-df3e3e?logo=youtube)]()

## Enabling Reproducibility

To enable the reproduction of our robot, we provide the following installation instructions:

1.Install Raspberry Pi OS

Install Raspberry Pi OS on your Raspberry Pi using the official guide:
https://www.raspberrypi.org/documentation/installation/installing-images/README.md
While the OS is installing, flash the Arduino code to the Arduino Nano.

The Arduino code is located in the /arduino/OutputProxy folder.

Use PlatformIO to upload the code.

2.Set up the Raspberry Pi
After booting up the Raspberry Pi, connect via SSH and install the required packages:

```bash
sudo apt-get update
sudo apt-get install python3-opencv python3-websockets python3-numpy python3-pyserial
```
Enable the Camera
Enable the camera interface:
```bash
sudo raspi-config
```
Reboot for changes to take effect, then install the PiCamera2 Python module:

```bash
sudo apt-get install python3-picamera2
```
4.Clone the Repository
```bash
git clone https://github.com/Maverick-WRO/WRO-2025-Future-Engineers
```

5.Running the Robot in Dev Mode

Check in the config file if the correct USB port is set for the Arduino.
Find the correct port:
```bash
ls /dev/tty*
```
Look for the port that matches the Arduino connection and update it in the config file.
Make sure pillars are enabled/disabled in the config file as needed, and that no fixed round 
direction is set.

6.Run the Main Script
```bash
cd WRO-Mavericks 2025/raspy
python3 roi.py
```

7.Control the Robot

Open the web interface in your browser.

Click Connect to start autonomous driving.

To stop:
Close the web interface
Press the Stop button on the robot
Press Ctrl+C in the SSH session

