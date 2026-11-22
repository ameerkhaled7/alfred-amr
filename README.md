# ALFRED — Automated Logistics & Freight Robot for Efficient Delivery

![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04_LTS-E95420?logo=ubuntu&logoColor=white)
![Hardware](https://img.shields.io/badge/Compute-Jetson_Nano_%2B_Arduino_Mega-76B900?logo=nvidia&logoColor=white)
![CAD/FEA](https://img.shields.io/badge/CAD_%26_FEA-SolidWorks_%2B_ANSYS-005A9C)
![Payload](https://img.shields.io/badge/Static_Payload-500_kg-blue)

**ALFRED** is a heavy-duty, differential-drive Autonomous Mobile Robot (AMR) designed and fabricated for indoor warehouse and factory intralogistics. Built from the ground up as a B.Sc. Mechatronics Engineering graduation project at **Nile University** (sponsored by **ITIDA** and **Glide Mobility**), the platform integrates a structural steel/aluminum chassis validated for **500 kg static loads**, dual **350W BLDC hub motors**, a **Jetson Nano + Arduino Mega 2560** dual-compute control architecture, and a **ROS Noetic** SLAM and autonomous navigation stack.

<table align="center">
  <tr>
    <td align="center" width="50%">
      <img src="alfred-autonomous-mobile-robot/Media/alfred-prototype.jpg" alt="ALFRED Assembled AMR Prototype" width="400" /><br />
      <sub><b>Assembled AMR Prototype</b></sub>
    </td>
    <td align="center" width="50%">
      <img src="alfred-autonomous-mobile-robot/Media/cad-assembly.PNG" alt="ALFRED SolidWorks CAD Assembly" width="400" /><br />
      <sub><b>SolidWorks CAD Assembly</b></sub>
    </td>
  </tr>
</table>

---

## Key Specifications

| Parameter | Specification |
| :--- | :--- |
| **Static / Dynamic Load Capacity** | 500 kg (5,000 N) static / 381.6 kg dynamic traction limit |
| **Base Footprint** | 1000 mm × 750 mm (laser-cut 2 mm ST37 steel + 20×20 mm tubular subframe) |
| **Drive Configuration** | Differential drive: 2 × 36V 350W CZJB BLDC hub motors (Ø 520 mm, 30 N·m peak torque) + 4 × Ø 150 mm swivel casters |
| **High-Level Compute** | NVIDIA Jetson Nano (Ubuntu 20.04, ROS Noetic) + HDMI operator UI display |
| **Low-Level Controller** | Arduino Mega 2560 (PWM motor control, odometry acquisition, safety interlocks) |
| **Perception & Localization** | Slamtec RPLIDAR A1 (360° 2D LiDAR), Intel RealSense D435c (RGB-D), GY-87 9-DOF IMU, 2 × KY-003 magnetic wheel encoders, 2 × front ultrasonic sensors |
| **Power & Safety System** | 36V 10.4Ah Li-ion battery, dual isolated DC-DC buck converters (5V/4A Jetson rail + 5V sensor/MCU rail), fuses, physical E-Stop push button |
| **FEA Structural Validation** | Max deformation **0.28 mm** (`2.81e-4 m`), minimum safety factor **1.78** under 5,000 N load |

---

## System Architecture

ALFRED separates high-level perception and path planning from deterministic real-time actuation and safety monitoring:

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│                     HIGH-LEVEL COMPUTE: NVIDIA JETSON NANO                   │
│                            (Ubuntu 20.04 • ROS Noetic)                       │
│                                                                              │
│  ┌──────────────┐   ┌──────────────────┐   ┌──────────────────────────────┐  │
│  │ RPLIDAR A1   │──▶│  2D SLAM & AMCL  │──▶│     move_base Navigation     │  │
│  │   (/scan)    │   │ (gmapping/map)   │   │  (Global & Local Costmaps)   │  │
│  └──────────────┘   └──────────────────┘   └──────────────┬───────────────┘  │
│  ┌──────────────┐   ┌──────────────────┐                  │                  │
│  │ RealSense    │──▶│  3D Depth / Obs. │                  │ /cmd_vel         │
│  │ D435c (USB3) │   │    Perception    │                  ▼                  │
│  └──────────────┘   └──────────────────┘   ┌──────────────────────────────┐  │
│  ┌──────────────┐                          │   Extended Kalman Filter     │  │
│  │ HDMI Monitor │◀── RViz / Operator UI ───│  (EKF: Wheel Odom + IMU Yaw) │  │
│  └──────────────┘                          └──────────────┬───────────────┘  │
└───────────────────────────────────────────────────────────┼──────────────────┘
                                                            │ USB Serial
                                                            │ (rosserial)
┌───────────────────────────────────────────────────────────▼──────────────────┐
│                   LOW-LEVEL CONTROLLER: ARDUINO MEGA 2560                    │
│                                                                              │
│  ┌─────────────────────┐   ┌───────────────────────┐   ┌──────────────────┐  │
│  │ 2× KY-003 Magnetic  │──▶│  Dead-Reckoning Odom  │   │  2× Ultrasonic   │  │
│  │ Encoders (8 tick/r) │   │  + GY-87 IMU (I2C)    │   │ (<50 cm E-Stop)  │  │
│  └─────────────────────┘   └───────────────────────┘   └────────┬─────────┘  │
│                                                                 │            │
│  ┌──────────────────────────────────────────────────────────────▼─────────┐  │
│  │               Dual PWM Speed Control + Resistor Divider Interface      │  │
│  └───────────────┬──────────────────────────────────────────┬─────────────┘  │
└──────────────────┼──────────────────────────────────────────┼────────────────┘
                   ▼                                          ▼
        ┌─────────────────────┐                    ┌─────────────────────┐
        │ RioRand BLDC Driver │                    │ RioRand BLDC Driver │
        │  (Left Hub Motor)   │                    │ (Right Hub Motor)   │
        └─────────────────────┘                    └─────────────────────┘
```

---

## Mechanical Design & ANSYS FEA Validation

### Structural Fabrication
* **Chassis & Subframe:** Laser-cut **2 mm ST37 steel** base and payload plates reinforced with welded **20×20×2 mm ST37 square steel tubing** ("bones") to distribute concentrated cargo loads.
* **Modular Uprights & Mast:** **40×40 mm aluminum V-slot extrusions** fastened via T-slot brackets and rivet nuts separate the lower electronics bay from the upper cargo deck. An **80×40×2 mm steel mast** elevates the RPLIDAR A1 above the payload plane for an unobstructed 360° scan field.
* **Custom Enclosures:** Protective housings for the Jetson Nano, Arduino Mega, RioRand drivers, buck converters, RPLIDAR, and front ultrasonic sensors were 3D-printed in **PLA+**.

### Finite Element Analysis (ANSYS Static Structural)
Three 5,000 N (500 kg) loading scenarios—**Center**, **Front-Heavy**, and **Rear-Heavy**—were simulated to evaluate von-Mises stress, total deformation, and fatigue safety factor:

| Load Case (5,000 N) | Max Equivalent Stress | Max Deformation | Min Safety Factor |
| :--- | :---: | :---: | :---: |
| **Design Iteration 1 (Baseline)** | — | Excessive plate bending | 1.04 |
| **Final Design — Center Load** | 48.42 MPa | **0.281 mm** | **1.78** |
| **Final Design — Off-Center (Front/Rear)** | 63.48 MPa | **0.203 mm** | > 1.78 |


### Differential Drive Kinematics
With wheel radius $r = 0.26\text{ m}$ and track width $s$, wheel angular velocities $(\dot{\phi}_L, \dot{\phi}_R)$ map to global robot pose rates $(\dot{x}, \dot{y}, \dot{\theta})$ via:

$$\dot{x} = \frac{r(\dot{\phi}_L + \dot{\phi}_R)}{2}\cos\theta, \qquad \dot{y} = \frac{r(\dot{\phi}_L + \dot{\phi}_R)}{2}\sin\theta, \qquad \dot{\theta} = \frac{r(\dot{\phi}_R - \dot{\phi}_L)}{s}$$

---

## ROS Noetic Simulation & Progressive Validation

1. **URDF/Xacro & Gazebo Digital Twin:** Exported the SolidWorks assembly into a parameterized `.xacro` model with 9 `.stl` collision/visual meshes (`robot_footprint` → `base_link` → left/right drive wheels, 4 casters, and `lidar_link`). Validated `gmapping` SLAM, `amcl` localization, and `move_base` obstacle avoidance in Gazebo + RViz prior to physical deployment.
2. **Stage 1 — PWM Bench & Serial Verification:** Validated 0–255 PWM speed modulation from the Arduino Mega to both RioRand ZS-X11H BLDC controllers.
3. **Stage 2 — Wireless Bluetooth Teleoperation:** Verified manual forward and differential turning maneuvers on the full chassis via a smartphone Bluetooth interface.
4. **Stage 3 — Closed-Loop IMU Heading Hold & Ultrasonic Safety Stop:** Implemented proportional yaw-drift correction ($\pm 5\text{ PWM}$ differential trim) using the GY-87 IMU alongside an automatic zero-PWM collision cutoff when front ultrasonic sensors detect obstacles within **50 cm**.
5. **Stage 4 — Full ROS Noetic SLAM & Autonomous Waypoint Navigation:** Deployed `gmapping`, `amcl`, EKF sensor fusion, and `move_base` on the Jetson Nano, validating `teleop_twist_keyboard` mapping, 1-meter precision goal tracking, and autonomous indoor obstacle avoidance.


---

## Hardware Troubleshooting & Engineering Iterations

Real-world hardware integration uncovered several undocumented ("black-box") component constraints that required mechanical and electrical redesigns:

1. **Unidirectional Freewheel Hub Motors:** Bench testing revealed an internal planetary freewheel clutch inside the sponsored CZJB BLDC wheels that physically prevented reverse torque and active electronic braking. *Fix:* Adapted the differential-drive motion planner to forward-only arcs, single-wheel pivot turns, and coast-to-stop deceleration profiles.
2. **Hollow-Shaft Torsional Failure & Form-Fit Redesign:** Under sustained torque, friction-clamped wheel shafts rotated ~90° in their mounts, twisting the internal Hall-sensor harness passed through the hollow axle and shorting phases to ground. *Fix:* Disassembled the hub, replaced the burnt Hall-sensor PCB and wiring, and cross-drilled a steel locking pin through the shaft and bracket—converting a friction fit into a positive mechanical form fit.
3. **External Magnetic Odometry + IMU Fusion:** Tapping the motor's internal Hall lines loaded down the RioRand speed-control circuit and cut motor power by 50%. *Fix:* Mounted **8 permanent magnets** externally on each wheel rim paired with **KY-003 Hall sensors** (8 ticks/rev) for linear displacement, fused with **GY-87 IMU** yaw via an Extended Kalman Filter to eliminate rotational drift.
4. **Safe Analog Driver Interfacing:** Without schematics for the RioRand controller's 0–5V speed input, direct microcontroller connection risked damaging the onboard potentiometer. *Fix:* Characterized the 6-pin control header and built an external resistor-divider interface to regulate speed safely.
5. **RealSense D435c Bandwidth Stabilization:** Resolved depth-stream dropouts and frame-rate glitches by migrating the camera from USB 2.0 test ports onto the Jetson Nano's dedicated **USB 3.0** bus.

---

## Repository Structure

```text
alfred-autonomous-mobile-robot/
├── CAD/                            # SolidWorks assembly, modeling steps & ANSYS stress analysis
├── Docs/                           # Graduation thesis PDF
├── Firmware/                       # Arduino Mega 2560 (.ino) test & low-level control sketches
├── Hardware/                       # Wiring & hardware build photos
├── Media/                          # Prototype photo & CAD assembly render
└── System-Architecture/            # Hardware block diagram
```

---

## Project Team & Credits

Developed at **Nile University — School of Engineering and Applied Sciences** (MENG 495, Fall 2025) under the supervision of **Prof. Raafat Shalaby**, with technical and manufacturing support from **ITIDA** and **Glide Mobility**.

* **Amir Khaled Shaker** — Hardware Integration, Electrical Architecture & Embedded Testing
* **Mazen Mohamed Kandil** — 3D CAD Modelling & ANSYS FEA Simulation
* **Mrwan Mohamed Kandil** — ROS Noetic Stack, Gazebo Simulation & Navigation
* **Osama Elsayed Ryad** — Experimental Validation & Testing Verification
* **Mohamed Mahmoud Elsouda** — Operations & Project Coordination
* **Mostafa Yasser El-Melijy** — Technical Research & Documentation
