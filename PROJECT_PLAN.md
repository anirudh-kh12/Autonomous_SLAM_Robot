# MASTER PROJECT REFERENCE
# Autonomous SLAM-Based Mobile Robot with Web Dashboard

**Document purpose:** Persistent project context, planning, implementation guide and future ChatGPT reference.

**Project status:** Planning and environment verification  
**Project version:** Working Draft V1 — scope approved in principle by both teammates; technical and purchasing decisions not yet frozen.

---

# 1. PROJECT OVERVIEW

## 1.1 Project title

**Design and Development of a Low-Cost Autonomous SLAM-Based Mobile Robot with Real-Time Web Monitoring and Manual/Autonomous Control**

Alternative shorter title: **Autonomous Indoor Mobile Robot with SLAM and Web-Based Control.**

## 1.2 Project purpose

This is a long-term personal engineering project being developed by two B.Tech students in Industrial Internet of Things (IIoT).

The project is intended for:

- Learning robotics, embedded systems and autonomous navigation.
- Developing practical software, hardware and mechanical-design skills.
- Creating a strong technical portfolio and GitHub repository.
- Demonstrating the system at future hackathons.
- Potentially developing it into a Semester 7 minor project.
- Exploring improvements that could eventually support research or a more advanced prototype.

**There is no fixed college submission deadline.** The project should be developed at a sustainable pace, with emphasis on understanding, experimentation, documentation, reliability and genuine real-world functionality.

The team will use ChatGPT to understand concepts, make design decisions, implement code, troubleshoot hardware/software integration, evaluate results and maintain documentation.

## 1.3 Primary real-world application

Indoor warehouse material transport.

The robot will navigate a controlled indoor environment containing rooms, corridors, walls and obstacles. It should be able to build a map, estimate its position, travel to a selected destination, detect obstacles and stop or replan when necessary.

The warehouse scenario can later be extended to:

- Laboratory equipment or sample delivery.
- Industrial inspection and environmental monitoring.
- Campus or office delivery demonstrations.
- More advanced autonomous mobile robot applications.

These are possible future applications, not initial project requirements.

## 1.4 Core problem statement

Basic mobile robots commonly depend on predefined movements or local obstacle sensors. Such robots may not maintain a map of an unfamiliar environment or estimate their position accurately enough to navigate to arbitrary destinations.

This project aims to develop a low-cost indoor mobile robot that combines 2D LiDAR, wheel encoders and appropriate robotics algorithms to perform simultaneous localization and mapping (SLAM), estimate its position on a map, plan a route to a selected destination, and respond safely to obstacles.

The system will additionally provide a locally hosted web dashboard over Wi-Fi for live monitoring, manual driving, autonomous goal selection and display of robot status.

The project will first be developed and tested in simulation, then implemented and evaluated on a physical prototype.

---

# 2. TEAM AND AVAILABLE RESOURCES

## 2.1 Team

- Team size: 2 people.
- Both teammates are learning the robotics software stack from scratch.
- Current development computer environment: Windows.
- Linux and ROS 2 experience: beginner level; learning required.
- The team will share responsibility for system integration, testing, safety, documentation and the final demonstration.

## 2.2 Available college resources

The following resources have been confirmed by the user:

1. Campus-wide access to MATLAB and its toolboxes, subject to installation and account entitlements.
2. Two 3D printers in the IoT laboratory:
   - One enclosed printer described as an AutoForge glass-box machine.
   - One open-frame printer with a material wheel.
3. Some electronic components may be borrowed from the laboratory.
4. Remaining components will be purchased as required.

Before mechanical design or procurement, identify the exact printer models, build volumes, available filament materials and borrowing rules.

The project should prioritize lab equipment and existing components to reduce unnecessary expenditure.

## 2.3 Current MATLAB environment

Verified information:

- MATLAB version: 26.1.0.3346908, R2026a Update 5.
- Operating system: Windows 11 Pro Insider Preview, build 26220.
- Products shown by `ver`:
  - MATLAB R2026a.
  - Control System Toolbox R2026a.

The following commands returned `not found`:

- `lidarSLAM`
- `differentialDriveKinematics`
- `occupancyMap`
- `controllerPurePursuit`
- `plannerAStarGrid`
- `ros2node`

This indicates that the corresponding functions were not discoverable in the current MATLAB environment at the time of testing. The Robotics System Toolbox, Navigation Toolbox and ROS Toolbox still need to be checked and installed as necessary.

Do not assume the required toolboxes are installed merely because the college provides access.

---

# 3. AGREED PROJECT SCOPE

## 3.1 Essential features

The initial complete system should include:

1. A differential-drive mobile robot.
2. A custom-designed, 3D-printed chassis.
3. Encoder-based wheel odometry.
4. A 2D LiDAR for environmental mapping.
5. SLAM-based mapping.
6. Robot localization on the generated map.
7. Goal-based autonomous navigation.
8. Global path planning.
9. Obstacle detection and safe stopping.
10. Dynamic obstacle response and route replanning, subject to successful integration.
11. Local Wi-Fi communication.
12. A browser-based dashboard.
13. Manual control through the dashboard.
14. Autonomous navigation commands through the dashboard.
15. Live display of map, robot position, sensor information and system status.
16. Simulation before physical implementation.
17. Repeatable tests and quantitative evaluation.
18. GitHub repository, CAD files, documentation and demonstration video.

## 3.2 Advanced features

These should only be added after the core system works reliably:

- Localization-loss detection and recovery.
- IMU-assisted sensor fusion.
- Comparison of different SLAM configurations.
- Comparison of path-planning algorithms.
- Multiple destination waypoints.
- Delivery-task automation.
- Improved battery monitoring and energy-efficiency analysis.
- More advanced environmental sensing.
- Multi-robot coordination.

## 3.3 Explicit exclusions from the initial version

Do not include the following as mandatory initial requirements:

- Outdoor GPS navigation.
- Public-road operation.
- Heavy industrial payloads.
- Cloud-dependent control.
- Internet-based remote driving.
- Deep-learning-based object recognition.
- Multi-robot coordination.
- Industrial safety certification.

These may be reconsidered in a future project version if a concrete application justifies them.

---

# 4. MECHANICAL DESIGN DECISION

## 4.1 Two-wheel differential drive versus four-wheel drive

The number of motors has not been finalized.

### Option A: Two drive motors and a caster wheel

Advantages:

- Simpler differential-drive kinematics.
- Easier odometry calibration.
- Less wheel scrub during turns.
- Lower component count and power demand.
- Easier first implementation.

### Option B: Four-wheel skid-steer

Two motors on the left and two on the right, controlled as two independent sides.

Advantages:

- Potentially greater payload capacity and stability.
- More suitable if the intended chassis or available hardware requires four wheels.
- Can be useful for exploring skid-steer motion.

Disadvantages:

- More mechanical and electrical complexity.
- Additional encoder integration and calibration.
- More wheel scrub during turning, potentially reducing odometry accuracy.
- Greater power demand and possible need for additional motor drivers.

**Current recommendation:** begin with two independently controlled drive wheels and a caster unless the available components or application justify four-wheel skid-steer. Four-wheel drive remains an open design decision.

Use MATLAB or another suitable simulation tool to compare the alternatives before designing the chassis.

## 4.2 3D-printed mechanical components

The final chassis should be designed around the selected components.

Likely printed parts:

- Main chassis plates or body.
- Motor mounting brackets.
- LiDAR mounting tower.
- Electronics enclosure.
- Battery holder.
- Sensor mounts.
- Cable guides and protective covers.
- Optional payload platform.

Before printing, validate component clearances, mounting holes, LiDAR visibility, wheel geometry, centre of gravity and access for maintenance.

Fusion is the tentative first choice for mechanical CAD. FreeCAD is a possible alternative. AutoCAD may be useful for dimensioned drawings, but is not a required choice.

The exact software will be selected after confirming availability and suitability.

---

# 5. ULTRASONIC SENSORS AND SAFETY

## 5.1 Ultrasonic sensors

HC-SR04-class ultrasonic sensors are proposed as supplementary distance sensors.

Potential functions:

- Front obstacle-distance monitoring.
- Additional near-field obstacle detection.
- Supplementary collision-prevention logic.
- Comparison of ultrasonic and LiDAR distance readings.

Start with one front-facing sensor during basic testing. Add additional angled sensors only if testing shows that they improve coverage.

Check electrical compatibility with the selected controller. In particular, an HC-SR04 Echo output may require voltage-level adaptation when connected to a 3.3 V ESP32 input.

Ultrasonic sensors should not be treated as the primary SLAM sensor or the sole emergency-stop mechanism.

## 5.2 Required safety layers

1. **Physical safety:** accessible emergency-stop switch or appropriate motor-power cutoff.
2. **Control safety:** watchdog and safe-stop behaviour when commands become stale, communication is lost or a critical fault occurs.
3. **Obstacle response:** LiDAR-based obstacle handling with optional ultrasonic distance checks.
4. **Operational safety:** low-speed testing in a clear, controlled area.

Software obstacle detection and wireless communication are not substitutes for a physical emergency stop.

---

# 6. HARDWARE REQUIREMENTS AND PRELIMINARY BOM

All prices below are planning estimates in Indian rupees, not verified quotations. Actual procurement will depend on supplier availability, exact models, component compatibility and laboratory inventory.

| Component | Initial specification | Estimated cost |
|---|---|---:|
| Main computer | Raspberry Pi 5 or compatible Linux computer | ₹9,000–₹15,000 |
| 2D LiDAR | RPLIDAR A1M8 or compatible scanner | ₹8,000–₹14,000 |
| Drive motors | Two encoder-equipped geared DC motors | ₹1,000–₹3,000 |
| Motor driver | Driver rated for motor voltage and stall current | ₹500–₹2,500 |
| Microcontroller | ESP32 or compatible board | ₹300–₹700 |
| Ultrasonic sensors | 1–3 HC-SR04-class sensors | ₹150–₹450 |
| Power system | Battery, regulators, charger and protection | ₹2,000–₹4,000 |
| Mechanical parts | Filament, wheels, caster and fasteners | ₹500–₹1,500 |
| Safety and wiring | Switch, fuse, connectors and wiring | ₹500–₹1,000 |
| Contingency | Replacement parts and unforeseen expenses | ₹1,500–₹3,000 |

Indicative total for the two-motor design: approximately ₹23,450–₹45,150.

The actual expenditure may be substantially lower if the computer, motors, microcontroller, power equipment or sensors can be borrowed.

For a four-motor design, update the BOM for additional motors, encoders, driver capacity, mechanical parts and power demand before purchasing.

## 6.1 Procurement rules

Before buying components:

1. Inventory the IoT laboratory.
2. Record exact models and quantities.
3. Check motor voltage and stall current.
4. Check encoder output type and interface.
5. Verify LiDAR compatibility with the intended software stack.
6. Check battery requirements and voltage regulation.
7. Confirm the onboard computer's Linux and ROS 2 compatibility.
8. Confirm that required cables, power accessories and mounting hardware are included.
9. Update the BOM and total budget.
10. Purchase only after the core architecture is sufficiently defined.

---

# 7. SOFTWARE ARCHITECTURE

## 7.1 Main software components

### Operating system and robotics middleware

- Ubuntu Linux.
- ROS 2.
- RViz for visualizing sensor scans, maps, coordinate frames and navigation.
- Git and GitHub for version control.

### Mapping and navigation

- A suitable ROS 2-compatible LiDAR driver.
- Wheel-encoder interface and odometry.
- SLAM Toolbox or another compatible SLAM implementation.
- Nav2 or an equivalent navigation implementation.
- A compatible global planner, such as A*, where appropriate.
- A local obstacle-avoidance and motion-control system.

The final selection depends on the ROS 2 distribution, LiDAR driver, available software support and the physical robot's capabilities.

### Low-level controller

ESP32 or Arduino handles suitable time-critical motor and encoder operations. The main computer sends velocity or movement commands through a defined communication interface, such as USB serial.

The controller should report encoder measurements and relevant status information back to the main computer.

### Backend

A Python service using FastAPI or a similar framework.

Responsibilities:

- Connect the browser interface to the robotics software.
- Receive validated manual-control commands.
- Receive navigation-goal requests.
- Forward selected telemetry to the browser.
- Report robot status, errors and communication state.
- Implement communication timeouts and appropriate command validation.

### Frontend

A responsive browser-based web dashboard using HTML, CSS and JavaScript.

A separate native mobile application is not required initially.

## 7.2 Communication architecture

Primary communication: local Wi-Fi.

The dashboard should work on a laptop or phone connected to the same suitable local network as the robot.

Proposed communication methods:

- WebSockets for live telemetry and low-latency updates.
- HTTP endpoints for selected requests and configuration operations.
- ROS 2 communication between onboard robotics components.
- USB serial or another suitable wired link between the main computer and the low-level controller.

Bluetooth is optional and should only be added if a specific use case justifies it.

The initial design should not require cloud hosting or internet connectivity.

---

# 8. SYSTEM ARCHITECTURE — STEP BY STEP

## Layer 1: User interface

The user interacts with a web dashboard to:

- Select manual or autonomous mode.
- Drive the robot.
- Set a navigation destination.
- View the map and robot position.
- Monitor sensor information and connection status.
- Cancel a navigation task.
- View errors and logs.

## Layer 2: Backend and communication bridge

The backend receives requests from the dashboard and validates them before forwarding them to the robotics software.

Telemetry from the robotics software is processed and sent to the dashboard.

The backend must not bypass safety logic or provide unrestricted access to motor commands.

## Layer 3: Robotics software

ROS 2 connects sensor drivers, odometry, mapping, localization, planning, navigation and control.

### Mapping and localization

The robot receives LiDAR scans and motion estimates from wheel encoders. The SLAM system uses sensor observations and motion estimates to construct a map and estimate the robot's pose.

### Path planning

When a destination is selected, the global planner calculates a path through the available map.

### Navigation control

The navigation system follows the path and produces velocity commands. A local planner or controller responds to obstacles and adjusts the motion.

### Safety monitoring

The system monitors valid sensor data, command freshness, navigation state and communication status. It requests a safe stop when necessary.

## Layer 4: Onboard computer

A Raspberry Pi or compatible Linux computer runs the robotics software and backend.

The exact model will be chosen after compatibility testing and the laboratory inventory.

## Layer 5: Sensors and low-level controller

Sensors include:

- 2D LiDAR.
- Wheel encoders.
- Optional IMU.
- Optional ultrasonic sensors.

The ESP32 or Arduino reads suitable encoder signals and controls the motors through the motor driver.

## Layer 6: Actuators and feedback

The motors move the robot. Encoder measurements provide feedback about wheel movement, supporting odometry and motion control.

This creates a closed-loop process: sensor data informs the navigation system, the navigation system issues commands, and motor feedback helps estimate the resulting motion.

---

# 9. FULL DEVELOPMENT WORKFLOW

The project must be developed incrementally. Do not attempt to build the physical robot, SLAM system and web application simultaneously.

## PHASE 0 — Project organization and inventory

**Goal:** establish a reliable starting point.

Tasks:

1. Create the main GitHub repository.
2. Create a project folder structure.
3. Maintain the master project document.
4. Inspect available laboratory hardware.
5. Record component models and specifications.
6. Check the MATLAB installation and required products.
7. Record the exact 3D-printer models and supported materials.
8. Establish an initial budget.
9. Confirm the responsibilities of both teammates.

Deliverable: inventory, initial BOM, project structure and confirmed requirements.

## PHASE 1 — MATLAB environment verification

**Goal:** prepare the Windows-based MATLAB environment.

Current status: MATLAB R2026a Update 5 is installed. Only MATLAB and Control System Toolbox appeared in the `ver` output.

Required actions:

1. Check whether Robotics System Toolbox is installed.
2. Check whether Navigation Toolbox is installed.
3. Check whether ROS Toolbox is installed.
4. Install the required products through the authorized college license.
5. Verify required functions using `which`.
6. Run a small example from the relevant documentation.

Initial MATLAB checks:

`version`

`ver`

`which lidarSLAM -all`

`which differentialDriveKinematics -all`

`which occupancyMap -all`

`which controllerPurePursuit -all`

`which plannerAStarGrid -all`

Do not start the planned robotics experiments until the required products are confirmed.

Deliverable: working MATLAB environment and a record of installed products.

## PHASE 2 — Fundamentals of mobile robotics

**Goal:** understand the mathematics behind robot movement.

Topics to learn:

1. Cartesian coordinates and angles.
2. Robot pose: position and orientation.
3. Coordinate frames and transformations.
4. Differential-drive kinematics.
5. Wheel radius, wheel separation and encoder counts.
6. Wheel odometry.
7. Sensor measurements and measurement noise.
8. Occupancy-grid maps.
9. Basic path-planning concepts.
10. Difference between mapping, localization and SLAM.

Exercises:

- Calculate wheel movement from encoder counts.
- Simulate straight movement and turning.
- Plot the robot's trajectory.
- Investigate the effect of wheel-radius and wheel-separation errors.

Deliverable: working robot-motion simulation and written explanation of the equations.

## PHASE 3 — MATLAB simulation

**Goal:** build a virtual environment before physical construction.

Tasks:

1. Create a simple room or corridor layout.
2. Represent walls and obstacles.
3. Model a differential-drive robot.
4. Simulate robot movement.
5. Add simulated range measurements or LiDAR scans.
6. Implement or configure occupancy-grid mapping.
7. Test SLAM where supported.
8. Implement or configure global path planning.
9. Test path following.
10. Add static obstacles.
11. Add moving obstacles as a later experiment.
12. Record simulation outputs and performance metrics.

Deliverable: a simulated robot navigating a virtual environment.

Do not assume simulated success guarantees physical success. Real hardware introduces noise, wheel slip, latency, vibration and calibration errors.

## PHASE 4 — Linux and ROS 2 learning

**Goal:** establish the software architecture for the physical robot.

The team currently uses Windows and is learning Linux and ROS 2 from scratch.

Tasks:

1. Learn Linux terminal basics.
2. Learn filesystem, package installation and networking.
3. Choose a compatible Ubuntu and ROS 2 combination.
4. Decide whether to use WSL2 initially or a native Linux installation.
5. Install ROS 2 in the selected environment.
6. Learn nodes, topics, messages, services and actions.
7. Learn coordinate frames and transforms.
8. Learn launch files and parameter configuration.
9. Install and test RViz.
10. Run a basic simulated robot.
11. Introduce mapping and navigation packages.

WSL2 may be useful for initial learning, but simulator graphics, hardware drivers and some robotics integrations may require a native Linux environment.

Deliverable: a working ROS 2 environment and basic simulated robot.

## PHASE 5 — Mechanical design and component selection

**Goal:** design a printable chassis around real components.

Tasks:

1. Finalize the two-wheel versus four-wheel decision.
2. Select the motor and encoder combination.
3. Select the main computer and LiDAR.
4. Determine wheel dimensions and robot footprint.
5. Determine battery size and power requirements.
6. Choose suitable mechanical CAD software.
7. Model the chassis and component mounts.
8. Check clearances and sensor visibility.
9. Check mounting-hole alignment.
10. Consider weight distribution and centre of gravity.
11. Review print orientation, material and supports.
12. Print small test pieces before the complete chassis.
13. Assemble the mechanical prototype.
14. Update the BOM to reflect the actual build.

Deliverable: designed, fabricated and assembled chassis.

## PHASE 6 — Basic motor control and electrical integration

**Goal:** make the physical robot move reliably.

Tasks:

1. Test each motor independently.
2. Confirm motor-driver voltage and current ratings.
3. Implement forward and reverse control.
4. Implement left and right turning.
5. Read encoder measurements.
6. Calculate wheel movement.
7. Calibrate wheel radius and separation.
8. Implement basic speed control.
9. Verify low-speed movement and repeatability.
10. Integrate the battery and voltage regulators.
11. Add physical power cutoff and wiring protection.
12. Test communication between the main computer and microcontroller.

Deliverable: a robot that moves predictably and reports usable odometry.

## PHASE 7 — Sensor integration

**Goal:** obtain reliable sensor data.

Tasks:

1. Connect the LiDAR.
2. Install and test the compatible driver.
3. Display live scans in RViz.
4. Integrate wheel-encoder odometry.
5. Check coordinate-frame alignment.
6. Add an IMU if it is useful and available.
7. Integrate ultrasonic sensors if required.
8. Check measurement stability and noise.
9. Confirm timestamp and communication behaviour.
10. Record known sensor limitations.

Deliverable: live sensor data with consistent coordinate frames and usable odometry.

## PHASE 8 — SLAM and localization

**Goal:** construct and use a map of the physical environment.

Tasks:

1. Configure the robot description and coordinate frames.
2. Integrate LiDAR scans and odometry.
3. Run the selected SLAM implementation.
4. Drive the robot through a controlled environment.
5. Generate a 2D map.
6. Compare the map with a measured reference layout.
7. Investigate drift and map distortion.
8. Tune relevant parameters.
9. Save the generated map.
10. Test localization against the saved map.
11. Record mapping and localization performance.

Deliverable: a physical robot that can build a usable map and estimate its position.

## PHASE 9 — Autonomous navigation

**Goal:** reach destinations without continuous manual steering.

Tasks:

1. Configure a global planner.
2. Configure a suitable local planner or controller.
3. Set appropriate robot dimensions and obstacle clearances.
4. Define a safe speed range.
5. Select a destination on the map.
6. Generate a planned route.
7. Execute the route.
8. Monitor localization and motion.
9. Detect obstacles and stop when necessary.
10. Implement or configure route replanning.
11. Test multiple destinations.
12. Record successful and failed trials.

Deliverable: autonomous goal-based navigation with safe obstacle response.

## PHASE 10 — Web backend and frontend

**Goal:** make the robot usable through a live browser interface.

This phase can begin earlier with mock data, but physical integration should occur after the basic robot and sensor interfaces are reliable.

### Backend tasks

1. Create a Python project.
2. Choose FastAPI or a suitable alternative.
3. Establish the ROS 2 communication bridge.
4. Define the telemetry data format.
5. Create WebSocket connections for live updates.
6. Validate movement commands.
7. Implement mode management.
8. Add connection timeouts.
9. Implement safe handling of stale commands.
10. Expose selected robot status and logs.

### Frontend tasks

1. Build a responsive dashboard.
2. Display the robot connection status.
3. Add manual movement controls.
4. Add a speed setting within safe limits.
5. Display map and robot position.
6. Display sensor data.
7. Add goal selection.
8. Display navigation progress.
9. Add stop and cancel controls.
10. Add error messages and event logs.

### Integration tasks

1. Connect the frontend and backend.
2. Connect the backend to ROS 2.
3. Test communication on the local network.
4. Verify manual command handling.
5. Verify autonomous-goal requests.
6. Test loss of Wi-Fi.
7. Test backend restart.
8. Confirm that communication loss cannot leave the motors moving indefinitely.

Deliverable: browser-based live monitoring, manual driving and autonomous-goal control.

## PHASE 11 — Safety, testing and validation

**Goal:** demonstrate reliable behaviour, not just a successful one-time run.

Tasks:

1. Test the physical emergency stop.
2. Test command timeouts.
3. Test safe stopping after communication loss.
4. Test sensor failure handling.
5. Test obstacle detection.
6. Test manual-to-autonomous transitions.
7. Test goal cancellation.
8. Run repeated navigation trials.
9. Measure mapping and localization errors.
10. Measure navigation time and distance.
11. Compare planned and actual routes.
12. Record limitations and unresolved issues.

Deliverable: test report with quantitative results.

## PHASE 12 — Portfolio and future extensions

**Goal:** make the project understandable and demonstrable to others.

Deliverables:

- GitHub repository.
- Architecture and flowcharts.
- MATLAB simulation files.
- ROS 2 packages and configurations.
- CAD files and printable parts.
- Wiring diagrams and BOM.
- Backend and frontend code.
- Test data and performance plots.
- Project documentation.
- Demonstration video.
- Explanation of technical decisions and limitations.

After the core system is complete, select one advanced direction such as localization recovery, algorithm comparison or improved dynamic obstacle handling.

---

# 10. PROPOSED TEAM RESPONSIBILITIES

These are primary ownership areas, not isolated responsibilities.

## Person A — Hardware and integration

- Component inventory.
- Electrical design.
- Motor and encoder interfacing.
- Power system.
- Mechanical CAD and 3D printing.
- Sensor installation.
- Physical robot calibration.
- Hardware troubleshooting.

## Person B — Robotics software and simulation

- MATLAB experiments.
- Robot kinematics and motion modelling.
- Linux and ROS 2 setup.
- SLAM and localization.
- Path planning and navigation.
- Simulation and performance analysis.
- Software configuration and test scripts.

## Shared responsibilities

Both teammates should participate in:

- Project scope and architecture decisions.
- Backend/frontend design.
- Hardware-software integration.
- Safety testing.
- GitHub and documentation.
- Validation and demonstration.
- Advanced-feature selection.

The roles may be adjusted based on interests and progress.

---

# 11. PRELIMINARY TIMELINE

There is no fixed submission deadline. The following durations are planning estimates, and phases may overlap.

| Phase | Indicative duration |
|---|---|
| Project organization and inventory | 1–2 weeks |
| MATLAB environment and fundamentals | 2–4 weeks |
| MATLAB simulation | 3–5 weeks |
| Linux and ROS 2 learning | 3–6 weeks |
| Mechanical design and fabrication | 2–3 weeks |
| Motor control and sensor integration | 2–4 weeks |
| SLAM and localization | 3–6 weeks |
| Autonomous navigation | 2–4 weeks |
| Backend and frontend | 2–4 weeks |
| Validation and documentation | 2–3 weeks |

These estimates should not be added mechanically because several phases can run in parallel. Hardware delivery, debugging and the learning curve may extend the overall project.

The objective is to complete milestones in the correct order rather than meet an arbitrary date.

---

# 12. EVALUATION AND ACCEPTANCE CRITERIA

The robot must be evaluated using repeatable tests.

## Mapping

- Compare the generated map with a measured reference floor plan.
- Document map distortion and missing features.

## Localization

- Compare estimated position against known reference positions.
- Record average and maximum position error.

## Navigation

- Record successful goal arrivals.
- Measure travel time and actual distance.
- Compare actual distance with planned path length.

## Obstacle response

- Test controlled obstacle placements.
- Record safe stops, replanning events and collisions.
- Distinguish static from moving-obstacle tests.

## Web dashboard

- Measure responsiveness of telemetry.
- Test manual command handling.
- Verify correct mode transitions.
- Test connection loss and recovery.

## Example initial acceptance target

At least 9 successful goal arrivals in 10 controlled trials, with zero collisions during those trials.

This is a proposed target for evaluation, not an assumed result. Adjust it after baseline tests.

---

# 13. MAJOR RISKS AND MITIGATION

| Risk | Mitigation |
|---|---|
| Required MATLAB toolboxes are not installed | Verify and install them before beginning toolbox-dependent experiments |
| Linux/ROS 2 learning takes longer than expected | Start with small tutorials and a basic simulated robot |
| LiDAR compatibility issues | Verify model, driver and ROS 2 support before purchase |
| Odometry drift | Calibrate wheel dimensions and encoder measurements |
| Four-wheel wheel scrub | Compare skid-steer and differential drive before locking the chassis |
| Insufficient computer performance | Benchmark the selected software and computer before final procurement |
| Power instability | Calculate current requirements and test regulators under load |
| Web commands create unsafe motion | Implement watchdogs, command limits and physical emergency stop |
| Integration becomes too complex | Validate each subsystem independently |
| Budget increases | Reuse lab equipment and approve purchases against the BOM |
| Printed chassis needs repeated redesign | Validate dimensions and print small test components first |

---

# 14. IMMEDIATE NEXT STEPS

Current stage: MATLAB environment verification.

1. Confirm whether Robotics System Toolbox and Navigation Toolbox are installed.
2. Install required products through the college license if available.
3. Verify the required functions.
4. Create the project directory and GitHub repository.
5. Start the first MATLAB experiment: a differential-drive robot moving in a simple environment.
6. Learn pose, wheel motion and coordinate frames.
7. Progress to occupancy grids and path planning.
8. Inventory the IoT laboratory components in parallel.
9. Defer final hardware procurement and chassis design until component compatibility and the basic simulation are understood.

The next milestone is **a working simulated robot that moves to a target**, not the complete physical robot.

---

# 15. FUTURE CHATGPT CONTEXT AND WORKING RULES

When continuing this project in a future conversation, use this document as the primary project reference.

Important rules:

1. Do not treat the project as a college assignment with a fixed deadline.
2. Teach concepts before providing implementation details.
3. Prefer incremental, beginner-friendly implementation and testing.
4. Do not assume the robotics toolboxes are installed until verified.
5. Do not assume the exact hardware, motor count, LiDAR model or budget is finalized.
6. Do not purchase or recommend hardware without considering lab inventory and compatibility.
7. Keep simulation and physical validation clearly distinguished.
8. Maintain the agreed separation between robotics software, low-level motor control, backend and frontend.
9. Prioritize local Wi-Fi and a browser-based dashboard; Bluetooth and cloud services are optional.
10. Treat ultrasonic sensors as supplementary distance sensors, not the sole emergency-stop system.
11. Require a physical safety cutoff and appropriate software safeguards.
12. Do not expand the project with advanced features until the core scope works reliably.
13. Update the project plan as decisions are made and mark each decision as tentative or finalized.
14. Record tested results rather than claiming unverified performance.
15. Help the team understand the design well enough to explain and defend it independently.

## Current decision status

**Agreed in principle:** project purpose, indoor autonomous navigation application, SLAM-based approach, web dashboard, simulation-first development, use of available lab resources, and long-term portfolio objective.

**Not finalized:** exact computer, LiDAR model, motor count, chassis geometry, battery, budget, Linux/ROS 2 environment and exact software configurations.

**Latest verified technical status:** MATLAB R2026a Update 5 on Windows 11; `ver` lists MATLAB and Control System Toolbox; the selected robotics and ROS functions were not found in the current environment.

**Next action:** verify the required MATLAB products and start the first simulation milestone.