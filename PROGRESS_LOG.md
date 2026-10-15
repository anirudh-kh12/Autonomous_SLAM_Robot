# Progress Log — Autonomous SLAM Robot

This file records actual project progress. Update it after meaningful work sessions, experiments, installations, tests and design changes.

## Current Status

- **Project phase:** Phase 0 — Project organization and inventory
- **Current milestone:** MATLAB robotics environment setup
- **Last updated:** 2026-10-14
- **MATLAB version:** R2026a Update 5
- **Operating system:** Windows 11 Pro Insider Preview
- **MATLAB installation:** MATLAB and Control System Toolbox were listed by `ver`. Robotics System Toolbox and Navigation Toolbox installation is in progress.
- **ROS 2:** Not yet installed or configured.
- **Physical robot:** Not yet assembled.
- **Web dashboard:** Not yet implemented.

## Session Log

### 2026-10-14 — Initial project setup

**Work completed**
- Agreed on the autonomous indoor SLAM mobile robot project direction.
- Defined warehouse material transport as the primary application.
- Agreed to use simulation before physical construction.
- Agreed to develop a web dashboard for live monitoring, manual driving and autonomous navigation.
- Identified MATLAB, ROS 2, Python, browser technologies and mechanical CAD as candidate tools.
- Confirmed access to campus MATLAB toolboxes, two 3D printers and some borrowable IoT lab components.
- Created the initial project scope, architecture and implementation roadmap.

**Environment checks**
- MATLAB R2026a Update 5 is installed.
- `ver` listed MATLAB and Control System Toolbox.
- Initial checks for `lidarSLAM`, `differentialDriveKinematics`, `occupancyMap`, `controllerPurePursuit`, `plannerAStarGrid` and `ros2node` returned `not found`.

**Problems or pending work**
- Required MATLAB robotics products must be installed and verified.
- Lab hardware inventory has not yet been completed.
- Exact motor configuration, onboard computer, LiDAR model and budget remain undecided.
- Linux and ROS 2 learning has not started.

**Next actions**
1. Finish MATLAB toolbox installation.
2. Verify required MATLAB functions.
3. Create the project folder structure.
4. Create and populate the project documentation files.
5. Inspect the IoT lab inventory.
6. Begin the first differential-drive simulation.

### 2026-10-14 — MATLAB toolbox verification

**Work completed**
- Installed or enabled the required MATLAB robotics products.
- Verified that MATLAB R2026a Update 5 lists Robotics System Toolbox, Navigation Toolbox and ROS Toolbox.
- Verified that the following functions resolve to file paths:
  - `lidarSLAM`
  - `differentialDriveKinematics`
  - `occupancyMap`
  - `controllerPurePursuit`
  - `plannerAStarGrid`

**Result**
- MATLAB environment is ready for initial robotics experiments.
- Function discovery has been verified, but successful execution of a complete simulation is still pending.

**Next actions**
1. Finish local Git setup.
2. Create the GitHub repository.
3. Record the first commit.
4. Begin the first MATLAB differential-drive simulation.

### 2026-10-14 — GitHub repository setup

**Work completed**
- Created the project directory and documentation structure.
- Initialized local Git version control.
- Created the GitHub repository.
- Confirmed that the repository is visible on GitHub.
- Prepared the repository for collaboration with the second teammate.

**Result**
- Project documentation and version control are established.
- Teammate invitation is planned; collaboration becomes active after the invitation is accepted.

**Next actions**
1. Begin the first MATLAB simulation.
2. Understand differential-drive robot motion.
3. Record simulation results after testing.
4. Inspect available IoT laboratory hardware.

### 2026-10-14 — Experiment 01: Differential-drive basics

**Objective:** Simulate straight-line motion and turning using differential-drive kinematics.

**File:** `simulation/MATLAB/exp01_differential_drive_basics.m`

**Parameters**
- Wheelbase: 0.18 m
- Time step: 0.05 s
- Simulation duration: 10 s
- First 5 seconds: both wheels move at 0.15 m/s
- Remaining 5 seconds: left wheel at 0.08 m/s and right wheel at 0.15 m/s

**Results**
- MATLAB script executed successfully.
- Generated robot trajectory and orientation-versus-time plots.
- Final X position: approximately 1.029 m.
- Final Y position: approximately 0.401 m.
- Final orientation: approximately 111.41 degrees.

**Observations**
- Equal wheel velocities produce straight-line motion.
- Unequal wheel velocities produce a curved trajectory.
- The robot's pose is represented by X position, Y position and orientation.
- The model uses ideal differential-drive kinematics and does not yet include real-world sensor noise, wheel slip or motor dynamics.

**Status:** Completed — initial mathematical simulation verified.

**Next actions**
1. Investigate the effect of changing wheel velocities and wheelbase.
2. Learn how robot pose is represented and updated.
3. Progress to occupancy-grid mapping and path planning after completing the motion-model experiments.

### 2026-10-15 — Experiment 02: Effect of wheel velocities

**Objective:** Compare differential-drive motion under four combinations of left and right wheel velocities.

**File:** `simulation/MATLAB/exp02_wheel_speed_comparison.m`

**Cases tested**
1. Equal forward wheel velocities for straight motion.
2. Right wheel faster than left for a left turn.
3. Left wheel faster than right for a right turn.
4. Equal and opposite wheel velocities for rotation in place.

**Results**
- MATLAB script executed successfully.
- Plotted the trajectories of all four motion cases.
- Plotted robot orientation against time.
- Observed that equal wheel speeds produce zero angular velocity.
- Observed that unequal wheel speeds produce curved trajectories.
- Observed that equal and opposite wheel speeds produce rotation in place in the ideal model.

**Learning outcomes**
- Understood the relationship between wheel-speed difference and angular velocity.
- Observed how wheel-speed combinations affect turning direction.
- Learned that accumulated orientation may exceed the conventional -180 to +180 degree range.
- Confirmed the basic differential-drive motion equations through simulation.

**Limitations**
- The simulation uses ideal kinematics without motor dynamics, wheel slip or sensor noise.

**Status:** Completed — four motion cases simulated successfully.

**Next actions**
1. Investigate the effect of wheelbase on turning.
2. Understand robot pose and coordinate frames.
3. Progress toward occupancy-grid mapping and path planning.

## Future Session Template

### YYYY-MM-DD — Session title

**Objective**
What were we trying to achieve?

**Work completed**
- 

**Commands, files or components used**
- 

**Tests performed**
- 

**Results**
- 

**Problems encountered**
- 

**Decisions made**
- 

**Next actions**
1. 
2. 
3. 

**Evidence**
Screenshots, plots, test logs, Git commit ID or other relevant references.

---

## Logging Rules

- Record only work that was actually completed.
- Distinguish between planned, attempted, successful and failed work.
- Include important error messages and test results.
- Record component models, versions and calibration values when relevant.
- Update the current status when a milestone changes.
- Do not delete failed experiments; record what was learned from them.
- Keep passwords, access tokens, Wi-Fi credentials and private keys out of the repository.