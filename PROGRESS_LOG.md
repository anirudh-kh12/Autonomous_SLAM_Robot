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

### 2026-10-15 — Experiment 03: Robot pose and coordinate frames

**Objective:** Simulate robot movement, visualize heading and transform a point from robot coordinates into world coordinates.

**File:** `simulation/MATLAB/exp03_robot_pose_frames.m`

**Work completed**
- Simulated straight motion followed by left and right turns.
- Plotted the robot trajectory with heading arrows.
- Implemented a 2D rotation matrix and coordinate transformation.
- Visualized the robot pose and transformed point.

**Results**
- MATLAB script executed successfully.
- Robot pose was represented using X position, Y position and heading angle.
- For a robot at (2, 1), facing 90 degrees, a point at local coordinates (1, 0) was transformed to world coordinates (2, 2).
- The calculated world coordinates matched the expected result.

**Learning outcomes**
- Understood the difference between world and robot coordinate frames.
- Learned how robot orientation affects the position of a locally measured point.
- Understood why coordinate transformations are essential for interpreting LiDAR measurements and building maps.

**Limitations**
- The experiment uses ideal motion and a manually specified point rather than real sensor measurements.

**Status:** Completed — pose visualization and coordinate transformation verified.

**Next milestone:** Experiment 04 — Occupancy grids and obstacle representation.

### 2026-10-15 — Experiment 04: Occupancy grid mapping

**Objective:** Create a 2D occupancy grid and represent walls and obstacles in a simulated indoor environment.

**File:** `simulation/MATLAB/exp04_occupancy_grid.m`

**Parameters**
- Room width: 10 m
- Room height: 8 m
- Map resolution: 10 cells per metre
- Approximate grid size: 8,000 cells
- Interior obstacles: Two rectangular regions

**Results**
- MATLAB script executed successfully.
- Displayed the room boundary walls and interior obstacles.
- Marked the starting position at (1, 1) and goal at (8.5, 6.5).
- Confirmed the map dimensions, resolution and cell count in the Command Window.

**Learning outcomes**
- Understood occupancy grids and their use in representing environments.
- Learned how map resolution determines cell size.
- Represented free space and occupied regions using MATLAB's occupancyMap.
- Distinguished a manually constructed map from a map generated by SLAM.

**Status:** Completed — occupancy grid visualization verified.

**Next milestone:** Experiment 05 — A* path planning through the occupancy grid.

### Experiment 05 — A* Path Planning
- Implemented A* path planning in MATLAB using a binary occupancy grid.
- Created a simulated room with boundary walls and two rectangular obstacles.
- Planned a route from start position (1.0, 1.0) m to goal position (8.5, 6.5) m.
- Successfully generated a path containing 86 waypoints with an approximate length of 10.364 m.
- Validated the path against occupied grid cells; validation passed.
- Status: **Completed and tested.**

### Experiment 07 — Pure Pursuit Path Following
- Implemented a Pure Pursuit controller for a simulated differential-drive robot in MATLAB.
- Defined a desired path using waypoints and interpolated it for target selection.
- Calculated path curvature and converted the resulting angular velocity into left- and right-wheel velocities.
- Simulated robot motion using differential-drive kinematics.
- Successfully reached the goal in 57.60 seconds.
- Recorded a mean path tracking error of 0.018 m and a maximum error of 0.060 m.
- Used a lookahead distance of 0.35 m and a forward speed of 0.15 m/s.
- Status: **Completed and tested.**

### Experiment 08 — Pure Pursuit Lookahead Comparison
- Compared Pure Pursuit lookahead distances of 0.20 m, 0.35 m and 0.60 m in MATLAB.
- All three simulations successfully reached the goal.
- The 0.20 m lookahead produced the lowest mean tracking error of approximately 0.0141 m.
- The 0.35 m lookahead produced a mean tracking error of approximately 0.0183 m and completed the route in 57.6 seconds.
- The 0.60 m lookahead completed the route fastest, in approximately 57.0 seconds, but produced the largest maximum tracking error of approximately 0.1245 m.
- Observed the trade-off between tracking accuracy, corner-cutting and completion time.
- Status: **Completed and tested.**

### Experiment 09 — A* Planning and Pure Pursuit Integration
- Integrated A* path planning and Pure Pursuit path following into one MATLAB simulation.
- Generated a route through an occupancy grid containing two obstacles and boundary walls.
- Inflated obstacles using a 0.20 m robot radius and a 0.10 m safety margin.
- Implemented progress-tracked target selection, angular velocity limiting and sampled movement-segment collision checking.
- Tuned the lookahead distance to 0.10 m after larger tested values caused collisions near the first obstacle.
- Successfully reached the goal without a detected collision.
- Simulation time: 70.05 seconds.
- Mean tracking error: 0.032 m; maximum tracking error: 0.073 m.
- Status: **Completed in simulation.**

### Experiment 10 — Navigation Robustness and Obstacle Clearance
- Tested the integrated A* and Pure Pursuit navigation system on three different start-goal routes.
- All three routes reached their goals without a detected collision.
- Test 1: 70.05 s completion time, 10.508 m trajectory length and 0.0320 m mean tracking error.
- Test 2: 65.45 s completion time, 9.8175 m trajectory length and 0.0365 m mean tracking error.
- Test 3: 54.45 s completion time, 8.1675 m trajectory length and 0.0299 m mean tracking error.
- Measured approximate obstacle clearance using distances from robot positions to occupied grid-cell centres.
- Observed route-dependent variation in tracking error.
- Status: **Completed in simulation; physical footprint clearance remains to be validated.**

### Experiment 11 — Robot Footprint and Swept-Path Validation
- Modelled the robot as a rectangular footprint measuring 0.40 m × 0.30 m.
- Inflated obstacles for A* planning using the footprint half-diagonal plus a 0.10 m safety margin, giving a planning clearance of approximately 0.350 m.
- Added footprint collision checks at interpolated robot poses along the movement trajectory.
- Successfully reached the goal without a detected footprint collision.
- Recorded a trajectory length of 10.508 m, mean tracking error of 0.032 m and maximum tracking error of 0.073 m.
- Status: **Completed in simulation; collision checking remains an approximation.**

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