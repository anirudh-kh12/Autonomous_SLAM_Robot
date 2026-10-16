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

### Experiment 12 — Simulated LiDAR and Sensor-Based Mapping
- Created a 10 m × 8 m ground-truth occupancy map with boundary walls and two rectangular obstacles.
- Simulated a 360-degree LiDAR with 181 beams per pose, using seven known robot poses.
- Recorded 1,213 LiDAR returns and constructed a basic occupancy-evidence grid.
- Generated figures for the ground-truth map, LiDAR returns, reconstructed occupancy grid, and ground-truth comparison.
- Total grid cells: 8,000; observed free cells: 7,010; observed occupied cells: 481; unknown cells: 509.
- Observed cell coverage: 93.64%.
- Observed-cell agreement: 89.87%.
- Occupied-cell precision: 73.18%.
- Occupied-cell recall among observed ground-truth occupied cells: 35.85%.
- Ground-truth occupied-cell coverage: 99.49% (982 of 987 cells observed).
- **Status: Completed as a basic simulated mapping experiment.** Mapping is not yet probabilistic SLAM, and obstacle-cell classification remains limited by the simplified ray-marching approach.


### 2026-10-16 — Experiment 13: Probabilistic LiDAR Occupancy Mapping

**Objective**
Implement a probabilistic occupancy grid using simulated LiDAR scans and log-odds evidence updates, then compare the reconstructed map with ground truth.

**Work completed**
- Created `exp13_probabilistic_lidar_mapping.m`.
- Generated simulated 2D LiDAR scans from seven predefined robot poses.
- Implemented log-odds updates for free-space and occupied-cell evidence.
- Converted log-odds values into occupancy probabilities using the logistic function.
- Classified cells as free, occupied, uncertain or unknown using probability thresholds.
- Generated five figures showing the ground-truth map, LiDAR returns, probability grid, classified grid and reconstruction comparison.

**Commands, files or components used**
- MATLAB R2026b.
- `simulation/MATLAB/exp13_probabilistic_lidar_mapping.m`
- `binaryOccupancyMap`, `setOccupancy`, `checkOccupancy`, `occupancyMatrix`.

**Tests performed**
- Ran the complete simulation.
- Checked the number of LiDAR returns and occupancy-grid statistics.
- Compared reconstructed classifications with ground-truth occupancy.

**Results**
- Map dimensions: 10 m × 8 m.
- Grid resolution: 10 cells/m; 8,000 total cells.
- Robot poses: 7.
- Total LiDAR returns: 1,213.
- Observed cells: 7,491; unknown cells: 509.
- Observed coverage: 93.64%.
- Agreement on definitely classified cells: 89.88%.
- Occupied-cell precision: 73.38%.
- Occupied-cell recall: 35.68%.
- Ground-truth occupied-cell coverage: 99.49%.
- The two rectangular obstacles were reconstructed visibly, although some boundary cells were misclassified.

**Problems encountered**
- Occupied-cell recall was relatively low because some observed obstacle cells did not cross the occupied classification threshold.
- The simplified ray-casting and cell-update model produces imperfect obstacle boundaries.
- The high ground-truth occupied-cell coverage and lower occupied-cell recall measure different things.

**Decisions made**
- Retain the experiment as a baseline for probabilistic occupancy mapping.
- Treat the current ray-casting implementation as a simplified simulation rather than a complete sensor model.

**Next actions**
1. Continue with LiDAR scan matching and relative pose estimation.
2. Explore mapping accuracy improvements in a future experiment.
3. Commit and push the validated experiment to GitHub.



### 2026-10-16 — Experiment 14: LiDAR Scan Matching and Relative Pose Estimation

**Objective**
Estimate relative translation and rotation between two simulated LiDAR scans using a 2D Iterative Closest Point (ICP) algorithm, then compare the estimated movement with ground truth.

**Work completed**
- Created `exp14_lidar_scan_matching.m`.
- Generated two simulated LiDAR scans from different robot poses in the same environment.
- Converted LiDAR hit points from world coordinates into each robot's local coordinate frame.
- Implemented iterative closest-point matching using nearest-neighbour correspondences, outlier rejection, centroid alignment and singular value decomposition (SVD).
- Generated figures showing scans before and after alignment.
- Compared estimated relative translation and rotation against the known simulated movement.

**Commands, files or components used**
- MATLAB R2026b.
- `simulation/MATLAB/exp14_lidar_scan_matching.m`
- `binaryOccupancyMap`, `checkOccupancy`, `svd`, `atan2`, `deg2rad`.

**Tests performed**
- Ran the complete scan-matching simulation.
- Checked that both scans contained sufficient LiDAR points.
- Examined the before-matching and after-matching plots.
- Compared the estimated relative pose with the known relative pose.

**Results**
- Scan 1 points: 339.
- Scan 2 points: 338.
- ICP iterations: 18.
- Actual relative translation: (0.2143, 0.0638) m.
- Estimated relative translation: (0.2080, 0.0698) m.
- Translation estimation error: 0.0087 m.
- Actual relative rotation: 5.000°.
- Estimated relative rotation: 4.833°.
- Rotation estimation error: 0.167°.
- Reported final mean nearest-neighbour distance: 0.0232 m.
- The aligned scan plot showed close overlap between the reference scan and the transformed second scan.

**Problems encountered**
- No execution error was observed.
- ICP is sensitive to initialization, correspondence selection and environmental geometry.
- The reported mean nearest-neighbour distance should be recalculated after the final transformation for a more rigorous residual measurement.

**Decisions made**
- Use the custom 2D ICP implementation as a learning baseline for relative pose estimation.
- Treat the results as controlled simulation results, not a guarantee of equivalent real-world accuracy.
- Further validate the final alignment residual before using scan matching in a larger SLAM pipeline.

**Next actions**
1. Validate the final ICP residual using correspondences recomputed after the last transformation.
2. Test scan matching with different pose changes and initial offsets.
3. Explore integrating estimated relative motion into an incremental mapping pipeline.
4. Commit and push the validated experiment to GitHub.


## 2026-10-16 — Experiment 15: ICP Scan Matching Robustness Validation

**Objective**

Evaluate the robustness of Iterative Closest Point (ICP) scan matching under different initial pose-estimation errors.

**Work completed**
- Created a simulated 10 m × 8 m occupancy map with boundary walls and two obstacles.
- Generated LiDAR scans from two nearby robot poses.
- Implemented ICP alignment using nearest-neighbour correspondences and rigid transformation estimation.
- Compared three initial alignment conditions: accurate, moderate error, and larger error.
- Calculated translation error, rotation error, final mean residual, iteration count, and convergence status.
- Generated figures showing scans before matching, aligned scans, and translation and rotation errors.

**Commands, files or components used**
- `simulation/MATLAB/exp15_icp_robustness.m`
- MATLAB `binaryOccupancyMap`, `checkOccupancy`, `pdist2`, and singular value decomposition (`svd`).

**Tests performed**
- Test 1: Accurate initial estimate.
- Test 2: Moderate initial error.
- Test 3: Larger initial error.

**Results**
- All three tests reported convergence.
- Accurate initial estimate: 5 iterations, 0.0101 m translation error, 0.3219° rotation error, and 0.0384 m final mean residual.
- Moderate initial error: 13 iterations, 0.0119 m translation error, 0.3491° rotation error, and 0.0385 m final mean residual.
- Larger initial error: 9 iterations, 0.0335 m translation error, 1.8425° rotation error, and 0.0647 m final mean residual.
- The larger initial error produced the least accurate alignment and the highest residual.
- Results demonstrate that ICP accuracy depends on the initial pose estimate, even when the algorithm reports convergence.

**Problems encountered**
- No execution-stopping errors were shown in the supplied results.
- ICP convergence did not imply identical accuracy across all tests.

**Decisions made**
- Use this experiment as a baseline for understanding ICP sensitivity to initial pose estimates.
- Treat convergence status and alignment accuracy as separate evaluation criteria.

**Next actions**
1. Update `TODO.md` to mark Experiment 15 complete and add Experiment 16 as the next task.
2. Continue with Experiment 16 after reviewing the next experiment's objective.
3. Commit and push the completed experiment and documentation updates to GitHub.

## 2026-10-16 — Experiment 16: ICP with Noise and Outliers

**Objective**

Evaluate the robustness of Iterative Closest Point (ICP) scan matching under clean measurements, Gaussian measurement noise, and additional outlier points.

**Work completed**
- Created and executed `exp16_icp_noise_outliers.m`.
- Generated two simulated LiDAR scans from known robot poses.
- Compared clean scans, scans with Gaussian noise, and scans with noise plus outliers.
- Implemented nearest-neighbour correspondence matching, correspondence-distance filtering, and iterative rigid transformation estimation.
- Visualized the scans before matching, aligned scans, and translation and rotation errors.

**Commands, files or components used**
- `simulation/MATLAB/exp16_icp_noise_outliers.m`
- MATLAB `binaryOccupancyMap` and `checkOccupancy`
- MATLAB matrix operations, SVD, and `pdist2`

**Tests performed**
- Test 1: Clean scan.
- Test 2: Scan with Gaussian noise (standard deviation 0.02 m).
- Test 3: Noisy scan with 17 additional outlier points.
- Compared convergence, iteration count, translation error, rotation error, and final mean residual.

**Results**
- All three tests converged.
- Clean scan: 11 iterations, 0.0119 m translation error, 0.3491° rotation error, and 0.0385 m final residual.
- Noisy scan: 13 iterations, 0.0122 m translation error, 0.3266° rotation error, and 0.0435 m final residual.
- Noisy scan with outliers: 10 iterations, 0.0145 m translation error, 0.3268° rotation error, and 0.0451 m final residual.
- Measurement noise and outliers increased the final residual, while translation error increased modestly.
- The experiment demonstrated that ICP can converge despite imperfect scan measurements, but convergence alone does not guarantee perfect alignment.

**Problems encountered**
- No execution-blocking errors were observed.
- The impact of outliers was modest in this setup; stronger outlier rejection and more varied test cases could be explored later.

**Decisions made**
- Keep this experiment as the baseline robustness test for ICP.
- Treat convergence, transformation error, and residual as separate performance measures.

**Next actions**
1. Review the experiment code and commit the completed script to Git.
2. Update `TODO.md` to mark Experiment 16 complete.
3. Proceed to Experiment 17, focusing on scan matching or mapping with a more challenging scenario.

## 2026-10-16 — Experiment 17: Robust ICP with Outlier Rejection

**Objective**

Compare standard ICP with a robust ICP variant to evaluate the effect of correspondence filtering and residual trimming on LiDAR scan alignment in the presence of noise and outliers.

**Work completed**
- Generated a reference scan containing an asymmetric arrangement of wall segments.
- Created a transformed source scan with Gaussian measurement noise and 35 additional outlier points.
- Implemented standard ICP and robust ICP using nearest-neighbour correspondence matching.
- Added correspondence-distance filtering and residual trimming to the robust method.
- Generated plots showing the scans before alignment, aligned scan comparison, and accuracy metrics.

**Commands, files or components used**
- `simulation/MATLAB/exp17_robust_icp_outlier_rejection.m`
- MATLAB matrix operations, SVD, `knnsearch`, and rigid transformation calculations.

**Tests performed**
- Standard ICP on the noisy source scan with outliers.
- Robust ICP on the same source scan with outliers.
- Compared convergence, iteration count, translation error, rotation error, and final mean residual.

**Results**
- Both methods converged.
- Standard ICP: 22 iterations, 0.0024 m translation error, 0.0300° rotation error, and 0.0346 m final mean residual.
- Robust ICP: 45 iterations, 0.0006 m translation error, 0.0250° rotation error, and 0.0272 m final mean residual.
- Robust ICP reduced translation error by approximately 75% and final mean residual by approximately 21.4%.
- Robust ICP achieved better accuracy in this test but required more iterations.
- Some outlier points remained visible after alignment, indicating that transformation estimation and outlier removal are separate concerns.

**Problems encountered**
- No execution-blocking errors were observed.
- Robust ICP required more iterations than standard ICP.
- The experiment used synthetic scan geometry and does not establish performance on real LiDAR data.

**Decisions made**
- Retain robust correspondence filtering and residual trimming as promising techniques for future scan-matching tests.
- Evaluate accuracy and computational cost together rather than judging performance from convergence alone.

**Next actions**
1. Save and commit the Experiment 17 script.
2. Update `TODO.md` to mark Experiment 17 complete.
3. Continue with Experiment 18, building toward reliable scan matching and localization.

## 2026-10-16 — Experiment 18: LiDAR Odometry

**Objective**  
Estimate robot motion between consecutive simulated LiDAR scans and compare the estimated trajectory with ground-truth motion.

**Work completed**
- Implemented consecutive LiDAR scan generation and scan matching using ICP.
- Estimated relative translation and rotation between consecutive scans.
- Compared estimated motion against ground-truth relative motion.
- Plotted the ground-truth and estimated odometry trajectories.
- Investigated and corrected a coordinate-frame/sign convention error in the relative-motion calculation.
- Reran the experiment and verified the corrected results.

**Commands, files or components used**
- `simulation/MATLAB/exp18_lidar_odometry.m`
- MATLAB simulation and plotting tools
- ICP scan matching and relative-pose estimation

**Tests performed**
- Evaluated 5 consecutive scan pairs.
- Compared relative translation and rotation estimates with ground truth.
- Checked accumulated position and heading errors.
- Visually inspected the trajectory overlay and error plots.

**Results**
- Mean relative translation error: 0.0022 m.
- Maximum relative translation error: 0.0052 m.
- Mean relative rotation error: 0.0207°.
- Maximum relative rotation error: 0.0508°.
- Final accumulated position error: 0.0071 m.
- Final accumulated heading error: 0.0752°.
- Estimated trajectory closely followed the ground-truth trajectory after correction.

**Problems encountered**
- Initial relative-motion conversion reversed the direction of the estimated motion, causing large translation and heading errors.
- Corrected the transform-to-motion conversion and reran the simulation successfully.

**Decisions made**
- Retained the corrected coordinate-frame conversion.
- Treated the results as simulation validation, not proof of real-world sensor accuracy.

**Next actions**
1. Save the final script and figures.
2. Update `TODO.md` to mark Experiment 18 complete.
3. Commit and push the changes to GitHub.
4. Proceed to Experiment 19.

## 2026-10-16 — Experiment 19: LiDAR Odometry Drift and Error Analysis

**Objective**  
Study how small distance and heading measurement errors accumulate during robot motion and cause odometry drift.

**Work completed**
- Simulated 30 consecutive robot motion steps.
- Generated ground-truth and estimated robot trajectories.
- Introduced a 2% distance scale error, a 0.30° heading bias per step, and small random measurement noise.
- Calculated position error, heading error, and position RMSE.
- Plotted the trajectory comparison and accumulated position and heading errors.

**Commands, files or components used**
- `simulation/MATLAB/exp19_lidar_odometry_drift.m`
- MATLAB trajectory simulation and plotting functions

**Tests performed**
- Compared estimated and ground-truth trajectories.
- Evaluated position and heading errors across all 30 steps.
- Inspected position-error and heading-error plots.

**Results**
- Number of motion steps: 30.
- Mean position error: 0.1061 m.
- Maximum position error: 0.2700 m.
- Final position error: 0.2700 m.
- Position RMSE: 0.1357 m.
- Mean absolute heading error: 4.5411°.
- Maximum absolute heading error: 9.1052°.
- Final absolute heading error: 9.1052°.
- The estimated trajectory progressively diverged from ground truth, demonstrating accumulated odometry drift.

**Problems encountered**
- No execution errors encountered.
- The experiment demonstrated the limitation of relying exclusively on imperfect odometry.

**Decisions made**
- Retained the simulated measurement biases and noise to demonstrate drift.
- Recognized that localization corrections from scan matching or SLAM are needed to limit long-term drift.

**Next actions**
1. Save the final script and figures.
2. Mark Experiment 19 complete in `TODO.md`.
3. Commit and push the changes to GitHub.
4. Proceed to Experiment 20.

## 2026-10-16 — Experiment 20: Loop Closure and Odometry Drift Correction

**Objective**

Demonstrate how accumulated odometry errors affect a robot's estimated trajectory and how a loop-closure correction can reduce the resulting drift.

**Work completed**
- Created and executed `exp20_loop_closure.m`.
- Generated a rectangular ground-truth trajectory with 80 motion steps.
- Simulated odometry drift using distance-scale error, heading bias, and random noise.
- Implemented a simplified trajectory correction distributed across the estimated path.
- Generated trajectory-comparison and position-error plots.

**Commands, files or components used**
- `simulation/MATLAB/exp20_loop_closure.m`
- MATLAB plotting functions and trajectory calculations.

**Tests performed**
- Executed the complete simulation.
- Compared ground-truth, uncorrected, and corrected trajectories.
- Compared position error before and after the simulated loop-closure correction.

**Results**
- Total motion steps: 80.
- Final ground-truth position: approximately (0, 0) m.
- Final estimated position before correction: (8.1175, 0.8600) m.
- Position error before correction: 8.1629 m.
- Position error after correction: 0.0000 m.
- Both figures generated successfully.

**Problems encountered**
- No execution errors reported.
- The correction is idealized: the final position is explicitly aligned with the ground-truth endpoint, so the zero final error is imposed by the simulation rather than independently achieved by a SLAM optimizer.

**Decisions made**
- Treat this experiment as a conceptual demonstration of loop closure and accumulated drift.
- Acknowledge that real loop closure requires recognizing revisited places and optimizing the robot's trajectory or pose graph.

**Next actions**
1. Commit and push the completed Experiment 20 script.
2. Continue with the next SLAM simulation experiment.
3. Later, implement a more realistic loop-closure method using scan matching and pose-graph optimization.

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