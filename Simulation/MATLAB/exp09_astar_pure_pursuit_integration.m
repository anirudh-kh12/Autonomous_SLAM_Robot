
clc;
clear;
close all;

%% Experiment 09: A* + Pure Pursuit Integration

%% 1. Parameters

mapWidth = 10;
mapHeight = 8;
resolution = 10;

robotRadius = 0.20;
safetyMargin = 0.10;
wheelBase = 0.18;

linearSpeed = 0.15;
lookaheadDistance = 0.10;

% Limit turning rate for more controlled motion
maxAngularSpeed = 0.8;  % rad/s

dt = 0.05;
maxTime = 150;
maxSteps = round(maxTime / dt);

goalTolerance = 0.15;
collisionSampleSpacing = 0.01;

%% 2. Create occupancy map

map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

xCells = 0.05:0.1:9.95;
yCells = 0.05:0.1:7.95;

% Boundary walls
setOccupancy(map, [xCells(:), ...
    0.05 * ones(numel(xCells),1)], 1);

setOccupancy(map, [xCells(:), ...
    7.95 * ones(numel(xCells),1)], 1);

setOccupancy(map, [0.05 * ones(numel(yCells),1), ...
    yCells(:)], 1);

setOccupancy(map, [9.95 * ones(numel(yCells),1), ...
    yCells(:)], 1);

% Obstacle 1
[obsX,obsY] = meshgrid(3.05:0.1:4.95, ...
                        2.05:0.1:3.95);

setOccupancy(map, [obsX(:),obsY(:)], 1);

% Obstacle 2
[obsX2,obsY2] = meshgrid(6.05:0.1:7.05, ...
                          4.05:0.1:6.05);

setOccupancy(map, [obsX2(:),obsY2(:)], 1);

%% 3. Inflate obstacles

inflatedMap = copy(map);

inflate(inflatedMap, robotRadius + safetyMargin);

%% 4. Plan route using A*

startPosition = [1,1];
goalPosition = [8.5,6.5];

planner = plannerAStarGrid(inflatedMap);

plannedPath = plan(planner, startPosition, ...
    goalPosition, 'world');

if isempty(plannedPath)
    error('A* could not find a valid path.');
end

if any(checkOccupancy(inflatedMap,plannedPath) > 0)
    error('A* returned a path containing occupied points.');
end

fprintf('A* planning successful.\n');
fprintf('Planned waypoints: %d\n',size(plannedPath,1));

%% 5. Prepare path progress tracking

% Cumulative distance along the planned path
segmentLengths = sqrt(sum(diff(plannedPath).^2,2));

cumulativeDistance = [0; cumsum(segmentLengths)];

% Progress index will only move forward
progressIndex = 1;

%% 6. Initialize robot

% Pose = [x, y, heading]
pose = [startPosition, 0];

robotHistory = zeros(maxSteps + 1,3);
robotHistory(1,:) = pose;

stepsTaken = 0;
goalReached = false;
collisionDetected = false;

collisionPosition = [NaN, NaN];

%% 7. Follow path using Pure Pursuit

for step = 1:maxSteps

    % Check final goal
    goalDistance = norm(goalPosition - pose(1:2));

    if goalDistance < goalTolerance
        goalReached = true;
        break;
    end

    %% Find nearest waypoint without moving progress backwards

    remainingPath = plannedPath(progressIndex:end,:);

    distances = sqrt(sum( ...
        (remainingPath - pose(1:2)).^2,2));

    [~,localIndex] = min(distances);

    progressIndex = progressIndex + localIndex - 1;

    %% Select a target ahead along path distance

    targetDistanceAlongPath = ...
        cumulativeDistance(progressIndex) + lookaheadDistance;

    targetIndex = find( ...
        cumulativeDistance >= targetDistanceAlongPath,1,'first');

    if isempty(targetIndex)
        targetIndex = size(plannedPath,1);
    end

    target = plannedPath(targetIndex,:);

    %% Calculate Pure Pursuit steering

    dx = target(1) - pose(1);
    dy = target(2) - pose(2);

    alpha = atan2(dy,dx) - pose(3);

    % Normalize heading error
    alpha = atan2(sin(alpha),cos(alpha));

    % Use actual target distance
    actualLookahead = max(norm(target - pose(1:2)),0.001);

    curvature = 2 * sin(alpha) / actualLookahead;

    angularSpeed = linearSpeed * curvature;

    % Limit angular velocity
    angularSpeed = max(-maxAngularSpeed, ...
        min(maxAngularSpeed,angularSpeed));

    %% Differential-drive wheel velocities

    leftSpeed = linearSpeed - ...
        angularSpeed * wheelBase / 2;

    rightSpeed = linearSpeed + ...
        angularSpeed * wheelBase / 2;

    % Recalculate robot velocities
    actualLinearSpeed = (rightSpeed + leftSpeed) / 2;

    actualAngularSpeed = ...
        (rightSpeed - leftSpeed) / wheelBase;

    %% Predict next pose

    nextPose = pose;

    nextPose(1) = pose(1) + ...
        actualLinearSpeed * cos(pose(3)) * dt;

    nextPose(2) = pose(2) + ...
        actualLinearSpeed * sin(pose(3)) * dt;

    nextPose(3) = pose(3) + actualAngularSpeed * dt;

    %% Check the movement segment for collision

    previousPosition = pose(1:2);
    nextPosition = nextPose(1:2);

    movementDistance = norm(nextPosition - previousPosition);

    sampleCount = max(2, ...
        ceil(movementDistance / collisionSampleSpacing) + 1);

    t = linspace(0,1,sampleCount)';

    movementPoints = previousPosition + ...
        t .* (nextPosition - previousPosition);

    occupancyValues = checkOccupancy( ...
        inflatedMap,movementPoints);

    if any(occupancyValues > 0)

        collisionDetected = true;
        collisionPosition = nextPosition;

        fprintf('Collision detected near (%.2f, %.2f) m.\n', ...
            nextPosition(1),nextPosition(2));

        break;
    end

    %% Accept movement

    pose = nextPose;

    robotHistory(step + 1,:) = pose;
    stepsTaken = step;
end

%% 8. Check final goal after last accepted movement

if norm(goalPosition - pose(1:2)) < goalTolerance
    goalReached = true;
end

robotHistory = robotHistory(1:stepsTaken + 1,:);

%% 9. Calculate tracking error

trackingError = zeros(size(robotHistory,1),1);

for i = 1:size(robotHistory,1)

    distances = sqrt(sum( ...
        (plannedPath - robotHistory(i,1:2)).^2,2));

    trackingError(i) = min(distances);
end

meanError = mean(trackingError);
maxError = max(trackingError);

%% 10. Calculate trajectory length

actualPathLength = sum(sqrt(sum( ...
    diff(robotHistory(:,1:2)).^2,2)));

%% 11. Plot navigation result

figure;

show(inflatedMap);
hold on;

plot(plannedPath(:,1),plannedPath(:,2), ...
    'b--','LineWidth',1.5);

plot(robotHistory(:,1),robotHistory(:,2), ...
    'g-','LineWidth',2);

plot(startPosition(1),startPosition(2), ...
    'go','MarkerFaceColor','g','MarkerSize',9);

plot(goalPosition(1),goalPosition(2), ...
    'rp','MarkerFaceColor','r','MarkerSize',12);

if collisionDetected
    plot(collisionPosition(1),collisionPosition(2), ...
        'rx','MarkerSize',12,'LineWidth',2);
end

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Integrated A* Planning and Pure Pursuit');

legend('Inflated Map','A* Planned Path', ...
    'Robot Trajectory','Start','Goal', ...
    'Location','best');

hold off;

%% 12. Plot tracking error

figure;

time = (0:size(robotHistory,1)-1)' * dt;

plot(time,trackingError,'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Tracking Error (m)');
title('Integrated Navigation Tracking Error');

grid on;

%% 13. Display results

fprintf('\nExperiment 09 Results\n');

fprintf('Robot radius: %.2f m\n',robotRadius);
fprintf('Safety margin: %.2f m\n',safetyMargin);
fprintf('Lookahead distance: %.2f m\n',lookaheadDistance);
fprintf('Maximum angular speed: %.2f rad/s\n',maxAngularSpeed);

fprintf('Simulation time: %.2f s\n',stepsTaken * dt);
fprintf('Trajectory points: %d\n',size(robotHistory,1));
fprintf('Actual trajectory length: %.3f m\n',actualPathLength);
fprintf('Mean tracking error: %.3f m\n',meanError);
fprintf('Maximum tracking error: %.3f m\n',maxError);

fprintf('Final position: (%.2f, %.2f) m\n', ...
    pose(1),pose(2));

fprintf('Goal reached: %s\n',mat2str(goalReached));
fprintf('Collision detected: %s\n',mat2str(collisionDetected));

if goalReached && ~collisionDetected
    fprintf('Navigation result: PASSED\n');
elseif collisionDetected
    fprintf('Navigation result: FAILED - collision detected\n');
else
    fprintf('Navigation result: INCOMPLETE - goal not reached\n');
end
