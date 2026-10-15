
clc;
clear;
close all;

%% Experiment 07: Pure Pursuit Path Following

% Robot parameters
wheelBase = 0.18;       % Distance between wheels (m)
linearSpeed = 0.15;     % Forward speed (m/s)
lookaheadDistance = 0.35; % Pure Pursuit lookahead (m)

% Simulation parameters
dt = 0.05;
maxTime = 100;
maxSteps = round(maxTime / dt);

%% Define a desired path

% Each row represents [x, y] in metres
waypoints = [
    0.5, 0.5;
    2.0, 0.5;
    3.0, 1.0;
    4.0, 2.0;
    5.0, 2.0;
    6.0, 3.0;
    7.0, 4.0;
    8.0, 4.0
];

% Interpolate the path for smoother target selection
pathDistance = [0; cumsum(sqrt(sum(diff(waypoints).^2, 2)))];

fineDistance = (0:0.05:pathDistance(end))';

pathX = interp1(pathDistance, waypoints(:,1), ...
    fineDistance, 'linear');

pathY = interp1(pathDistance, waypoints(:,2), ...
    fineDistance, 'linear');

path = [pathX, pathY];

%% Initial robot pose

% Pose = [x, y, heading]
pose = [0.5, 0.5, 0];

robotHistory = zeros(maxSteps + 1, 3);
robotHistory(1,:) = pose;

%% Pure Pursuit simulation

goalTolerance = 0.12;
goalPosition = waypoints(end,:);

for step = 1:maxSteps

    % Find nearest path point
    distances = sqrt(sum((path - pose(1:2)).^2, 2));
    [~, nearestIndex] = min(distances);

    % Find a target point at least the lookahead
    % distance ahead of the nearest path point
    targetIndex = nearestIndex;

    while targetIndex < size(path,1)

        targetDistance = norm(path(targetIndex,:) - pose(1:2));

        if targetDistance >= lookaheadDistance
            break;
        end

        targetIndex = targetIndex + 1;
    end

    % Stop when the robot reaches the final goal
    goalDistance = norm(goalPosition - pose(1:2));

    if goalDistance < goalTolerance
        robotHistory = robotHistory(1:step,:);
        fprintf('Goal reached at %.2f seconds.\n', ...
            (step - 1) * dt);
        break;
    end

    % Target position in world coordinates
    target = path(targetIndex,:);

    % Convert target direction to robot coordinates
    dx = target(1) - pose(1);
    dy = target(2) - pose(2);

    alpha = atan2(dy, dx) - pose(3);

    % Normalize heading error to [-pi, pi]
    alpha = atan2(sin(alpha), cos(alpha));

    % Pure Pursuit curvature
    actualLookahead = max(norm(target - pose(1:2)), 0.001);

    curvature = 2 * sin(alpha) / actualLookahead;

    % Convert curvature to angular velocity
    angularSpeed = linearSpeed * curvature;

    % Differential-drive wheel velocities
    leftSpeed = linearSpeed - ...
        angularSpeed * wheelBase / 2;

    rightSpeed = linearSpeed + ...
        angularSpeed * wheelBase / 2;

    % Recalculate actual robot velocities
    actualLinearSpeed = (rightSpeed + leftSpeed) / 2;

    actualAngularSpeed = (rightSpeed - leftSpeed) ...
        / wheelBase;

    % Update robot pose
    pose(1) = pose(1) + ...
        actualLinearSpeed * cos(pose(3)) * dt;

    pose(2) = pose(2) + ...
        actualLinearSpeed * sin(pose(3)) * dt;

    pose(3) = pose(3) + actualAngularSpeed * dt;

    % Store robot pose
    robotHistory(step + 1,:) = pose;
end

%% Trim history to recorded simulation steps

validRows = find(any(robotHistory, 2));

if isempty(validRows)
    error('No robot trajectory was recorded.');
end

robotHistory = robotHistory(1:max(validRows),:);

%% Calculate tracking error

actualXY = robotHistory(:,1:2);

trackingError = zeros(size(actualXY,1),1);

for i = 1:size(actualXY,1)
    distances = sqrt(sum((path - actualXY(i,:)).^2, 2));
    trackingError(i) = min(distances);
end

meanError = mean(trackingError);
maxError = max(trackingError);

%% Plot desired and actual paths

figure;

plot(path(:,1), path(:,2), 'b--', ...
    'LineWidth', 1.8);
hold on;

plot(robotHistory(:,1), robotHistory(:,2), ...
    'g-', 'LineWidth', 2);

plot(waypoints(:,1), waypoints(:,2), ...
    'bo', 'MarkerFaceColor', 'b');

plot(waypoints(1,1), waypoints(1,2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

plot(goalPosition(1), goalPosition(2), ...
    'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Pure Pursuit Path Following');
legend('Desired Path', 'Robot Trajectory', ...
    'Waypoints', 'Start', 'Goal', ...
    'Location', 'best');

axis equal;
grid on;
hold off;

%% Plot tracking error

figure;

time = (0:size(robotHistory,1)-1)' * dt;

plot(time, trackingError, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Tracking Error (m)');
title('Robot Path Tracking Error');

grid on;

%% Display results

fprintf('\nPure Pursuit Results\n');
fprintf('Lookahead distance: %.2f m\n', lookaheadDistance);
fprintf('Forward speed: %.2f m/s\n', linearSpeed);
fprintf('Recorded trajectory points: %d\n', ...
    size(robotHistory,1));
fprintf('Mean tracking error: %.3f m\n', meanError);
fprintf('Maximum tracking error: %.3f m\n', maxError);
fprintf('Final robot position: (%.2f, %.2f) m\n', ...
    pose(1), pose(2));
fprintf('Goal position: (%.2f, %.2f) m\n', ...
    goalPosition(1), goalPosition(2));
