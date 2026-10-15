clc;
clear;
close all;

% Experiment 03: Robot pose and coordinate frames

% Robot parameters
wheelBase = 0.18;
dt = 0.05;
totalTime = 12;

time = 0:dt:totalTime;
N = length(time);

% Initialize robot pose
x = zeros(1, N);
y = zeros(1, N);
theta = zeros(1, N);

% Simulate three motion stages
for k = 1:N-1

    if time(k) < 4
        % Stage 1: Straight movement
        vLeft = 0.15;
        vRight = 0.15;

    elseif time(k) < 8
        % Stage 2: Turn left
        vLeft = 0.08;
        vRight = 0.15;

    else
        % Stage 3: Turn right
        vLeft = 0.15;
        vRight = 0.08;
    end

    % Robot linear and angular velocity
    v = (vRight + vLeft) / 2;
    omega = (vRight - vLeft) / wheelBase;

    % Update pose
    x(k+1) = x(k) + v * cos(theta(k)) * dt;
    y(k+1) = y(k) + v * sin(theta(k)) * dt;

    theta(k+1) = theta(k) + omega * dt;

end

% Plot trajectory and selected robot orientations
figure;
plot(x, y, 'b-', 'LineWidth', 2);
hold on;
plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g');
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');

% Draw heading arrows at selected positions
indices = 1:20:N;

quiver(x(indices), y(indices), ...
    0.12*cos(theta(indices)), ...
    0.12*sin(theta(indices)), ...
    0, 'r', 'LineWidth', 1.2);

xlabel('World X Position (m)');
ylabel('World Y Position (m)');
title('Robot Pose and Heading in the World Frame');

legend('Robot Trajectory', 'Start', ...
    'End', 'Heading');
axis equal;
grid on;

% Demonstrate coordinate transformation
robotX = 2;
robotY = 1;
robotTheta = pi/2;  % 90 degrees

% A point 1 metre ahead in robot coordinates
localPoint = [1; 0];

% Rotation matrix
R = [cos(robotTheta), -sin(robotTheta);
     sin(robotTheta),  cos(robotTheta)];

% Convert local point to world coordinates
worldPoint = [robotX; robotY] + R * localPoint;

fprintf('\nCoordinate Transformation Example\n');
fprintf('Robot position: (%.2f, %.2f)\n', ...
    robotX, robotY);
fprintf('Robot heading: %.2f degrees\n', ...
    robotTheta * 180 / pi);
fprintf('Local point: (%.2f, %.2f)\n', ...
    localPoint(1), localPoint(2));
fprintf('World point: (%.2f, %.2f)\n', ...
    worldPoint(1), worldPoint(2));

% Draw the world-frame transformation example
figure;
hold on;

quiver(robotX, robotY, 0.5, 0, 0, ...
    'k', 'LineWidth', 1.5);
quiver(robotX, robotY, 0, 0.5, 0, ...
    'k', 'LineWidth', 1.5);

quiver(robotX, robotY, ...
    0.5*cos(robotTheta), ...
    0.5*sin(robotTheta), 0, ...
    'b', 'LineWidth', 2);

plot(robotX, robotY, 'bo', ...
    'MarkerFaceColor', 'b');
plot(worldPoint(1), worldPoint(2), ...
    'ro', 'MarkerFaceColor', 'r');

plot([robotX worldPoint(1)], ...
    [robotY worldPoint(2)], 'r--');

text(robotX, robotY, ' Robot');
text(worldPoint(1), worldPoint(2), ' Point');

xlabel('World X (m)');
ylabel('World Y (m)');
title('Transforming a Robot-Frame Point to World Coordinates');

legend('World X', 'World Y', ...
    'Robot Heading', 'Robot Position', ...
    'Transformed Point', 'Location', 'best');

axis equal;
grid on;