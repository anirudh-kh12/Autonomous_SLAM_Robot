
clc;
clear;
close all;

% Differential-drive mobile robot simulation

% Robot parameters
wheelBase = 0.18;       % Distance between left and right wheels (m)
dt = 0.05;              % Time step (s)
totalTime = 10;         % Simulation duration (s)

time = 0:dt:totalTime;
N = length(time);

% Robot pose
x = zeros(1, N);
y = zeros(1, N);
theta = zeros(1, N);

% Wheel linear velocities (m/s)
vLeft = zeros(1, N);
vRight = zeros(1, N);

% First 5 seconds: straight movement
% Next 5 seconds: turning
for k = 1:N

    if time(k) < 5
        vLeft(k) = 0.15;
        vRight(k) = 0.15;
    else
        vLeft(k) = 0.08;
        vRight(k) = 0.15;
    end

end

% Simulate robot movement
for k = 1:N-1

    % Linear velocity of robot
    v = (vRight(k) + vLeft(k)) / 2;

    % Angular velocity of robot
    omega = (vRight(k) - vLeft(k)) / wheelBase;

    % Update robot position
    x(k+1) = x(k) + v * cos(theta(k)) * dt;
    y(k+1) = y(k) + v * sin(theta(k)) * dt;

    % Update robot orientation
    theta(k+1) = theta(k) + omega * dt;

end

% Plot robot trajectory
figure;

plot(x, y, 'b-', 'LineWidth', 2);
hold on;

plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g');
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Differential-Drive Robot Trajectory');

legend('Robot Path', 'Start Position', 'End Position');
axis equal;
grid on;

% Plot robot orientation over time
figure;

plot(time, theta * 180 / pi, 'LineWidth', 2);

xlabel('Time (s)');
ylabel('Orientation (degrees)');
title('Robot Orientation vs Time');
grid on;

% Display final pose
fprintf('Final X Position: %.3f m\n', x(end));
fprintf('Final Y Position: %.3f m\n', y(end));
fprintf('Final Orientation: %.2f degrees\n', ...
    theta(end) * 180 / pi);