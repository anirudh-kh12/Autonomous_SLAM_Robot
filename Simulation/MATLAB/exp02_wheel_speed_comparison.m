clc;
clear;
close all;

% Experiment 02: Effect of wheel velocities
% on differential-drive robot motion

wheelBase = 0.18;    % Distance between wheels (m)
dt = 0.05;           % Time step (s)
totalTime = 8;       % Simulation duration (s)

time = 0:dt:totalTime;
N = length(time);

% Four wheel-speed combinations
% Columns: [Left wheel, Right wheel]
cases = [
    0.15,  0.15;   % Case 1: Straight
    0.08,  0.15;   % Case 2: Left turn
    0.15,  0.08;   % Case 3: Right turn
    -0.10, 0.10    % Case 4: Rotate in place
];

caseNames = {
    'Straight Motion'
    'Left Turn'
    'Right Turn'
    'Rotate in Place'
};

% Store robot trajectories
X = zeros(N, 4);
Y = zeros(N, 4);
Theta = zeros(N, 4);

% Simulate each wheel-speed combination
for c = 1:4

    vLeft = cases(c, 1);
    vRight = cases(c, 2);

    % Calculate linear and angular velocity
    v = (vRight + vLeft) / 2;
    omega = (vRight - vLeft) / wheelBase;

    % Initial pose
    x = 0;
    y = 0;
    theta = 0;

    for k = 1:N-1

        % Update position
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;

        % Update orientation
        theta = theta + omega * dt;

        X(k+1, c) = x;
        Y(k+1, c) = y;
        Theta(k+1, c) = theta;

    end

    fprintf('\n%s\n', caseNames{c});
    fprintf('Linear velocity: %.3f m/s\n', v);
    fprintf('Angular velocity: %.3f rad/s\n', omega);
    fprintf('Final X: %.3f m\n', x);
    fprintf('Final Y: %.3f m\n', y);
    fprintf('Final orientation: %.2f degrees\n', ...
        theta * 180 / pi);

end

% Plot all trajectories
figure;
hold on;

for c = 1:4
    plot(X(:, c), Y(:, c), 'LineWidth', 2);
end

plot(0, 0, 'ko', 'MarkerFaceColor', 'g');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Effect of Wheel Speeds on Robot Trajectory');

legend([caseNames; {'Start Position'}], ...
    'Location', 'best');

axis equal;
grid on;

% Plot orientation against time
figure;
hold on;

for c = 1:4
    plot(time, Theta(:, c) * 180 / pi, ...
        'LineWidth', 2);
end

xlabel('Time (s)');
ylabel('Orientation (degrees)');
title('Wheel Speeds vs Robot Orientation');

legend(caseNames, 'Location', 'best');
grid on;
