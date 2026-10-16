
clc;
clear;
close all;

%% Experiment 19: LiDAR Odometry Drift and Error Analysis

%% 1. Simulation parameters

rng(7);  % Reproducible results

numberOfSteps = 30;
timeStep = 1.0;

% Simulated odometry errors
distanceScaleError = 1.02;      % 2% distance overestimation
headingBiasDeg = 0.30;          % Heading error per step
translationNoiseStd = 0.003;    % Translation noise (m)
rotationNoiseStdDeg = 0.05;     % Rotation noise (degrees)

%% 2. Initialize robot poses
% Each pose is [x, y, heading in radians]

groundTruth = zeros(numberOfSteps+1,3);
estimatedPose = zeros(numberOfSteps+1,3);

% Robot starts at the origin
groundTruth(1,:) = [0,0,0];
estimatedPose(1,:) = [0,0,0];

%% 3. Generate ground-truth motion and odometry estimates

for k = 1:numberOfSteps

    % Simulated forward movement and gradual turning
    trueDistance = 0.15;
    trueRotation = deg2rad(2 + 0.15*k);

    previousPose = groundTruth(k,:);

    % Update ground-truth pose
    groundTruth(k+1,1) = previousPose(1) + ...
        trueDistance*cos(previousPose(3));

    groundTruth(k+1,2) = previousPose(2) + ...
        trueDistance*sin(previousPose(3));

    groundTruth(k+1,3) = wrapToPi( ...
        previousPose(3) + trueRotation);

    % Simulate imperfect odometry measurements
    measuredDistance = trueDistance*distanceScaleError + ...
        translationNoiseStd*randn();

    measuredRotation = trueRotation + ...
        deg2rad(headingBiasDeg) + ...
        deg2rad(rotationNoiseStdDeg)*randn();

    % Integrate estimated motion
    estimatePrevious = estimatedPose(k,:);

    estimatedPose(k+1,1) = estimatePrevious(1) + ...
        measuredDistance*cos(estimatePrevious(3));

    estimatedPose(k+1,2) = estimatePrevious(2) + ...
        measuredDistance*sin(estimatePrevious(3));

    estimatedPose(k+1,3) = wrapToPi( ...
        estimatePrevious(3) + measuredRotation);

end

%% 4. Calculate position and heading errors

positionError = sqrt( ...
    (estimatedPose(:,1)-groundTruth(:,1)).^2 + ...
    (estimatedPose(:,2)-groundTruth(:,2)).^2);

headingErrorDeg = rad2deg(wrapToPi( ...
    estimatedPose(:,3)-groundTruth(:,3)));

meanPositionError = mean(positionError);
maximumPositionError = max(positionError);
finalPositionError = positionError(end);

meanHeadingError = mean(abs(headingErrorDeg));
maximumHeadingError = max(abs(headingErrorDeg));
finalHeadingError = abs(headingErrorDeg(end));

positionRMSE = sqrt(mean(positionError.^2));

%% 5. Figure 1: Ground truth vs estimated trajectory

figure;

plot(groundTruth(:,1),groundTruth(:,2), ...
    'b-o','LineWidth',1.8,'MarkerSize',4);

hold on;

plot(estimatedPose(:,1),estimatedPose(:,2), ...
    'r--','LineWidth',1.8);

plot(groundTruth(1,1),groundTruth(1,2), ...
    'go','MarkerFaceColor','g','MarkerSize',8);

plot(groundTruth(end,1),groundTruth(end,2), ...
    'bp','MarkerFaceColor','b','MarkerSize',10);

plot(estimatedPose(end,1),estimatedPose(end,2), ...
    'rp','MarkerFaceColor','r','MarkerSize',10);

grid on;
axis equal;

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Experiment 19: Odometry Drift');
legend('Ground Truth','Estimated Trajectory', ...
    'Start','Ground-Truth End','Estimated End', ...
    'Location','best');

hold off;

%% 6. Figure 2: Position error over time

time = (0:numberOfSteps)'*timeStep;

figure;

plot(time,positionError,'b-','LineWidth',1.8);

grid on;

xlabel('Time (s)');
ylabel('Position Error (m)');
title('Accumulated Position Error');

%% 7. Figure 3: Heading error over time

figure;

plot(time,abs(headingErrorDeg),'r-','LineWidth',1.8);

grid on;

xlabel('Time (s)');
ylabel('Absolute Heading Error (degrees)');
title('Accumulated Heading Error');

%% 8. Display results

fprintf('\nExperiment 19 Results\n');
fprintf('---------------------\n');

fprintf('Number of motion steps: %d\n',numberOfSteps);
fprintf('Distance scale error: %.1f%%\n', ...
    (distanceScaleError-1)*100);

fprintf('Heading bias per step: %.2f degrees\n', ...
    headingBiasDeg);

fprintf('\nPosition Error Metrics\n');
fprintf('Mean position error: %.4f m\n',meanPositionError);
fprintf('Maximum position error: %.4f m\n',maximumPositionError);
fprintf('Final position error: %.4f m\n',finalPositionError);
fprintf('Position RMSE: %.4f m\n',positionRMSE);

fprintf('\nHeading Error Metrics\n');
fprintf('Mean absolute heading error: %.4f degrees\n', ...
    meanHeadingError);

fprintf('Maximum absolute heading error: %.4f degrees\n', ...
    maximumHeadingError);

fprintf('Final absolute heading error: %.4f degrees\n', ...
    finalHeadingError);

fprintf('\nExperiment 19 simulation completed.\n');
