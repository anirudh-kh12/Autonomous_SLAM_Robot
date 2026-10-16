
clc;
clear;
close all;

%% Experiment 20: Loop Closure and Odometry Drift Correction

rng(20);

%% 1. Define a closed rectangular robot path

% Four sides of a rectangular route
sideLength = 2.0;
stepsPerSide = 20;
distancePerStep = sideLength / stepsPerSide;

% Heading for each side: 0, 90, 180 and 270 degrees
headings = [0, pi/2, pi, -pi/2];

numberOfSteps = 4 * stepsPerSide;

groundTruth = zeros(numberOfSteps+1,3);
estimatedPose = zeros(numberOfSteps+1,3);

%% 2. Generate ground truth and drifting odometry

distanceScaleError = 1.02;
headingBiasDeg = 0.15;
translationNoiseStd = 0.002;
rotationNoiseStdDeg = 0.03;

step = 1;

for side = 1:4

    for j = 1:stepsPerSide

        trueHeading = headings(side);

        % Ground-truth movement
        groundTruth(step+1,1) = ...
            groundTruth(step,1) + ...
            distancePerStep*cos(trueHeading);

        groundTruth(step+1,2) = ...
            groundTruth(step,2) + ...
            distancePerStep*sin(trueHeading);

        groundTruth(step+1,3) = trueHeading;

        % Noisy odometry measurement
        measuredDistance = distancePerStep*distanceScaleError + ...
            translationNoiseStd*randn();

        measuredHeading = trueHeading + ...
            deg2rad(headingBiasDeg) + ...
            deg2rad(rotationNoiseStdDeg)*randn();

        % Integrate the estimated pose using its current heading
        estimatedPose(step+1,1) = ...
            estimatedPose(step,1) + ...
            measuredDistance*cos(estimatedPose(step,3));

        estimatedPose(step+1,2) = ...
            estimatedPose(step,2) + ...
            measuredDistance*sin(estimatedPose(step,3));

        estimatedPose(step+1,3) = wrapToPi( ...
            estimatedPose(step,3) + ...
            wrapToPi(measuredHeading - trueHeading));

        step = step + 1;

    end
end

%% 3. Calculate drift before loop closure

finalGroundTruth = groundTruth(end,1:2);
finalEstimated = estimatedPose(end,1:2);

preClosureError = norm(finalEstimated-finalGroundTruth);

%% 4. Simulate loop closure correction

% The robot returns to its starting location.
% Assume a loop-closure detector recognizes this match.
% Apply a simple illustrative global correction distributed
% progressively across the estimated trajectory.

correctedPose = estimatedPose;

correctionXY = groundTruth(end,1:2) - estimatedPose(end,1:2);

for k = 1:size(correctedPose,1)

    fraction = (k-1)/(size(correctedPose,1)-1);

    correctedPose(k,1:2) = ...
        estimatedPose(k,1:2) + fraction*correctionXY;

end

% Heading correction is also distributed over the trajectory.
headingCorrection = wrapToPi( ...
    groundTruth(end,3)-estimatedPose(end,3));

for k = 1:size(correctedPose,1)

    fraction = (k-1)/(size(correctedPose,1)-1);

    correctedPose(k,3) = wrapToPi( ...
        estimatedPose(k,3) + fraction*headingCorrection);

end

%% 5. Calculate final errors

postClosureError = norm( ...
    correctedPose(end,1:2)-groundTruth(end,1:2));

positionErrorBefore = sqrt( ...
    sum((estimatedPose(:,1:2)-groundTruth(:,1:2)).^2,2));

positionErrorAfter = sqrt( ...
    sum((correctedPose(:,1:2)-groundTruth(:,1:2)).^2,2));

%% 6. Figure 1: Trajectory comparison

figure;

plot(groundTruth(:,1),groundTruth(:,2), ...
    'b-','LineWidth',2);

hold on;

plot(estimatedPose(:,1),estimatedPose(:,2), ...
    'r--','LineWidth',1.8);

plot(correctedPose(:,1),correctedPose(:,2), ...
    'g-.','LineWidth',1.8);

plot(groundTruth(1,1),groundTruth(1,2), ...
    'ko','MarkerFaceColor','y','MarkerSize',8);

plot(groundTruth(end,1),groundTruth(end,2), ...
    'kp','MarkerFaceColor','y','MarkerSize',10);

grid on;
axis equal;

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Experiment 20: Loop Closure Correction');

legend('Ground Truth','Odometry Before Correction', ...
    'Trajectory After Correction','Start','Loop Closure', ...
    'Location','best');

hold off;

%% 7. Figure 2: Position error comparison

figure;

plot(positionErrorBefore,'r--','LineWidth',1.8);
hold on;
plot(positionErrorAfter,'g-','LineWidth',1.8);

grid on;

xlabel('Motion Step');
ylabel('Position Error (m)');
title('Position Error Before and After Loop Closure');

legend('Before Correction','After Correction', ...
    'Location','best');

hold off;

%% 8. Display results

fprintf('\nExperiment 20 Results\n');
fprintf('---------------------\n');

fprintf('Total motion steps: %d\n',numberOfSteps);

fprintf('Final ground-truth position: (%.4f, %.4f) m\n', ...
    finalGroundTruth(1),finalGroundTruth(2));

fprintf('Final estimated position before correction: (%.4f, %.4f) m\n', ...
    finalEstimated(1),finalEstimated(2));

fprintf('Position error before loop closure: %.4f m\n', ...
    preClosureError);

fprintf('Position error after loop closure: %.4f m\n', ...
    postClosureError);

fprintf('Final corrected position: (%.4f, %.4f) m\n', ...
    correctedPose(end,1),correctedPose(end,2));

fprintf('\nExperiment 20 simulation completed.\n');
