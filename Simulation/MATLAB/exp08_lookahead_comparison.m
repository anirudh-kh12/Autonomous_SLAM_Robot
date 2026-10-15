
clc;
clear;
close all;

%% Experiment 08: Pure Pursuit Lookahead Comparison

% Robot parameters
linearSpeed = 0.15;
wheelBase = 0.18;

% Simulation parameters
dt = 0.05;
maxTime = 100;
maxSteps = round(maxTime / dt);
goalTolerance = 0.12;

% Lookahead values to compare
lookaheadValues = [0.20, 0.35, 0.60];

%% Define desired path

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

pathDistance = [0; cumsum(sqrt(sum(diff(waypoints).^2, 2)))];

fineDistance = (0:0.05:pathDistance(end))';

pathX = interp1(pathDistance, waypoints(:,1), ...
    fineDistance, 'linear');

pathY = interp1(pathDistance, waypoints(:,2), ...
    fineDistance, 'linear');

path = [pathX, pathY];
goalPosition = waypoints(end,:);

%% Storage for results

numTests = numel(lookaheadValues);

allTrajectories = cell(numTests,1);
meanErrors = zeros(numTests,1);
maxErrors = zeros(numTests,1);
completionTimes = NaN(numTests,1);
finalPositions = zeros(numTests,2);
goalReached = false(numTests,1);

%% Run each test

for test = 1:numTests

    lookaheadDistance = lookaheadValues(test);

    % Initial pose [x, y, heading]
    pose = [waypoints(1,:), 0];

    history = zeros(maxSteps + 1,3);
    history(1,:) = pose;
    stepsTaken = 0;

    for step = 1:maxSteps

        % Find nearest path point
        distances = sqrt(sum((path - pose(1:2)).^2,2));
        [~, nearestIndex] = min(distances);

        % Find target at the lookahead distance
        targetIndex = nearestIndex;

        while targetIndex < size(path,1)

            targetDistance = norm( ...
                path(targetIndex,:) - pose(1:2));

            if targetDistance >= lookaheadDistance
                break;
            end

            targetIndex = targetIndex + 1;
        end

        % Check goal condition
        goalDistance = norm(goalPosition - pose(1:2));

        if goalDistance < goalTolerance
            goalReached(test) = true;
            completionTimes(test) = (step - 1) * dt;
            break;
        end

        % Calculate heading error
        target = path(targetIndex,:);

        dx = target(1) - pose(1);
        dy = target(2) - pose(2);

        alpha = atan2(dy,dx) - pose(3);
        alpha = atan2(sin(alpha),cos(alpha));

        % Pure Pursuit curvature
        actualLookahead = max( ...
            norm(target - pose(1:2)), 0.001);

        curvature = 2 * sin(alpha) / actualLookahead;
        angularSpeed = linearSpeed * curvature;

        % Differential-drive wheel speeds
        leftSpeed = linearSpeed - ...
            angularSpeed * wheelBase / 2;

        rightSpeed = linearSpeed + ...
            angularSpeed * wheelBase / 2;

        % Robot kinematics
        actualLinearSpeed = (rightSpeed + leftSpeed) / 2;
        actualAngularSpeed = (rightSpeed - leftSpeed) / wheelBase;

        pose(1) = pose(1) + ...
            actualLinearSpeed * cos(pose(3)) * dt;

        pose(2) = pose(2) + ...
            actualLinearSpeed * sin(pose(3)) * dt;

        pose(3) = pose(3) + actualAngularSpeed * dt;

        history(step + 1,:) = pose;
        stepsTaken = step;
    end

    if ~goalReached(test)
        stepsTaken = maxSteps;
    end

    history = history(1:stepsTaken + 1,:);
    allTrajectories{test} = history;
    finalPositions(test,:) = pose(1:2);

    % Calculate tracking error
    errors = zeros(size(history,1),1);

    for i = 1:size(history,1)
        distances = sqrt(sum( ...
            (path - history(i,1:2)).^2,2));

        errors(i) = min(distances);
    end

    meanErrors(test) = mean(errors);
    maxErrors(test) = max(errors);
end

%% Plot all trajectories

figure;
plot(path(:,1),path(:,2),'k--','LineWidth',2);
hold on;

plot(waypoints(:,1),waypoints(:,2), ...
    'ko','MarkerFaceColor','k');

plot(allTrajectories{1}(:,1),allTrajectories{1}(:,2), ...
    'r-','LineWidth',1.5);

plot(allTrajectories{2}(:,1),allTrajectories{2}(:,2), ...
    'b-','LineWidth',1.5);

plot(allTrajectories{3}(:,1),allTrajectories{3}(:,2), ...
    'g-','LineWidth',1.5);

plot(goalPosition(1),goalPosition(2), ...
    'mp','MarkerFaceColor','m','MarkerSize',12);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Pure Pursuit: Lookahead Distance Comparison');

legend('Desired Path','Waypoints', ...
    'Lookahead 0.20 m','Lookahead 0.35 m', ...
    'Lookahead 0.60 m','Goal', ...
    'Location','best');

axis equal;
grid on;
hold off;

%% Plot mean and maximum tracking errors

figure;

bar(lookaheadValues, [meanErrors, maxErrors]);
xlabel('Lookahead Distance (m)');
ylabel('Tracking Error (m)');
title('Tracking Error vs Lookahead Distance');

legend('Mean Error','Maximum Error','Location','best');
grid on;

%% Display comparison table

results = table(lookaheadValues',goalReached, ...
    completionTimes,meanErrors,maxErrors, ...
    finalPositions(:,1),finalPositions(:,2), ...
    'VariableNames',{'Lookahead_m','GoalReached', ...
    'Time_s','MeanError_m','MaxError_m', ...
    'FinalX_m','FinalY_m'});

disp(' ');
disp('Experiment 08: Lookahead Comparison Results');
disp(results);
