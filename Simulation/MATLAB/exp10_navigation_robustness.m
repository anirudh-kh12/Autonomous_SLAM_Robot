
clc;
clear;
close all;

%% Experiment 10: Navigation Robustness

%% 1. Parameters

mapWidth = 10;
mapHeight = 8;
resolution = 10;

robotRadius = 0.20;
safetyMargin = 0.10;
wheelBase = 0.18;

linearSpeed = 0.15;
lookaheadDistance = 0.10;
maxAngularSpeed = 0.8;

dt = 0.05;
maxTime = 150;
maxSteps = round(maxTime / dt);

goalTolerance = 0.15;
collisionSampleSpacing = 0.01;

%% 2. Create occupancy map

map = binaryOccupancyMap(mapWidth,mapHeight,resolution);

xCells = 0.05:0.1:9.95;
yCells = 0.05:0.1:7.95;

% Boundary walls
setOccupancy(map,[xCells(:), ...
    0.05*ones(numel(xCells),1)],1);

setOccupancy(map,[xCells(:), ...
    7.95*ones(numel(xCells),1)],1);

setOccupancy(map,[0.05*ones(numel(yCells),1), ...
    yCells(:)],1);

setOccupancy(map,[9.95*ones(numel(yCells),1), ...
    yCells(:)],1);

% Obstacle 1
[obsX,obsY] = meshgrid(3.05:0.1:4.95, ...
                        2.05:0.1:3.95);

setOccupancy(map,[obsX(:),obsY(:)],1);

% Obstacle 2
[obsX2,obsY2] = meshgrid(6.05:0.1:7.05, ...
                          4.05:0.1:6.05);

setOccupancy(map,[obsX2(:),obsY2(:)],1);

% Preserve the original obstacle map for clearance measurements
originalMap = copy(map);

% Inflate obstacles for robot clearance
inflatedMap = copy(map);
inflate(inflatedMap,robotRadius+safetyMargin);

%% 3. Define test routes

% Each row is [startX startY goalX goalY]
testRoutes = [
    1.0, 1.0, 8.5, 6.5;
    1.0, 6.5, 8.5, 1.0;
    1.0, 4.0, 8.5, 4.0
];

numTests = size(testRoutes,1);

%% 4. Prepare results

goalReachedResults = false(numTests,1);
collisionResults = false(numTests,1);

pathLengths = NaN(numTests,1);
meanErrors = NaN(numTests,1);
maxErrors = NaN(numTests,1);
minimumClearances = NaN(numTests,1);
completionTimes = NaN(numTests,1);

allTrajectories = cell(numTests,1);
allPlannedPaths = cell(numTests,1);

%% 5. Run each test

for test = 1:numTests

    fprintf('\nRunning test %d of %d...\n',test,numTests);

    startPosition = testRoutes(test,1:2);
    goalPosition = testRoutes(test,3:4);

    % Plan using A*
    planner = plannerAStarGrid(inflatedMap);

    plannedPath = plan(planner,startPosition, ...
        goalPosition,'world');

    if isempty(plannedPath)
        fprintf('No path found for test %d.\n',test);
        continue;
    end

    if any(checkOccupancy(inflatedMap,plannedPath)>0)
        fprintf('Planned path contains occupied points.\n');
        continue;
    end

    allPlannedPaths{test} = plannedPath;

    % Initialize robot pose
    pose = [startPosition,0];

    history = zeros(maxSteps+1,3);
    history(1,:) = pose;

    segmentLengths = sqrt(sum(diff(plannedPath).^2,2));
    cumulativeDistance = [0;cumsum(segmentLengths)];

    progressIndex = 1;
    stepsTaken = 0;

    goalReached = false;
    collisionDetected = false;

    %% Pure Pursuit loop

    for step = 1:maxSteps

        if norm(goalPosition-pose(1:2)) < goalTolerance
            goalReached = true;
            break;
        end

        % Find nearest point at or ahead of progress index
        remainingPath = plannedPath(progressIndex:end,:);

        distances = sqrt(sum( ...
            (remainingPath-pose(1:2)).^2,2));

        [~,localIndex] = min(distances);

        progressIndex = progressIndex+localIndex-1;

        % Select target by distance along path
        targetDistance = cumulativeDistance(progressIndex) ...
            + lookaheadDistance;

        targetIndex = find( ...
            cumulativeDistance>=targetDistance,1,'first');

        if isempty(targetIndex)
            targetIndex = size(plannedPath,1);
        end

        target = plannedPath(targetIndex,:);

        % Pure Pursuit controller
        dx = target(1)-pose(1);
        dy = target(2)-pose(2);

        alpha = atan2(dy,dx)-pose(3);
        alpha = atan2(sin(alpha),cos(alpha));

        actualLookahead = max(norm(target-pose(1:2)),0.001);

        curvature = 2*sin(alpha)/actualLookahead;

        angularSpeed = linearSpeed*curvature;

        angularSpeed = max(-maxAngularSpeed, ...
            min(maxAngularSpeed,angularSpeed));

        % Differential-drive kinematics
        leftSpeed = linearSpeed-angularSpeed*wheelBase/2;
        rightSpeed = linearSpeed+angularSpeed*wheelBase/2;

        actualLinearSpeed = (rightSpeed+leftSpeed)/2;
        actualAngularSpeed = (rightSpeed-leftSpeed)/wheelBase;

        nextPose = pose;

        nextPose(1) = pose(1) ...
            + actualLinearSpeed*cos(pose(3))*dt;

        nextPose(2) = pose(2) ...
            + actualLinearSpeed*sin(pose(3))*dt;

        nextPose(3) = pose(3)+actualAngularSpeed*dt;

        % Check movement segment for collisions
        previousPosition = pose(1:2);
        nextPosition = nextPose(1:2);

        movementDistance = norm(nextPosition-previousPosition);

        sampleCount = max(2, ...
            ceil(movementDistance/collisionSampleSpacing)+1);

        t = linspace(0,1,sampleCount)';

        movementPoints = previousPosition ...
            + t.*(nextPosition-previousPosition);

        occupancyValues = checkOccupancy( ...
            inflatedMap,movementPoints);

        if any(occupancyValues>0)
            collisionDetected = true;
            break;
        end

        % Accept movement
        pose = nextPose;

        history(step+1,:) = pose;
        stepsTaken = step;
    end

    % Check goal after the last accepted movement
    if norm(goalPosition-pose(1:2)) < goalTolerance
        goalReached = true;
    end

    history = history(1:stepsTaken+1,:);

    % Save run results
    goalReachedResults(test) = goalReached;
    collisionResults(test) = collisionDetected;

    allTrajectories{test} = history;

    completionTimes(test) = stepsTaken*dt;

    pathLengths(test) = sum(sqrt(sum( ...
        diff(history(:,1:2)).^2,2)));

    % Tracking error to planned path
    errors = zeros(size(history,1),1);

    for i = 1:size(history,1)
        d = sqrt(sum( ...
            (plannedPath-history(i,1:2)).^2,2));

        errors(i) = min(d);
    end

    meanErrors(test) = mean(errors);
    maxErrors(test) = max(errors);

    % Clearance from original occupied cells.
    % Distance is measured from the robot centre to
    % occupied cell centres; this is an approximation.
    occupiedPoints = occupancyMatrix(originalMap);
    [row,col] = find(occupiedPoints>0);

    obstacleCenters = grid2world(originalMap,[row,col]);

    clearanceValues = zeros(size(history,1),1);

    for i = 1:size(history,1)

        distances = sqrt(sum( ...
            (obstacleCenters-history(i,1:2)).^2,2));

        clearanceValues(i) = min(distances);
    end

    minimumClearances(test) = min(clearanceValues);

    fprintf('Goal reached: %s\n',mat2str(goalReached));
    fprintf('Collision detected: %s\n',mat2str(collisionDetected));
end

%% 6. Plot all trajectories

figure;
show(inflatedMap);
hold on;

for test = 1:numTests

    if ~isempty(allPlannedPaths{test})
        p = allPlannedPaths{test};

        plot(p(:,1),p(:,2),'--','LineWidth',1.2);
    end

    if ~isempty(allTrajectories{test})
        h = allTrajectories{test};

        plot(h(:,1),h(:,2),'-','LineWidth',2);
    end
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Experiment 10: Navigation Robustness');

hold off;
grid on;

%% 7. Results table

testNumber = (1:numTests)';

results = table(testNumber,goalReachedResults, ...
    collisionResults,completionTimes,pathLengths, ...
    meanErrors,maxErrors,minimumClearances, ...
    'VariableNames',{'Test','GoalReached', ...
    'CollisionDetected','Time_s','PathLength_m', ...
    'MeanError_m','MaxError_m','MinClearance_m'});

disp(' ');
disp('Experiment 10 Results');
disp(results);

%% 8. Overall summary

successfulTests = goalReachedResults & ~collisionResults;

fprintf('\nSuccessful tests: %d of %d\n', ...
    sum(successfulTests),numTests);

if all(successfulTests)
    fprintf('All navigation robustness tests passed.\n');
else
    fprintf('Some tests failed; inspect the results table.\n');
end
