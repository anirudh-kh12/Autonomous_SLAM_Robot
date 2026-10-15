
clc;
clear;
close all;

%% Experiment 11: Footprint and Swept-Path Validation

%% 1. Parameters

mapWidth = 10;
mapHeight = 8;
resolution = 10;

% Provisional robot footprint
robotLength = 0.40;
robotWidth = 0.30;

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

originalMap = copy(map);

%% 3. Inflate obstacles for A* planning

% Use a conservative circular clearance based on the
% robot's half-diagonal plus the safety margin.
halfDiagonal = hypot(robotLength/2,robotWidth/2);

planningClearance = halfDiagonal + safetyMargin;

inflatedMap = copy(map);
inflate(inflatedMap,planningClearance);

%% 4. Plan route

startPosition = [1,1];
goalPosition = [8.5,6.5];

planner = plannerAStarGrid(inflatedMap);

plannedPath = plan(planner,startPosition, ...
    goalPosition,'world');

if isempty(plannedPath)
    error('A* could not find a valid path.');
end

if any(checkOccupancy(inflatedMap,plannedPath)>0)
    error('Planned path contains occupied points.');
end

fprintf('A* planning successful.\n');
fprintf('Planned waypoints: %d\n',size(plannedPath,1));
fprintf('Robot footprint: %.2f m x %.2f m\n', ...
    robotLength,robotWidth);
fprintf('Planning clearance: %.3f m\n',planningClearance);

%% 5. Initialize robot

% Pose = [x, y, heading]
pose = [startPosition,0];

robotHistory = zeros(maxSteps+1,3);
robotHistory(1,:) = pose;

stepsTaken = 0;
goalReached = false;
footprintCollision = false;

collisionPose = [NaN,NaN,NaN];

%% 6. Follow the path

segmentLengths = sqrt(sum(diff(plannedPath).^2,2));
cumulativeDistance = [0;cumsum(segmentLengths)];

progressIndex = 1;

for step = 1:maxSteps

    % Check goal
    if norm(goalPosition-pose(1:2)) < goalTolerance
        goalReached = true;
        break;
    end

    % Find nearest point at or ahead of progress
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
    alpha = atan2(target(2)-pose(2), ...
                  target(1)-pose(1))-pose(3);

    alpha = atan2(sin(alpha),cos(alpha));

    actualLookahead = max(norm(target-pose(1:2)),0.001);

    curvature = 2*sin(alpha)/actualLookahead;

    angularSpeed = linearSpeed*curvature;

    angularSpeed = max(-maxAngularSpeed, ...
        min(maxAngularSpeed,angularSpeed));

    % Differential-drive velocities
    leftSpeed = linearSpeed-angularSpeed*wheelBase/2;
    rightSpeed = linearSpeed+angularSpeed*wheelBase/2;

    actualLinearSpeed = (rightSpeed+leftSpeed)/2;
    actualAngularSpeed = (rightSpeed-leftSpeed)/wheelBase;

    % Predict next pose
    nextPose = pose;

    nextPose(1) = pose(1) ...
        + actualLinearSpeed*cos(pose(3))*dt;

    nextPose(2) = pose(2) ...
        + actualLinearSpeed*sin(pose(3))*dt;

    nextPose(3) = pose(3)+actualAngularSpeed*dt;

    % Check the swept movement using interpolated poses
    movementDistance = norm(nextPose(1:2)-pose(1:2));

    translationSamples = max(2, ...
        ceil(movementDistance/collisionSampleSpacing)+1);

    % Also sample rotation so the footprint cannot rotate
    % through an obstacle between checked poses.
    angleChange = abs(atan2( ...
        sin(nextPose(3)-pose(3)), ...
        cos(nextPose(3)-pose(3))));

    rotationSamples = max(2, ...
        ceil(angleChange/deg2rad(2))+1);

    sampleCount = max(translationSamples,rotationSamples);

    collisionThisStep = false;

    for j = 1:sampleCount

        fraction = (j-1)/(sampleCount-1);

        testPose = pose + fraction*(nextPose-pose);

        % Interpolate heading using the shortest angle
        headingDifference = atan2( ...
            sin(nextPose(3)-pose(3)), ...
            cos(nextPose(3)-pose(3)));

        testPose(3) = pose(3)+fraction*headingDifference;

        if footprintOccupied(originalMap,testPose, ...
                robotLength,robotWidth)

            collisionThisStep = true;
            collisionPose = testPose;
            break;
        end
    end

    if collisionThisStep
        footprintCollision = true;

        fprintf(['Footprint collision detected near ' ...
            '(%.2f, %.2f) m.\n'], ...
            collisionPose(1),collisionPose(2));

        break;
    end

    % Accept movement
    pose = nextPose;

    robotHistory(step+1,:) = pose;
    stepsTaken = step;
end

%% 7. Final goal check

if norm(goalPosition-pose(1:2)) < goalTolerance
    goalReached = true;
end

robotHistory = robotHistory(1:stepsTaken+1,:);

%% 8. Calculate tracking error

trackingError = zeros(size(robotHistory,1),1);

for i = 1:size(robotHistory,1)

    distances = sqrt(sum( ...
        (plannedPath-robotHistory(i,1:2)).^2,2));

    trackingError(i) = min(distances);
end

meanError = mean(trackingError);
maxError = max(trackingError);

trajectoryLength = sum(sqrt(sum( ...
    diff(robotHistory(:,1:2)).^2,2)));

%% 9. Plot map, path and robot footprint

figure;

show(originalMap);
hold on;

plot(plannedPath(:,1),plannedPath(:,2), ...
    'b--','LineWidth',1.5);

plot(robotHistory(:,1),robotHistory(:,2), ...
    'g-','LineWidth',2);

plot(startPosition(1),startPosition(2), ...
    'go','MarkerFaceColor','g','MarkerSize',9);

plot(goalPosition(1),goalPosition(2), ...
    'rp','MarkerFaceColor','r','MarkerSize',12);

% Draw selected robot footprints along the trajectory
footprintSpacing = max(1, ...
    floor(size(robotHistory,1)/15));

for i = 1:footprintSpacing:size(robotHistory,1)

    rectangleCorners = getFootprintCorners( ...
        robotHistory(i,:),robotLength,robotWidth);

    plot([rectangleCorners(:,1);rectangleCorners(1,1)], ...
         [rectangleCorners(:,2);rectangleCorners(1,2)], ...
         'm-','LineWidth',1);
end

% Draw the collision footprint, if one was detected
if footprintCollision

    corners = getFootprintCorners( ...
        collisionPose,robotLength,robotWidth);

    plot([corners(:,1);corners(1,1)], ...
         [corners(:,2);corners(1,2)], ...
         'r-','LineWidth',2);
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Footprint and Swept-Path Validation');

legend('Original Map','A* Planned Path', ...
    'Robot Trajectory','Start','Goal', ...
    'Robot Footprint','Location','best');

axis equal;
hold off;

%% 10. Plot tracking error

figure;

time = (0:size(robotHistory,1)-1)'*dt;

plot(time,trackingError,'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Tracking Error (m)');
title('Footprint Validation Tracking Error');

grid on;

%% 11. Display results

fprintf('\nExperiment 11 Results\n');

fprintf('Robot length: %.2f m\n',robotLength);
fprintf('Robot width: %.2f m\n',robotWidth);
fprintf('Planning clearance: %.3f m\n',planningClearance);
fprintf('Lookahead distance: %.2f m\n',lookaheadDistance);
fprintf('Simulation time: %.2f s\n',stepsTaken*dt);
fprintf('Trajectory points: %d\n',size(robotHistory,1));
fprintf('Trajectory length: %.3f m\n',trajectoryLength);
fprintf('Mean tracking error: %.3f m\n',meanError);
fprintf('Maximum tracking error: %.3f m\n',maxError);
fprintf('Final position: (%.2f, %.2f) m\n', ...
    pose(1),pose(2));

fprintf('Goal reached: %s\n',mat2str(goalReached));
fprintf('Footprint collision detected: %s\n', ...
    mat2str(footprintCollision));

if goalReached && ~footprintCollision
    fprintf('Footprint validation: PASSED\n');
elseif footprintCollision
    fprintf('Footprint validation: FAILED - collision detected\n');
else
    fprintf('Footprint validation: INCOMPLETE - goal not reached\n');
end

%% Local helper functions

function corners = getFootprintCorners(pose,lengthValue,widthValue)

    halfLength = lengthValue/2;
    halfWidth = widthValue/2;

    % Rectangle corners in robot-local coordinates
    localCorners = [
        -halfLength,-halfWidth;
         halfLength,-halfWidth;
         halfLength, halfWidth;
        -halfLength, halfWidth
    ];

    c = cos(pose(3));
    s = sin(pose(3));

    rotationMatrix = [c,-s;s,c];

    corners = localCorners*rotationMatrix' + pose(1:2);
end

function occupied = footprintOccupied(map,pose,lengthValue,widthValue)

    % Conservative check using occupied cell centres.
    % Expand the rectangle by half a cell diagonal to account
    % approximately for the finite size of occupied cells.

    resolution = map.Resolution;
    cellPadding = sqrt(2)/(2*resolution);

    halfLength = lengthValue/2 + cellPadding;
    halfWidth = widthValue/2 + cellPadding;

    occupancy = occupancyMatrix(map);

    [rows,cols] = find(occupancy>0);

    if isempty(rows)
        occupied = false;
        return;
    end

    % Convert occupied cell indices to world coordinates
    obstacleCenters = grid2world(map,[rows,cols]);

    % Transform obstacle centres into robot-local coordinates
    delta = obstacleCenters-pose(1:2);

    c = cos(pose(3));
    s = sin(pose(3));

    localX = c*delta(:,1)+s*delta(:,2);
    localY = -s*delta(:,1)+c*delta(:,2);

    insideFootprint = ...
        abs(localX)<=halfLength & abs(localY)<=halfWidth;

    occupied = any(insideFootprint);
end
