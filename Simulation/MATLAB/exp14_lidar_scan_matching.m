
clc;
clear;
close all;

%% Experiment 14: LiDAR Scan Matching and Relative Pose Estimation

%% 1. Create ground-truth map

mapWidth = 10;
mapHeight = 8;
resolution = 10;
cellSize = 1 / resolution;

map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

xCells = 0.05:cellSize:mapWidth-0.05;
yCells = 0.05:cellSize:mapHeight-0.05;

% Boundary walls
setOccupancy(map, [xCells(:), ...
    0.05*ones(numel(xCells),1)], 1);

setOccupancy(map, [xCells(:), ...
    (mapHeight-0.05)*ones(numel(xCells),1)], 1);

setOccupancy(map, [0.05*ones(numel(yCells),1), ...
    yCells(:)], 1);

setOccupancy(map, [(mapWidth-0.05)*ones(numel(yCells),1), ...
    yCells(:)], 1);

% Obstacle 1
[obsX, obsY] = meshgrid(3.05:0.1:4.95, ...
                         2.05:0.1:3.95);

setOccupancy(map, [obsX(:), obsY(:)], 1);

% Obstacle 2
[obsX2, obsY2] = meshgrid(6.05:0.1:7.05, ...
                           4.05:0.1:6.05);

setOccupancy(map, [obsX2(:), obsY2(:)], 1);

%% 2. Define two known robot poses

% Pose format: [x, y, heading in radians]
pose1 = [1.5, 1.0, deg2rad(10)];
pose2 = [1.7, 1.1, deg2rad(15)];

% Actual relative movement expressed in pose 1's frame
rotation1 = [cos(pose1(3)), -sin(pose1(3)); ...
             sin(pose1(3)),  cos(pose1(3))];

actualTranslation = ...
    rotation1' * (pose2(1:2)' - pose1(1:2)');

actualRotation = pose2(3) - pose1(3);

%% 3. Configure simulated LiDAR

numberOfBeams = 361;
fieldOfView = 2*pi;
maxRange = 8.0;
rayStep = 0.02;

angles = linspace(-fieldOfView/2, ...
                   fieldOfView/2, numberOfBeams);

%% 4. Generate scans in each robot's local frame

scan1 = simulateScan(map, pose1, angles, maxRange, rayStep);
scan2 = simulateScan(map, pose2, angles, maxRange, rayStep);

fprintf('Scan 1 points: %d\n', size(scan1,1));
fprintf('Scan 2 points: %d\n', size(scan2,1));

if size(scan1,1) < 20 || size(scan2,1) < 20
    error('Not enough LiDAR points. Check the poses and map.');
end

%% 5. Match scan 2 to scan 1 using 2D ICP

% Transform scan 2 points into scan 1's coordinate frame.
% Start with an initial estimate of zero translation and rotation.
estimatedRotation = 0;
estimatedTranslation = [0; 0];

maxIterations = 60;
maxCorrespondenceDistance = 0.75;
convergenceTolerance = 1e-5;

previousError = inf;
validMatch = false;

for iteration = 1:maxIterations

    % Transform the second scan using the current estimate
    rotationMatrix = [cos(estimatedRotation), ...
                     -sin(estimatedRotation); ...
                      sin(estimatedRotation), ...
                      cos(estimatedRotation)];

    transformedScan2 = ...
        (rotationMatrix * scan2' + estimatedTranslation)';

    % Find the closest scan 1 point for every transformed scan 2 point
    nearestPoints = zeros(size(transformedScan2));
    nearestDistances = zeros(size(transformedScan2,1),1);

    for i = 1:size(transformedScan2,1)

        differences = scan1 - transformedScan2(i,:);
        distances = sqrt(sum(differences.^2,2));

        [nearestDistances(i), nearestIndex] = min(distances);
        nearestPoints(i,:) = scan1(nearestIndex,:);
    end

    % Reject distant, unreliable point correspondences
    inliers = nearestDistances < maxCorrespondenceDistance;

    sourcePoints = transformedScan2(inliers,:);
    targetPoints = nearestPoints(inliers,:);

    if size(sourcePoints,1) < 20
        error('Too few valid scan correspondences. Try adjusting the poses.');
    end

    % Calculate centroids
    sourceCentroid = mean(sourcePoints,1);
    targetCentroid = mean(targetPoints,1);

    sourceCentered = sourcePoints - sourceCentroid;
    targetCentered = targetPoints - targetCentroid;

    % Estimate rotation using the 2D SVD method
    H = sourceCentered' * targetCentered;
    [U,~,V] = svd(H);

    correction = eye(2);

    if det(V * U') < 0
        correction(2,2) = -1;
    end

    deltaRotationMatrix = V * correction * U';

    deltaRotation = atan2( ...
        deltaRotationMatrix(2,1), ...
        deltaRotationMatrix(1,1));

    % Estimate translation for the incremental transform
    deltaTranslation = ...
        targetCentroid' - deltaRotationMatrix * sourceCentroid';

    % Apply the incremental transform to the current estimate
    estimatedTranslation = ...
        deltaRotationMatrix * estimatedTranslation + deltaTranslation;

    estimatedRotation = estimatedRotation + deltaRotation;

    % Normalize estimated angle to [-pi, pi]
    estimatedRotation = atan2( ...
        sin(estimatedRotation), cos(estimatedRotation));

    % Calculate the current alignment error
    rotationMatrix = [cos(estimatedRotation), ...
                     -sin(estimatedRotation); ...
                      sin(estimatedRotation), ...
                      cos(estimatedRotation)];

    transformedScan2 = ...
        (rotationMatrix * scan2' + estimatedTranslation)';

    currentError = mean(nearestDistances(inliers));

    validMatch = true;

    if abs(previousError - currentError) < convergenceTolerance
        break;
    end

    previousError = currentError;
end

if ~validMatch
    error('Scan matching did not produce a valid estimate.');
end

%% 6. Calculate pose estimation errors

translationError = norm( ...
    estimatedTranslation - actualTranslation);

rotationError = atan2( ...
    sin(estimatedRotation - actualRotation), ...
    cos(estimatedRotation - actualRotation));

rotationErrorDegrees = abs(rad2deg(rotationError));

%% 7. Transform both scans into a common frame

rotationMatrix = [cos(estimatedRotation), ...
                 -sin(estimatedRotation); ...
                  sin(estimatedRotation), ...
                  cos(estimatedRotation)];

alignedScan2 = ...
    (rotationMatrix * scan2' + estimatedTranslation)';

%% 8. Plot the two scans before alignment

figure;

plot(scan1(:,1), scan1(:,2), 'b.', 'MarkerSize', 6);
hold on;

plot(scan2(:,1), scan2(:,2), 'r.', 'MarkerSize', 6);

axis equal;
grid on;

xlabel('Local X Position (m)');
ylabel('Local Y Position (m)');
title('LiDAR Scans Before Matching');

legend('Scan 1', 'Scan 2', 'Location', 'best');
hold off;

%% 9. Plot scans after alignment

figure;

plot(scan1(:,1), scan1(:,2), 'b.', 'MarkerSize', 7);
hold on;

plot(alignedScan2(:,1), alignedScan2(:,2), ...
    'r.', 'MarkerSize', 7);

axis equal;
grid on;

xlabel('Scan 1 Frame X (m)');
ylabel('Scan 1 Frame Y (m)');
title('ICP Scan Matching Result');

legend('Reference Scan', 'Aligned Scan 2', ...
    'Location', 'best');
hold off;

%% 10. Display results

fprintf('\nExperiment 14 Results\n');

fprintf('ICP iterations: %d\n', iteration);
fprintf('Actual relative translation: (%.4f, %.4f) m\n', ...
    actualTranslation(1), actualTranslation(2));

fprintf('Estimated relative translation: (%.4f, %.4f) m\n', ...
    estimatedTranslation(1), estimatedTranslation(2));

fprintf('Translation estimation error: %.4f m\n', ...
    translationError);

fprintf('Actual relative rotation: %.3f degrees\n', ...
    rad2deg(actualRotation));

fprintf('Estimated relative rotation: %.3f degrees\n', ...
    rad2deg(estimatedRotation));

fprintf('Rotation estimation error: %.3f degrees\n', ...
    rotationErrorDegrees);

fprintf('Final mean nearest-neighbour distance: %.4f m\n', ...
    currentError);

fprintf('\nExperiment 14 simulation completed.\n');

%% Local function: Simulate a 2D LiDAR scan

function scanPoints = simulateScan(map, pose, angles, maxRange, rayStep)

    scanPoints = zeros(0,2);

    for b = 1:numel(angles)

        rayAngle = pose(3) + angles(b);
        hitDetected = false;

        for distance = rayStep:rayStep:maxRange

            pointX = pose(1) + distance*cos(rayAngle);
            pointY = pose(2) + distance*sin(rayAngle);

            if pointX < 0 || pointX >= 10 || ...
               pointY < 0 || pointY >= 8
                break;
            end

            if checkOccupancy(map, [pointX, pointY]) > 0

                % Convert the hit from world coordinates to
                % the robot's local coordinate frame.
                dx = pointX - pose(1);
                dy = pointY - pose(2);

                localX = cos(pose(3))*dx + sin(pose(3))*dy;
                localY = -sin(pose(3))*dx + cos(pose(3))*dy;

                scanPoints(end+1,:) = ...
                    [localX, localY]; %#ok<AGROW>

                hitDetected = true;
                break;
            end
        end

        if ~hitDetected
            % Ignore rays that do not hit an obstacle.
        end
    end
end
