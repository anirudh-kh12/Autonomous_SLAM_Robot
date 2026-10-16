
clc;
clear;
close all;

%% Experiment 15: ICP Scan Matching Robustness Validation

% This experiment compares ICP scan matching under three
% different initial alignment conditions.

%% 1. Create the ground-truth occupancy map

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
[obsX,obsY] = meshgrid(3.05:0.1:4.95, ...
                        2.05:0.1:3.95);

setOccupancy(map, [obsX(:),obsY(:)], 1);

% Obstacle 2
[obsX2,obsY2] = meshgrid(6.05:0.1:7.05, ...
                          4.05:0.1:6.05);

setOccupancy(map, [obsX2(:),obsY2(:)], 1);

%% 2. Configure the simulated LiDAR

numberOfBeams = 181;
fieldOfView = 2*pi;
maxRange = 8.0;
rayStep = 0.02;

angles = linspace(-fieldOfView/2, ...
                   fieldOfView/2,numberOfBeams);

%% 3. Define the two robot poses

% Pose format: [x, y, heading in radians]

pose1 = [1.0, 1.0, 0];

pose2 = [1.2143, 1.0638, deg2rad(5)];

%% 4. Generate LiDAR scans

scan1 = generateScan(map, pose1, angles, ...
                     rayStep, maxRange);

scan2 = generateScan(map, pose2, angles, ...
                     rayStep, maxRange);

if size(scan1,1) < 10 || size(scan2,1) < 10
    error('Insufficient LiDAR points for ICP matching.');
end

fprintf('Scan 1 points: %d\n', size(scan1,1));
fprintf('Scan 2 points: %d\n', size(scan2,1));

%% 5. Calculate the true relative transformation

% Transform from Scan 2 local coordinates into Scan 1
% local coordinates.

R1 = [cos(pose1(3)), -sin(pose1(3)); ...
      sin(pose1(3)),  cos(pose1(3))];

R2 = [cos(pose2(3)), -sin(pose2(3)); ...
      sin(pose2(3)),  cos(pose2(3))];

trueRotation = R1' * R2;

trueTranslation = R1' * ...
    (pose2(1:2)' - pose1(1:2)');

trueAngle = atan2(trueRotation(2,1), ...
                  trueRotation(1,1));

%% 6. Configure the ICP tests

% Each row contains initial estimate errors:
% [translation error X, translation error Y, angle error degrees]

initialErrors = [
    0.00,  0.00, 0;
    0.03, -0.02, 2;
    0.08, -0.05, 5
];

testNames = [
    "Accurate Initial Estimate";
    "Moderate Initial Error";
    "Larger Initial Error"
];

maxIterations = 100;
tolerance = 1e-6;
maxCorrespondenceDistance = 0.50;

numberOfTests = size(initialErrors,1);

translationErrors = zeros(numberOfTests,1);
rotationErrors = zeros(numberOfTests,1);
finalResiduals = zeros(numberOfTests,1);
iterationCounts = zeros(numberOfTests,1);
convergenceStatus = false(numberOfTests,1);

alignedScans = cell(numberOfTests,1);

%% 7. Run ICP for each initial estimate

for test = 1:numberOfTests

    fprintf('\nRunning test %d of %d: %s\n', ...
        test, numberOfTests, testNames(test));

    initialAngle = trueAngle + ...
        deg2rad(initialErrors(test,3));

    initialRotation = [
        cos(initialAngle), -sin(initialAngle);
        sin(initialAngle),  cos(initialAngle)
    ];

    initialTranslation = trueTranslation + ...
        initialErrors(test,1:2)';

    % Transform Scan 2 using the initial estimate
    source = (initialRotation * scan2')' + ...
        initialTranslation';

    target = scan1;

    previousError = inf;
    converged = false;

    for iteration = 1:maxIterations

        % Find nearest-neighbour correspondences
        distances = pdist2(source, target);

        [nearestDistances, nearestIndices] = ...
            min(distances, [], 2);

        valid = nearestDistances <= ...
            maxCorrespondenceDistance;

        matchedSource = source(valid,:);
        matchedTarget = target(nearestIndices(valid),:);

        if size(matchedSource,1) < 10
            warning('Test %d has too few valid correspondences.', test);
            break;
        end

        % Calculate centroids
        centroidSource = mean(matchedSource,1);
        centroidTarget = mean(matchedTarget,1);

        centeredSource = matchedSource - centroidSource;
        centeredTarget = matchedTarget - centroidTarget;

        % Estimate rotation using singular value decomposition
        H = centeredSource' * centeredTarget;

        [U,~,V] = svd(H);

        correction = eye(2);

        if det(V*U') < 0
            correction(2,2) = -1;
        end

        deltaRotation = V * correction * U';

        deltaTranslation = ...
            centroidTarget' - deltaRotation*centroidSource';

        % Update all source points
        source = (deltaRotation * source')' + ...
            deltaTranslation';

        % Calculate correspondence residual after update
        updatedDistances = pdist2(source, target);

        updatedNearestDistances = min(updatedDistances,[],2);

        updatedValid = updatedNearestDistances <= ...
            maxCorrespondenceDistance;

        if any(updatedValid)
            currentError = mean( ...
                updatedNearestDistances(updatedValid));
        else
            currentError = inf;
        end

        if abs(previousError-currentError) < tolerance
            converged = true;
            break;
        end

        previousError = currentError;
    end

    % Store the aligned scan and final metrics
    alignedScans{test} = source;

    % Estimate the final rigid transform using the final aligned points
    finalDistances = pdist2(source,target);

    [nearestDistances,nearestIndices] = ...
        min(finalDistances,[],2);

    finalValid = nearestDistances <= maxCorrespondenceDistance;

    finalResiduals(test) = mean(nearestDistances(finalValid));

    % Re-estimate the total transformation from the original scan
    % to the final aligned scan using corresponding points.
    originalSource = scan2;
    matchedSource = originalSource(finalValid,:);
    matchedTarget = target(nearestIndices(finalValid),:);

    centroidSource = mean(matchedSource,1);
    centroidTarget = mean(matchedTarget,1);

    H = (matchedSource-centroidSource)' * ...
        (matchedTarget-centroidTarget);

    [U,~,V] = svd(H);

    correction = eye(2);

    if det(V*U') < 0
        correction(2,2) = -1;
    end

    estimatedRotation = V*correction*U';

    estimatedTranslation = ...
        centroidTarget' - estimatedRotation*centroidSource';

    estimatedAngle = atan2(estimatedRotation(2,1), ...
                           estimatedRotation(1,1));

    % Translation error
    translationErrors(test) = ...
        norm(estimatedTranslation-trueTranslation);

    % Rotation error in degrees
    angleDifference = atan2( ...
        sin(estimatedAngle-trueAngle), ...
        cos(estimatedAngle-trueAngle));

    rotationErrors(test) = abs(rad2deg(angleDifference));

    iterationCounts(test) = iteration;
    convergenceStatus(test) = converged;

    fprintf('Iterations: %d\n',iteration);
    fprintf('Converged: %s\n',mat2str(converged));
    fprintf('Translation error: %.4f m\n', ...
        translationErrors(test));
    fprintf('Rotation error: %.4f degrees\n', ...
        rotationErrors(test));
    fprintf('Final mean residual: %.4f m\n', ...
        finalResiduals(test));
end

%% 8. Display the scans before matching

figure;

plot(scan1(:,1),scan1(:,2),'b.','MarkerSize',8);
hold on;

plot(scan2(:,1),scan2(:,2),'r.','MarkerSize',8);

xlabel('Local X Position (m)');
ylabel('Local Y Position (m)');
title('LiDAR Scans Before Matching');

legend('Scan 1','Scan 2','Location','best');
axis equal;
grid on;
hold off;

%% 9. Display the aligned scans

figure;

plot(scan1(:,1),scan1(:,2),'b.','MarkerSize',8);
hold on;

plot(alignedScans{1}(:,1),alignedScans{1}(:,2), ...
    'r.','MarkerSize',7);

plot(alignedScans{2}(:,1),alignedScans{2}(:,2), ...
    'g.','MarkerSize',6);

plot(alignedScans{3}(:,1),alignedScans{3}(:,2), ...
    'm.','MarkerSize',5);

xlabel('Scan 1 Frame X (m)');
ylabel('Scan 1 Frame Y (m)');
title('ICP Alignment Under Different Initial Errors');

legend('Reference Scan', ...
       'Accurate Initial Estimate', ...
       'Moderate Initial Error', ...
       'Larger Initial Error', ...
       'Location','best');

axis equal;
grid on;
hold off;

%% 10. Plot translation and rotation errors

figure;

subplot(1,2,1);

bar(translationErrors);

xticks(1:numberOfTests);
xticklabels({'Accurate','Moderate','Larger'});

ylabel('Translation Error (m)');
title('ICP Translation Error');
grid on;

subplot(1,2,2);

bar(rotationErrors);

xticks(1:numberOfTests);
xticklabels({'Accurate','Moderate','Larger'});

ylabel('Rotation Error (degrees)');
title('ICP Rotation Error');
grid on;

%% 11. Display summary results

results = table( ...
    testNames, ...
    convergenceStatus, ...
    iterationCounts, ...
    translationErrors, ...
    rotationErrors, ...
    finalResiduals, ...
    'VariableNames', { ...
        'Test', ...
        'Converged', ...
        'Iterations', ...
        'TranslationError_m', ...
        'RotationError_deg', ...
        'FinalResidual_m'});

fprintf('\nExperiment 15 Results\n\n');
disp(results);

fprintf('Experiment 15 simulation completed.\n');

%% Local function: Simulate a LiDAR scan

function scan = generateScan(map, pose, angles, rayStep, maxRange)

    scan = zeros(0,2);

    for b = 1:numel(angles)

        rayAngle = pose(3) + angles(b);
        hitDetected = false;

        for distance = rayStep:rayStep:maxRange

            pointX = pose(1) + distance*cos(rayAngle);
            pointY = pose(2) + distance*sin(rayAngle);

            if pointX < 0 || pointX >= map.XWorldLimits(2) || ...
               pointY < 0 || pointY >= map.YWorldLimits(2)
                break;
            end

            if checkOccupancy(map,[pointX,pointY]) > 0

                % Convert the world hit point to the robot's
                % local coordinate frame.
                dx = pointX-pose(1);
                dy = pointY-pose(2);

                localX = cos(pose(3))*dx + sin(pose(3))*dy;
                localY = -sin(pose(3))*dx + cos(pose(3))*dy;

                scan(end+1,:) = [localX,localY]; %#ok<AGROW>

                hitDetected = true;
                break;
            end
        end

        if ~hitDetected
            % No return for this beam.
        end
    end
end
