
clc;
clear;
close all;

%% Experiment 16: ICP with Noise and Outliers

% Compare ICP scan matching using:
% 1. Clean LiDAR scans
% 2. Noisy LiDAR scans
% 3. Noisy scans with outlier points

%% 1. Create the ground-truth map

mapWidth = 10;
mapHeight = 8;
resolution = 10;

map = binaryOccupancyMap(mapWidth,mapHeight,resolution);

cellSize = 1/resolution;

xCells = 0.05:cellSize:mapWidth-0.05;
yCells = 0.05:cellSize:mapHeight-0.05;

% Boundary walls
setOccupancy(map,[xCells(:), ...
    0.05*ones(numel(xCells),1)],1);

setOccupancy(map,[xCells(:), ...
    (mapHeight-0.05)*ones(numel(xCells),1)],1);

setOccupancy(map,[0.05*ones(numel(yCells),1), ...
    yCells(:)],1);

setOccupancy(map,[(mapWidth-0.05)*ones(numel(yCells),1), ...
    yCells(:)],1);

% Obstacle 1
[obsX,obsY] = meshgrid(3.05:0.1:4.95, ...
                        2.05:0.1:3.95);

setOccupancy(map,[obsX(:),obsY(:)],1);

% Obstacle 2
[obsX2,obsY2] = meshgrid(6.05:0.1:7.05, ...
                          4.05:0.1:6.05);

setOccupancy(map,[obsX2(:),obsY2(:)],1);

%% 2. Configure simulated LiDAR

numberOfBeams = 181;
fieldOfView = 2*pi;
maxRange = 8.0;
rayStep = 0.02;

angles = linspace(-fieldOfView/2, ...
                   fieldOfView/2,numberOfBeams);

%% 3. Define robot poses

% Pose format: [x, y, heading in radians]

pose1 = [1.0,1.0,0];

pose2 = [1.2143,1.0638,deg2rad(5)];

%% 4. Generate clean LiDAR scans

scan1 = generateScan(map,pose1,angles,rayStep,maxRange);
scan2 = generateScan(map,pose2,angles,rayStep,maxRange);

if size(scan1,1) < 10 || size(scan2,1) < 10
    error('Insufficient LiDAR points for ICP.');
end

%% 5. Calculate the true relative transformation

R1 = [cos(pose1(3)),-sin(pose1(3)); ...
      sin(pose1(3)), cos(pose1(3))];

R2 = [cos(pose2(3)),-sin(pose2(3)); ...
      sin(pose2(3)), cos(pose2(3))];

trueRotation = R1' * R2;

trueTranslation = R1' * ...
    (pose2(1:2)' - pose1(1:2)');

trueAngle = atan2(trueRotation(2,1), ...
                  trueRotation(1,1));

%% 6. Create noisy and outlier-contaminated scans

rng(16,'twister');

% Measurement noise standard deviation in metres
noiseStd = 0.02;

% Add independent Gaussian noise to each coordinate
noisyScan2 = scan2 + noiseStd*randn(size(scan2));

% Add points that do not correspond to actual LiDAR surfaces
numberOfOutliers = round(0.10*size(noisyScan2,1));

outliers = [
    -2 + 8*rand(numberOfOutliers,1), ...
    -2 + 10*rand(numberOfOutliers,1)
];

noisyOutlierScan2 = [noisyScan2;outliers];

fprintf('Clean Scan 1 points: %d\n',size(scan1,1));
fprintf('Clean Scan 2 points: %d\n',size(scan2,1));
fprintf('Noise standard deviation: %.3f m\n',noiseStd);
fprintf('Added outlier points: %d\n',numberOfOutliers);
fprintf('Scan 2 points with outliers: %d\n', ...
    size(noisyOutlierScan2,1));

%% 7. Configure ICP tests

testNames = [
    "Clean Scan";
    "Noisy Scan";
    "Noisy Scan + Outliers"
];

sourceScans = {
    scan2;
    noisyScan2;
    noisyOutlierScan2
};

numberOfTests = numel(sourceScans);

% Initial transformation deliberately has a small error
initialAngle = trueAngle + deg2rad(1.0);

initialRotation = [
    cos(initialAngle),-sin(initialAngle);
    sin(initialAngle), cos(initialAngle)
];

initialTranslation = trueTranslation + [0.02;-0.015];

maxIterations = 100;
tolerance = 1e-6;
maxCorrespondenceDistance = 0.50;

translationErrors = zeros(numberOfTests,1);
rotationErrors = zeros(numberOfTests,1);
finalResiduals = zeros(numberOfTests,1);
iterationCounts = zeros(numberOfTests,1);
convergenceStatus = false(numberOfTests,1);

alignedScans = cell(numberOfTests,1);

%% 8. Run ICP for each test

for test = 1:numberOfTests

    fprintf('\nRunning test %d of %d: %s\n', ...
        test,numberOfTests,testNames(test));

    originalSource = sourceScans{test};

    % Apply the same initial estimate to each scan
    source = (initialRotation*originalSource')' + ...
        initialTranslation';

    totalRotation = initialRotation;
    totalTranslation = initialTranslation;

    previousError = inf;
    converged = false;

    for iteration = 1:maxIterations

        % Find nearest neighbours
        distances = pdist2(source,scan1);

        [nearestDistances,nearestIndices] = ...
            min(distances,[],2);

        % Reject correspondences that are too far apart
        valid = nearestDistances <= maxCorrespondenceDistance;

        matchedSource = source(valid,:);
        matchedTarget = scan1(nearestIndices(valid),:);

        if size(matchedSource,1) < 10
            warning('Test %d has too few valid correspondences.',test);
            break;
        end

        % Calculate centroids
        centroidSource = mean(matchedSource,1);
        centroidTarget = mean(matchedTarget,1);

        centeredSource = matchedSource-centroidSource;
        centeredTarget = matchedTarget-centroidTarget;

        % Estimate incremental rotation
        H = centeredSource'*centeredTarget;

        [U,~,V] = svd(H);

        correction = eye(2);

        if det(V*U') < 0
            correction(2,2) = -1;
        end

        deltaRotation = V*correction*U';

        deltaTranslation = ...
            centroidTarget' - deltaRotation*centroidSource';

        % Update source points
        source = (deltaRotation*source')' + ...
            deltaTranslation';

        % Accumulate the complete transformation
        totalTranslation = ...
            deltaRotation*totalTranslation + deltaTranslation;

        totalRotation = deltaRotation*totalRotation;

        % Calculate the new mean nearest-neighbour residual
        updatedDistances = pdist2(source,scan1);

        updatedNearestDistances = min(updatedDistances,[],2);

        updatedValid = updatedNearestDistances <= ...
            maxCorrespondenceDistance;

        if ~any(updatedValid)
            break;
        end

        currentError = mean( ...
            updatedNearestDistances(updatedValid));

        if abs(previousError-currentError) < tolerance
            converged = true;
            break;
        end

        previousError = currentError;
    end

    % Store results
    alignedScans{test} = source;

    finalDistances = pdist2(source,scan1);
    finalNearestDistances = min(finalDistances,[],2);

    finalValid = finalNearestDistances <= ...
        maxCorrespondenceDistance;

    if any(finalValid)
        finalResiduals(test) = ...
            mean(finalNearestDistances(finalValid));
    else
        finalResiduals(test) = NaN;
    end

    % Compare estimated transformation with ground truth
    translationErrors(test) = ...
        norm(totalTranslation-trueTranslation);

    estimatedAngle = atan2(totalRotation(2,1), ...
                           totalRotation(1,1));

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

%% 9. Figure 1: Compare scans before matching

figure;

plot(scan1(:,1),scan1(:,2),'b.','MarkerSize',8);
hold on;

plot(scan2(:,1),scan2(:,2),'r.','MarkerSize',8);

plot(noisyScan2(:,1),noisyScan2(:,2), ...
    'g.','MarkerSize',5);

plot(noisyOutlierScan2(:,1),noisyOutlierScan2(:,2), ...
    'm.','MarkerSize',4);

xlabel('Local X Position (m)');
ylabel('Local Y Position (m)');
title('Clean, Noisy and Outlier-Contaminated Scans');

legend('Reference Scan','Clean Scan 2', ...
    'Noisy Scan 2','Noisy Scan 2 with Outliers', ...
    'Location','best');

axis equal;
grid on;
hold off;

%% 10. Figure 2: Compare aligned scans

figure;

plot(scan1(:,1),scan1(:,2),'b.','MarkerSize',8);
hold on;

plot(alignedScans{1}(:,1),alignedScans{1}(:,2), ...
    'r.','MarkerSize',6);

plot(alignedScans{2}(:,1),alignedScans{2}(:,2), ...
    'g.','MarkerSize',5);

plot(alignedScans{3}(:,1),alignedScans{3}(:,2), ...
    'm.','MarkerSize',4);

xlabel('Scan 1 Frame X (m)');
ylabel('Scan 1 Frame Y (m)');
title('ICP Alignment with Noise and Outliers');

legend('Reference Scan','Clean Scan','Noisy Scan', ...
    'Noisy Scan + Outliers','Location','best');

axis equal;
grid on;
hold off;

%% 11. Figure 3: Compare errors

figure;

subplot(1,2,1);

bar(translationErrors);

xticks(1:numberOfTests);
xticklabels({'Clean','Noisy','Noise + Outliers'});

ylabel('Translation Error (m)');
title('Translation Error Comparison');
grid on;

subplot(1,2,2);

bar(rotationErrors);

xticks(1:numberOfTests);
xticklabels({'Clean','Noisy','Noise + Outliers'});

ylabel('Rotation Error (degrees)');
title('Rotation Error Comparison');
grid on;

%% 12. Display results

results = table( ...
    testNames, ...
    convergenceStatus, ...
    iterationCounts, ...
    translationErrors, ...
    rotationErrors, ...
    finalResiduals, ...
    'VariableNames',{ ...
        'Test','Converged','Iterations', ...
        'TranslationError_m','RotationError_deg', ...
        'FinalResidual_m'});

fprintf('\nExperiment 16 Results\n\n');
disp(results);

fprintf('Experiment 16 simulation completed.\n');

%% Local function: Generate a simulated LiDAR scan

function scan = generateScan(map,pose,angles,rayStep,maxRange)

    scan = zeros(0,2);

    for b = 1:numel(angles)

        rayAngle = pose(3)+angles(b);
        hitDetected = false;

        for distance = rayStep:rayStep:maxRange

            pointX = pose(1)+distance*cos(rayAngle);
            pointY = pose(2)+distance*sin(rayAngle);

            if pointX < 0 || pointX >= 10 || ...
               pointY < 0 || pointY >= 8
                break;
            end

            if checkOccupancy(map,[pointX,pointY]) > 0

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
            % This beam produced no obstacle return.
        end
    end
end
