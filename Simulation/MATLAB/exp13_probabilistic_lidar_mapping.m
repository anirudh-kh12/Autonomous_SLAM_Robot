
clc;
clear;
close all;

%% Experiment 13: Probabilistic LiDAR Occupancy Mapping

%% 1. Map parameters

mapWidth = 10;
mapHeight = 8;
resolution = 10;

cellSize = 1/resolution;

numRows = mapHeight*resolution;
numCols = mapWidth*resolution;

%% 2. Create ground-truth map

trueMap = binaryOccupancyMap( ...
    mapWidth,mapHeight,resolution);

xCells = 0.05:cellSize:mapWidth-0.05;
yCells = 0.05:cellSize:mapHeight-0.05;

% Boundary walls
setOccupancy(trueMap,[xCells(:), ...
    0.05*ones(numel(xCells),1)],1);

setOccupancy(trueMap,[xCells(:), ...
    (mapHeight-0.05)*ones(numel(xCells),1)],1);

setOccupancy(trueMap,[0.05*ones(numel(yCells),1), ...
    yCells(:)],1);

setOccupancy(trueMap,[(mapWidth-0.05)*ones(numel(yCells),1), ...
    yCells(:)],1);

% Obstacle 1
[obsX,obsY] = meshgrid(3.05:0.1:4.95, ...
                        2.05:0.1:3.95);

setOccupancy(trueMap,[obsX(:),obsY(:)],1);

% Obstacle 2
[obsX2,obsY2] = meshgrid(6.05:0.1:7.05, ...
                          4.05:0.1:6.05);

setOccupancy(trueMap,[obsX2(:),obsY2(:)],1);

%% 3. Configure simulated LiDAR

numberOfBeams = 181;
fieldOfView = 2*pi;
maxRange = 8.0;
rayStep = 0.02;

angles = linspace(-fieldOfView/2, ...
                   fieldOfView/2,numberOfBeams);

%% 4. Define robot poses

% Each row is [x, y, heading in radians]
robotPoses = [
    1.0, 1.0, 0;
    2.0, 5.5, 0;
    5.0, 1.0, pi/2;
    8.5, 2.0, pi/2;
    8.5, 6.8, pi;
    5.0, 6.8, pi;
    1.5, 6.5, -pi/2
];

%% 5. Initialize log-odds grid and observation mask

% Zero log-odds corresponds to probability 0.5.
logOdds = zeros(numRows,numCols);

% Track cells traversed by LiDAR rays independently
% of their occupancy classification.
observedMask = false(numRows,numCols);

% Evidence update strengths
freeUpdate = -0.40;
occupiedUpdate = 0.85;

% Limit accumulated evidence
minLogOdds = -4;
maxLogOdds = 4;

allScanEndpoints = zeros(0,2);

%% 6. Simulate LiDAR and accumulate evidence

for p = 1:size(robotPoses,1)

    pose = robotPoses(p,:);
    scanEndpoints = zeros(0,2);

    for b = 1:numberOfBeams

        rayAngle = pose(3)+angles(b);
        hitDetected = false;

        for distance = rayStep:rayStep:maxRange

            pointX = pose(1)+distance*cos(rayAngle);
            pointY = pose(2)+distance*sin(rayAngle);

            % Stop when the ray leaves the map
            if pointX < 0 || pointX >= mapWidth || ...
               pointY < 0 || pointY >= mapHeight
                break;
            end

            % Convert world coordinates to grid indices
            col = floor(pointX*resolution)+1;
            row = floor(pointY*resolution)+1;

            col = max(1,min(numCols,col));
            row = max(1,min(numRows,row));

            % Record that the ray visited this cell
            observedMask(row,col) = true;

            if checkOccupancy(trueMap,[pointX,pointY]) > 0

                % LiDAR hit: add occupied evidence
                logOdds(row,col) = min(maxLogOdds, ...
                    logOdds(row,col)+occupiedUpdate);

                scanEndpoints(end+1,:) = ...
                    [pointX,pointY]; %#ok<SAGROW>

                allScanEndpoints(end+1,:) = ...
                    [pointX,pointY]; %#ok<SAGROW>

                hitDetected = true;
                break;

            else

                % Ray passed through this cell:
                % add free-space evidence
                logOdds(row,col) = max(minLogOdds, ...
                    logOdds(row,col)+freeUpdate);
            end
        end

        if ~hitDetected
            % Cells traversed before leaving the map or
            % reaching maxRange retain free-space evidence.
        end
    end

    fprintf('Pose %d: %d LiDAR returns.\n', ...
        p,size(scanEndpoints,1));
end

%% 7. Convert log-odds to occupancy probabilities

occupancyProbability = ...
    1 ./ (1 + exp(-logOdds));

% Classification thresholds
freeThreshold = 0.35;
occupiedThreshold = 0.65;

% A cell is unknown only if no LiDAR ray visited it
unknownMask = ~observedMask;

% Classify observed cells
freeMask = observedMask & ...
    (occupancyProbability < freeThreshold);

occupiedMask = observedMask & ...
    (occupancyProbability > occupiedThreshold);

% Observed cells with insufficient evidence for either
% definite classification are marked uncertain.
uncertainMask = observedMask & ...
    (occupancyProbability >= freeThreshold) & ...
    (occupancyProbability <= occupiedThreshold);

%% 8. Evaluate mapping against ground truth

groundTruthGrid = occupancyMatrix(trueMap) > 0;

totalCells = numel(logOdds);

observedCells = sum(observedMask(:));
unknownCells = sum(unknownMask(:));
freeCells = sum(freeMask(:));
occupiedCells = sum(occupiedMask(:));
uncertainCells = sum(uncertainMask(:));

coveragePercent = 100*observedCells/totalCells;

% Evaluate agreement only among observed cells with a
% definite free or occupied classification.
classifiedMask = freeMask | occupiedMask;

predictedOccupied = occupiedMask(classifiedMask);
truthOccupied = groundTruthGrid(classifiedMask);

if ~isempty(predictedOccupied)
    observedAgreement = ...
        100*mean(predictedOccupied == truthOccupied);
else
    observedAgreement = NaN;
end

% Confusion counts on definitely classified cells
truePositive = sum(predictedOccupied & truthOccupied);
falsePositive = sum(predictedOccupied & ~truthOccupied);
falseNegative = sum(~predictedOccupied & truthOccupied);

% Occupied-cell precision
if truePositive+falsePositive > 0
    occupiedPrecision = ...
        truePositive/(truePositive+falsePositive);
else
    occupiedPrecision = NaN;
end

% Occupied-cell recall on the evaluated cells
if truePositive+falseNegative > 0
    occupiedRecall = ...
        truePositive/(truePositive+falseNegative);
else
    occupiedRecall = NaN;
end

% Ground-truth occupied-cell coverage
totalGroundTruthOccupied = sum(groundTruthGrid(:));

observedGroundTruthOccupied = ...
    sum(groundTruthGrid(observedMask));

groundTruthOccupiedCoverage = ...
    100*observedGroundTruthOccupied/totalGroundTruthOccupied;

%% 9. Figure 1: Ground-truth map and robot poses

figure;

show(trueMap);
hold on;

plot(robotPoses(:,1),robotPoses(:,2), ...
    'bo','MarkerFaceColor','b','MarkerSize',6);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Ground-Truth Map and LiDAR Poses');

hold off;

%% 10. Figure 2: Simulated LiDAR returns

figure;

show(trueMap);
hold on;

if ~isempty(allScanEndpoints)
    plot(allScanEndpoints(:,1),allScanEndpoints(:,2), ...
        'r.','MarkerSize',4);
end

plot(robotPoses(:,1),robotPoses(:,2), ...
    'bo','MarkerFaceColor','b','MarkerSize',6);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Simulated LiDAR Returns');

hold off;

%% 11. Figure 3: Probabilistic occupancy grid

figure;

imagesc([cellSize/2,mapWidth-cellSize/2], ...
        [cellSize/2,mapHeight-cellSize/2], ...
        occupancyProbability);

axis xy;
axis equal tight;
clim([0,1]);
colorbar;

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Probabilistic Occupancy Grid');

%% 12. Figure 4: Classified occupancy map

% 0 = unknown, 1 = free, 2 = occupied, 3 = uncertain
displayGrid = zeros(numRows,numCols);

displayGrid(freeMask) = 1;
displayGrid(occupiedMask) = 2;
displayGrid(uncertainMask) = 3;

mapColors = [
    0.55,0.55,0.55;  % Unknown: gray
    0.95,0.95,0.95;  % Free: near-white
    0.10,0.10,0.10;  % Occupied: dark
    0.30,0.60,0.90   % Uncertain: blue
];

figure;

imagesc([cellSize/2,mapWidth-cellSize/2], ...
        [cellSize/2,mapHeight-cellSize/2], ...
        displayGrid);

axis xy;
axis equal tight;

colormap(mapColors);
clim([-0.5,3.5]);

cb = colorbar;
cb.Ticks = [0,1,2,3];
cb.TickLabels = {'Unknown','Free','Occupied','Uncertain'};

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Classified Probabilistic Occupancy Map');

%% 13. Figure 5: Ground-truth comparison

figure;

subplot(1,2,1);

show(trueMap);
title('Ground-Truth Map');

subplot(1,2,2);

imagesc([cellSize/2,mapWidth-cellSize/2], ...
        [cellSize/2,mapHeight-cellSize/2], ...
        displayGrid);

axis xy;
axis equal tight;

colormap(gca,mapColors);
clim([-0.5,3.5]);

cb = colorbar;
cb.Ticks = [0,1,2,3];
cb.TickLabels = {'Unknown','Free','Occupied','Uncertain'};

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Probabilistic Reconstruction');

%% 14. Display results

fprintf('\nExperiment 13 Results\n');

fprintf('Map dimensions: %.1f m x %.1f m\n', ...
    mapWidth,mapHeight);

fprintf('Grid resolution: %d cells/m\n',resolution);
fprintf('LiDAR beams per pose: %d\n',numberOfBeams);
fprintf('Robot poses used: %d\n',size(robotPoses,1));
fprintf('Total LiDAR returns: %d\n',size(allScanEndpoints,1));

fprintf('\nGrid Statistics\n');

fprintf('Total grid cells: %d\n',totalCells);
fprintf('Observed cells: %d\n',observedCells);
fprintf('Free cells: %d\n',freeCells);
fprintf('Occupied cells: %d\n',occupiedCells);
fprintf('Uncertain cells: %d\n',uncertainCells);
fprintf('Unknown cells: %d\n',unknownCells);
fprintf('Observed coverage: %.2f%%\n',coveragePercent);

fprintf('\nMapping Metrics\n');

fprintf('Agreement on definitely classified cells: %.2f%%\n', ...
    observedAgreement);

fprintf('Occupied-cell precision: %.2f%%\n', ...
    occupiedPrecision*100);

fprintf('Occupied-cell recall: %.2f%%\n', ...
    occupiedRecall*100);

fprintf('Ground-truth occupied cells: %d\n', ...
    totalGroundTruthOccupied);

fprintf('Observed ground-truth occupied cells: %d\n', ...
    observedGroundTruthOccupied);

fprintf('Ground-truth occupied-cell coverage: %.2f%%\n', ...
    groundTruthOccupiedCoverage);

fprintf('\nExperiment 13 simulation completed.\n');
