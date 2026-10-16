
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

%% 4. Robot poses

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

%% 5. Initialize log-odds occupancy grid

% Zero log-odds means unknown, with probability 0.5.
logOdds = zeros(numRows,numCols);

% Evidence update strengths
freeUpdate = -0.40;
occupiedUpdate = 0.85;

% Prevent evidence from growing without limit
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

            col = floor(pointX*resolution)+1;
            row = floor(pointY*resolution)+1;

            col = max(1,min(numCols,col));
            row = max(1,min(numRows,row));

            if checkOccupancy(trueMap,[pointX,pointY]) > 0

                % A LiDAR return provides occupied evidence
                logOdds(row,col) = min(maxLogOdds, ...
                    logOdds(row,col)+occupiedUpdate);

                scanEndpoints(end+1,:) = ...
                    [pointX,pointY]; %#ok<SAGROW>

                allScanEndpoints(end+1,:) = ...
                    [pointX,pointY]; %#ok<SAGROW>

                hitDetected = true;
                break;

            else

                % The ray passed through this cell:
                % accumulate free-space evidence
                logOdds(row,col) = max(minLogOdds, ...
                    logOdds(row,col)+freeUpdate);
            end
        end

        if ~hitDetected
            % No hit: traversed cells retain free-space evidence.
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

% Unknown cells have probabilities between the thresholds
unknownMask = ...
    occupancyProbability >= freeThreshold & ...
    occupancyProbability <= occupiedThreshold;

freeMask = occupancyProbability < freeThreshold;
occupiedMask = occupancyProbability > occupiedThreshold;

%% 8. Evaluate mapping against ground truth

groundTruthGrid = occupancyMatrix(trueMap) > 0;

observedMask = ~unknownMask;

totalCells = numel(logOdds);
observedCells = sum(observedMask(:));
unknownCells = sum(unknownMask(:));
freeCells = sum(freeMask(:));
occupiedCells = sum(occupiedMask(:));

coveragePercent = 100*observedCells/totalCells;

% Compare classifications only where the mapper classified cells
predictedOccupied = occupiedMask(observedMask);
truthOccupied = groundTruthGrid(observedMask);

observedAgreement = ...
    100*mean(predictedOccupied == truthOccupied);

truePositive = sum(predictedOccupied & truthOccupied);
falsePositive = sum(predictedOccupied & ~truthOccupied);
falseNegative = sum(~predictedOccupied & truthOccupied);

if truePositive+falsePositive > 0
    occupiedPrecision = ...
        truePositive/(truePositive+falsePositive);
else
    occupiedPrecision = NaN;
end

if truePositive+falseNegative > 0
    occupiedRecall = ...
        truePositive/(truePositive+falseNegative);
else
    occupiedRecall = NaN;
end

%% 9. Figure 1: Ground-truth map

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

plot(allScanEndpoints(:,1),allScanEndpoints(:,2), ...
    'r.','MarkerSize',4);

plot(robotPoses(:,1),robotPoses(:,2), ...
    'bo','MarkerFaceColor','b','MarkerSize',6);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Simulated LiDAR Returns');

hold off;

%% 11. Figure 3: Occupancy probability map

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

% 0 = unknown, 1 = free, 2 = occupied
displayGrid = zeros(numRows,numCols);

displayGrid(freeMask) = 1;
displayGrid(occupiedMask) = 2;

figure;

imagesc([cellSize/2,mapWidth-cellSize/2], ...
        [cellSize/2,mapHeight-cellSize/2], ...
        displayGrid);

axis xy;
axis equal tight;

colormap([0.55,0.55,0.55; ...
          0.95,0.95,0.95; ...
          0.10,0.10,0.10]);

clim([-0.5,2.5]);

cb = colorbar;
cb.Ticks = [0,1,2];
cb.TickLabels = {'Unknown','Free','Occupied'};

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

colormap(gca,[0.55,0.55,0.55; ...
              0.95,0.95,0.95; ...
              0.10,0.10,0.10]);

clim([-0.5,2.5]);

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
fprintf('Unknown cells: %d\n',unknownCells);
fprintf('Observed coverage: %.2f%%\n',coveragePercent);

fprintf('\nMapping Metrics\n');

fprintf('Observed-cell agreement: %.2f%%\n',observedAgreement);
fprintf('Occupied-cell precision: %.2f%%\n',occupiedPrecision*100);
fprintf('Occupied-cell recall: %.2f%%\n',occupiedRecall*100);

fprintf('\nExperiment 13 simulation completed.\n');
