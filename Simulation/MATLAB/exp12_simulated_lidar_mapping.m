
clc;
clear;
close all;

%% Experiment 12: Simulated LiDAR and Sensor-Based Mapping

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

%% 5. Initialize evidence grid

% 0 = unknown
% -1 = free
% +1 = occupied
mappedGrid = zeros(numRows,numCols);

allScanEndpoints = zeros(0,2);

%% 6. Simulate LiDAR and update the grid

for p = 1:size(robotPoses,1)

    pose = robotPoses(p,:);

    scanEndpoints = zeros(0,2);

    for b = 1:numberOfBeams

        rayAngle = pose(3)+angles(b);

        hitDetected = false;

        for distance = rayStep:rayStep:maxRange

            pointX = pose(1)+distance*cos(rayAngle);
            pointY = pose(2)+distance*sin(rayAngle);

            % Stop if the ray leaves the map
            if pointX < 0 || pointX >= mapWidth || ...
                    pointY < 0 || pointY >= mapHeight
                break;
            end

            col = floor(pointX*resolution)+1;
            row = floor(pointY*resolution)+1;

            col = max(1,min(numCols,col));
            row = max(1,min(numRows,row));

            if checkOccupancy(trueMap,[pointX,pointY]) > 0

                % Occupied endpoint takes priority over free evidence
                mappedGrid(row,col) = 1;

                scanEndpoints(end+1,:) = [pointX,pointY]; %#ok<SAGROW>
                allScanEndpoints(end+1,:) = [pointX,pointY]; %#ok<SAGROW>

                hitDetected = true;
                break;
            else

                % Record observed free space unless already occupied
                if mappedGrid(row,col) ~= 1
                    mappedGrid(row,col) = -1;
                end
            end
        end

        % Rays that reach maximum range provide free-space evidence
        % along the traversed part of the ray.
        if ~hitDetected
            % Free cells were updated during ray marching.
        end
    end

    fprintf('Pose %d: %d LiDAR returns.\n', ...
        p,size(scanEndpoints,1));
end

%% 7. Prepare ground-truth cell labels

groundTruthGrid = occupancyMatrix(trueMap) > 0;

observedMask = mappedGrid ~= 0;
mappedOccupied = mappedGrid == 1;


%% 8. Calculate mapping metrics

totalCells = numel(mappedGrid);

% Cells for which LiDAR provided evidence
observedMask = mappedGrid ~= 0;

% Occupancy classifications
mappedOccupied = mappedGrid == 1;
mappedFree = mappedGrid == -1;

% Ground-truth occupancy
groundTruthGrid = occupancyMatrix(trueMap) > 0;

% Coverage statistics
unknownCells = sum(~observedMask(:));
freeCells = sum(mappedFree(:));
occupiedCells = sum(mappedOccupied(:));
observedCells = sum(observedMask(:));

coveragePercent = 100*observedCells/totalCells;

% Compare classifications only on observed cells
observedMapped = mappedOccupied(observedMask);
observedTruth = groundTruthGrid(observedMask);

correctObservedCells = observedMapped == observedTruth;

observedAgreement = 100*mean(correctObservedCells);

% Observed-cell confusion counts
truePositive = sum(observedMapped & observedTruth);
falsePositive = sum(observedMapped & ~observedTruth);
falseNegative = sum(~observedMapped & observedTruth);

% Occupied-cell precision
if truePositive+falsePositive > 0
    occupiedPrecision = ...
        truePositive/(truePositive+falsePositive);
else
    occupiedPrecision = NaN;
end

% Occupied-cell recall among observed ground-truth occupied cells
if truePositive+falseNegative > 0
    occupiedRecall = ...
        truePositive/(truePositive+falseNegative);
else
    occupiedRecall = NaN;
end

% How many ground-truth occupied cells received observations?
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
title('Ground-Truth Map and LiDAR Positions');

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

%% 11. Figure 3: Reconstructed occupancy grid

% Use explicit categorical display values:
% Unknown = 0
% Free = 1
% Occupied = 2
displayGrid = zeros(numRows,numCols);

displayGrid(mappedGrid == -1) = 1;
displayGrid(mappedGrid == 1) = 2;

figure;

imagesc([cellSize/2,mapWidth-cellSize/2], ...
    [cellSize/2,mapHeight-cellSize/2], ...
    displayGrid);

axis xy;
axis equal tight;

% 3 distinct colours: unknown, free, occupied
colormap([0.55,0.55,0.55; ...
    0.95,0.95,0.95; ...
    0.10,0.10,0.10]);

clim([-0.5,2.5]);

cb = colorbar;
cb.Ticks = [0,1,2];
cb.TickLabels = {'Unknown','Free','Occupied'};

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('LiDAR-Reconstructed Occupancy Grid');

%% 12. Figure 4: Ground truth vs reconstructed map

figure;

subplot(1,2,1);

imagesc([cellSize/2,mapWidth-cellSize/2], ...
    [cellSize/2,mapHeight-cellSize/2], ...
    double(groundTruthGrid));

axis xy;
axis equal tight;
colormap(gca,gray(2));
clim([0,1]);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Ground-Truth Occupancy');

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
title('Reconstructed Occupancy');


%% 13. Display results

fprintf('\nExperiment 12 Results\n');

fprintf('Map dimensions: %.1f m x %.1f m\n', ...
    mapWidth,mapHeight);

fprintf('Grid resolution: %d cells/m\n',resolution);
fprintf('LiDAR beams per pose: %d\n',numberOfBeams);
fprintf('Robot poses used: %d\n',size(robotPoses,1));
fprintf('Total LiDAR returns: %d\n',size(allScanEndpoints,1));

fprintf('\nGrid Statistics\n');
fprintf('Total grid cells: %d\n',totalCells);
fprintf('Observed free cells: %d\n',freeCells);
fprintf('Observed occupied cells: %d\n',occupiedCells);
fprintf('Unknown cells: %d\n',unknownCells);
fprintf('Observed cell coverage: %.2f%%\n',coveragePercent);

fprintf('\nMapping Accuracy Metrics\n');
fprintf('Observed-cell agreement: %.2f%%\n',observedAgreement);
fprintf('Occupied-cell precision: %.2f%%\n',occupiedPrecision*100);
fprintf('Occupied-cell recall (observed cells): %.2f%%\n', ...
    occupiedRecall*100);

fprintf('Ground-truth occupied cells: %d\n', ...
    totalGroundTruthOccupied);

fprintf('Observed ground-truth occupied cells: %d\n', ...
    observedGroundTruthOccupied);

fprintf('Ground-truth occupied-cell coverage: %.2f%%\n', ...
    groundTruthOccupiedCoverage);

fprintf('\nExperiment 12 simulation completed.\n');

