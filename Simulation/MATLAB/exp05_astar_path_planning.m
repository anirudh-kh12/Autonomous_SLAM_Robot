
clc;
clear;
close all;

% Experiment 05: A* Path Planning

% Map parameters
mapWidth = 10;
mapHeight = 8;
resolution = 10;       % Cells per metre

% Create a binary occupancy map
map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Cell-centre coordinates
xCells = 0.5/resolution : 1/resolution : ...
         mapWidth - 0.5/resolution;

yCells = 0.5/resolution : 1/resolution : ...
         mapHeight - 0.5/resolution;

[X, Y] = meshgrid(xCells, yCells);

% Set the entire room to free
setOccupancy(map, [X(:), Y(:)], 0);

% Boundary walls
wallX = xCells(:);
wallY = yCells(:);

setOccupancy(map, [wallX, ...
    0.05*ones(numel(wallX),1)], 1);

setOccupancy(map, [wallX, ...
    (mapHeight-0.05)*ones(numel(wallX),1)], 1);

setOccupancy(map, [0.05*ones(numel(wallY),1), ...
    wallY], 1);

setOccupancy(map, [(mapWidth-0.05)*ones(numel(wallY),1), ...
    wallY], 1);

% First rectangular obstacle
[obsX, obsY] = meshgrid(3.05:0.1:4.95, ...
                        2.05:0.1:3.95);

setOccupancy(map, [obsX(:), obsY(:)], 1);

% Second rectangular obstacle
[obsX2, obsY2] = meshgrid(6.05:0.1:7.05, ...
                          4.05:0.1:6.05);

setOccupancy(map, [obsX2(:), obsY2(:)], 1);

% Define start and goal in world coordinates
startPosition = [1, 1];
goalPosition = [8.5, 6.5];

% Create A* path planner
planner = plannerAStarGrid(map);

% Plan the path in world coordinates
path = plan(planner, startPosition, ...
            goalPosition, 'world');

% Check that a path was returned
if isempty(path)
    error('No valid path found between start and goal.');
end

% Validate that path points are in free cells
occupancyValues = checkOccupancy(map, path);

if any(occupancyValues > 0)
    error('The planned path contains occupied cells.');
end

% Calculate approximate path length
pathLength = sum(sqrt(sum(diff(path).^2, 2)));

% Display map and planned path
figure;
show(map);
hold on;

plot(path(:,1), path(:,2), 'b-', ...
    'LineWidth', 2.5);

plot(startPosition(1), startPosition(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

plot(goalPosition(1), goalPosition(2), ...
    'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('A* Path Planning on an Occupancy Grid');

legend('Planned Path', 'Start Position', ...
    'Goal Position', 'Location', 'best');

hold off;

% Display results
fprintf('\nA* Path Planning Results\n');
fprintf('Start: (%.2f, %.2f) m\n', ...
    startPosition(1), startPosition(2));

fprintf('Goal: (%.2f, %.2f) m\n', ...
    goalPosition(1), goalPosition(2));

fprintf('Number of path waypoints: %d\n', ...
    size(path, 1));

fprintf('Approximate path length: %.3f m\n', ...
    pathLength);

fprintf('Path occupancy validation: PASSED\n');
