clc;
clear;
close all;

% Experiment 04: Occupancy grid and obstacle representation

% Map parameters
mapWidth = 10;       % Width of room (m)
mapHeight = 8;       % Height of room (m)
resolution = 10;     % Grid cells per metre

% Create occupancy map
map = occupancyMap(mapWidth, mapHeight, resolution);

% Generate cell-centre coordinates
xCells = (0.5/resolution):(1/resolution): ...
    (mapWidth - 0.5/resolution);

yCells = (0.5/resolution):(1/resolution): ...
    (mapHeight - 0.5/resolution);

[X, Y] = meshgrid(xCells, yCells);

% Initially mark the room as free
freePoints = [X(:), Y(:)];
setOccupancy(map, freePoints, 0);

% Add the four boundary walls
wallX = 0.05:(1/resolution):mapWidth-0.05;
wallY = 0.05:(1/resolution):mapHeight-0.05;

% Bottom and top walls
setOccupancy(map, [wallX(:), ...
    0.05*ones(numel(wallX),1)], 1);

setOccupancy(map, [wallX(:), ...
    (mapHeight-0.05)*ones(numel(wallX),1)], 1);

% Left and right walls
setOccupancy(map, [0.05*ones(numel(wallY),1), ...
    wallY(:)], 1);

setOccupancy(map, [(mapWidth-0.05)*ones(numel(wallY),1), ...
    wallY(:)], 1);

% Add a rectangular obstacle
[obsX, obsY] = meshgrid(3.05:0.1:4.95, ...
    2.05:0.1:3.95);

setOccupancy(map, [obsX(:), obsY(:)], 1);

% Add a second rectangular obstacle
[obsX2, obsY2] = meshgrid(6.05:0.1:7.05, ...
    4.05:0.1:6.05);

setOccupancy(map, [obsX2(:), obsY2(:)], 1);

% Display occupancy grid
figure;
show(map);
hold on;

% Mark start and goal positions
startPosition = [1, 1];
goalPosition = [8.5, 6.5];

plot(startPosition(1), startPosition(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

plot(goalPosition(1), goalPosition(2), ...
    'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Occupancy Grid Map with Obstacles');

legend('Start Position', 'Goal Position', ...
    'Location', 'bestoutside');

hold off;

% Display map information
fprintf('Occupancy Grid Information\n');
fprintf('Map width: %.1f m\n', mapWidth);
fprintf('Map height: %.1f m\n', mapHeight);
fprintf('Resolution: %d cells per metre\n', resolution);
fprintf('Approximate number of cells: %d\n', ...
    mapWidth * mapHeight * resolution^2);