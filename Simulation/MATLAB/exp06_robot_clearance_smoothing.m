
clc;
clear;
close all;

%% Experiment 06: Robot Clearance and Path Smoothing

% Map dimensions
mapWidth = 10;
mapHeight = 8;
resolution = 10;

% Create occupancy map
map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Cell-center coordinates
xCells = 0.05:0.1:9.95;
yCells = 0.05:0.1:7.95;

%% Create boundary walls

setOccupancy(map, [xCells(:), ...
    0.05 * ones(numel(xCells), 1)], 1);

setOccupancy(map, [xCells(:), ...
    7.95 * ones(numel(xCells), 1)], 1);

setOccupancy(map, [0.05 * ones(numel(yCells), 1), ...
    yCells(:)], 1);

setOccupancy(map, [9.95 * ones(numel(yCells), 1), ...
    yCells(:)], 1);

%% Create rectangular obstacles

[obsX, obsY] = meshgrid(3.05:0.1:4.95, ...
                         2.05:0.1:3.95);

setOccupancy(map, [obsX(:), obsY(:)], 1);

[obsX2, obsY2] = meshgrid(6.05:0.1:7.05, ...
                           4.05:0.1:6.05);

setOccupancy(map, [obsX2(:), obsY2(:)], 1);

%% Robot dimensions and clearance

robotRadius = 0.20;
safetyMargin = 0.10;

inflationRadius = robotRadius + safetyMargin;

% Copy original map and inflate obstacles
inflatedMap = copy(map);
inflate(inflatedMap, inflationRadius);

%% A* planning on inflated map

startPosition = [1, 1];
goalPosition = [8.5, 6.5];

planner = plannerAStarGrid(inflatedMap);

path = plan(planner, startPosition, ...
    goalPosition, 'world');

if isempty(path)
    error('No path found on the inflated map.');
end

%% Greedy path smoothing

% Keep the first waypoint
smoothedPath = path(1, :);

currentIndex = 1;

while currentIndex < size(path, 1)

    nextIndex = size(path, 1);

    % Find the farthest waypoint reachable
    % by a collision-free straight segment
    while nextIndex > currentIndex + 1

        p1 = path(currentIndex, :);
        p2 = path(nextIndex, :);

        distance = norm(p2 - p1);

        % Sample points along the segment
        sampleCount = max(2, ceil(distance * ...
            resolution * 2) + 1);

        t = linspace(0, 1, sampleCount)';

        segmentPoints = p1 + t .* (p2 - p1);

        occupied = checkOccupancy( ...
            inflatedMap, segmentPoints);

        if all(occupied == 0)
            break;
        end

        nextIndex = nextIndex - 1;
    end

    smoothedPath = [smoothedPath; ...
        path(nextIndex, :)];

    currentIndex = nextIndex;
end

%% Calculate path lengths

originalLength = sum(sqrt(sum(diff(path).^2, 2)));

smoothedLength = sum(sqrt( ...
    sum(diff(smoothedPath).^2, 2)));

%% Plot results

figure;

subplot(1, 2, 1);
show(map);
hold on;
plot(path(:, 1), path(:, 2), ...
    'b-', 'LineWidth', 2);

plot(startPosition(1), startPosition(2), ...
    'go', 'MarkerFaceColor', 'g');

plot(goalPosition(1), goalPosition(2), ...
    'rp', 'MarkerFaceColor', 'r');

title('Original Map - Clearance Planned Path');
xlabel('X (m)');
ylabel('Y (m)');
hold off;

subplot(1, 2, 2);
show(inflatedMap);
hold on;

plot(path(:, 1), path(:, 2), ...
    'b--', 'LineWidth', 1.5);

plot(smoothedPath(:, 1), ...
    smoothedPath(:, 2), ...
    'g-', 'LineWidth', 2.5);

plot(startPosition(1), startPosition(2), ...
    'go', 'MarkerFaceColor', 'g');

plot(goalPosition(1), goalPosition(2), ...
    'rp', 'MarkerFaceColor', 'r');

title('Inflated Map - Smoothed Path');
xlabel('X (m)');
ylabel('Y (m)');
legend('A* Path', 'Smoothed Path', ...
    'Start', 'Goal', 'Location', 'best');

hold off;

%% Display results

fprintf('\nExperiment 06 Results\n');
fprintf('Robot radius: %.2f m\n', robotRadius);
fprintf('Safety margin: %.2f m\n', safetyMargin);
fprintf('Total inflation: %.2f m\n', inflationRadius);

fprintf('A* waypoints: %d\n', size(path, 1));
fprintf('Smoothed waypoints: %d\n', ...
    size(smoothedPath, 1));

fprintf('A* path length: %.3f m\n', originalLength);
fprintf('Smoothed path length: %.3f m\n', ...
    smoothedLength);

fprintf('Experiment 06 completed.\n');
