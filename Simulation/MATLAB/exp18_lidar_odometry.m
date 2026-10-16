
clc;
clear;
close all;

%% Experiment 18: LiDAR Odometry Using Consecutive Scans

rng(18);

%% 1. Create an asymmetric 2D environment

wall1 = [linspace(-1,4,160)', -0.9*ones(160,1)];
wall2 = [-1*ones(120,1), linspace(-0.9,6.8,120)'];
wall3 = [linspace(-1,3.2,130)', 6.8*ones(130,1)];
wall4 = [3.2*ones(100,1), linspace(6.8,1.0,100)'];
wall5 = [linspace(3.2,2.0,60)', 1.0*ones(60,1)];

environment = [wall1; wall2; wall3; wall4; wall5];

%% 2. Define robot poses in the world frame

% Pose = [x, y, heading in radians]
truePoses = [
    0.00, 0.00, 0.0;
    0.15, 0.04, deg2rad(2);
    0.29, 0.12, deg2rad(5);
    0.40, 0.23, deg2rad(8);
    0.48, 0.37, deg2rad(12);
    0.53, 0.52, deg2rad(16)
    ];

numberOfPoses = size(truePoses,1);

%% 3. Generate simulated scans

noiseStd = 0.008;
scans = cell(numberOfPoses,1);

for i = 1:numberOfPoses

    x = truePoses(i,1);
    y = truePoses(i,2);
    theta = truePoses(i,3);

    RworldToRobot = [
        cos(theta), sin(theta);
        -sin(theta), cos(theta)
        ];

    localScan = (RworldToRobot * ...
        (environment' - [x;y]))';

    % Keep points within a simulated LiDAR range
    ranges = sqrt(sum(localScan.^2,2));
    visible = ranges <= 8.0;

    localScan = localScan(visible,:);

    % Add small measurement noise
    localScan = localScan + ...
        noiseStd*randn(size(localScan));

    scans{i} = localScan;
end

%% 4. Plot the first and last scans

figure;

plot(scans{1}(:,1),scans{1}(:,2), ...
    'b.','DisplayName','First Scan');
hold on;

plot(scans{end}(:,1),scans{end}(:,2), ...
    'r.','DisplayName','Last Scan');

xlabel('Local X (m)');
ylabel('Local Y (m)');
title('Experiment 18: Consecutive LiDAR Scans');
legend('Location','best');
axis equal;
grid on;
hold off;

%% 5. Estimate relative motion between consecutive scans

estimatedRelative = zeros(numberOfPoses-1,3);
trueRelative = zeros(numberOfPoses-1,3);

maxIterations = 50;
maxCorrespondenceDistance = 0.35;
trimFraction = 0.85;
tolerance = 1e-6;

for i = 1:numberOfPoses-1

    reference = scans{i};
    source = scans{i+1};

    % Ground-truth relative transform: scan i+1 into scan i.
    theta1 = truePoses(i,3);
    theta2 = truePoses(i+1,3);

    R1 = [cos(theta1), -sin(theta1); ...
        sin(theta1),  cos(theta1)];

    R2 = [cos(theta2), -sin(theta2); ...
        sin(theta2),  cos(theta2)];

    p1 = truePoses(i,1:2)';
    p2 = truePoses(i+1,1:2)';

    RtrueRelative = R1' * R2;
    ttrueRelative = R1' * (p2-p1);

    trueRelative(i,:) = [
        ttrueRelative(1), ...
        ttrueRelative(2), ...
        atan2(RtrueRelative(2,1),RtrueRelative(1,1))
        ];

    % Estimate the transformation mapping source scan into reference scan.
    R = eye(2);
    t = [0;0];
    previousResidual = inf;
    converged = false;

    for iteration = 1:maxIterations

        transformed = (R*source' + t)';

        nearestIndex = knnsearch(reference,transformed);
        matchedReference = reference(nearestIndex,:);

        residuals = sqrt(sum( ...
            (transformed-matchedReference).^2,2));

        valid = residuals <= maxCorrespondenceDistance;

        sourceValid = transformed(valid,:);
        referenceValid = matchedReference(valid,:);
        validResiduals = residuals(valid);

        if numel(validResiduals) < 10
            warning('Insufficient correspondences between scans %d and %d.', ...
                i,i+1);
            break;
        end

        residualLimit = quantile(validResiduals,trimFraction);
        keep = validResiduals <= residualLimit;

        sourceValid = sourceValid(keep,:);
        referenceValid = referenceValid(keep,:);

        if size(sourceValid,1) < 3
            warning('Insufficient trimmed correspondences.');
            break;
        end

        sourceCentroid = mean(sourceValid,1);
        referenceCentroid = mean(referenceValid,1);

        H = (sourceValid-sourceCentroid)' * ...
            (referenceValid-referenceCentroid);

        [U,~,V] = svd(H);

        Rdelta = V*U';

        if det(Rdelta) < 0
            V(:,end) = -V(:,end);
            Rdelta = V*U';
        end

        tdelta = referenceCentroid' - ...
            Rdelta*sourceCentroid';

        R = Rdelta*R;
        t = Rdelta*t+tdelta;

        transformedUpdated = (R*source' + t)';

        nearestUpdated = knnsearch(reference,transformedUpdated);
        matchedUpdated = reference(nearestUpdated,:);

        updatedResiduals = sqrt(sum( ...
            (transformedUpdated-matchedUpdated).^2,2));

        validUpdated = updatedResiduals <= maxCorrespondenceDistance;

        if any(validUpdated)
            currentResidual = mean(updatedResiduals(validUpdated));
        else
            currentResidual = inf;
        end

        if abs(previousResidual-currentResidual) < tolerance
            converged = true;
            break;
        end

        previousResidual = currentResidual;
    end

    % Convert the source-to-reference transform into robot motion.
    % ICP maps scan i+1 coordinates into scan i coordinates.
    estimatedRotation = atan2(R(2,1),R(1,1));

    % Translation and rotation are expressed in the reference robot frame.
    estimatedRelative(i,:) = [
        t(1), ...
        t(2), ...
        estimatedRotation
        ];

    fprintf('Scan pair %d -> %d: %d ICP iterations, converged: %s\n', ...
        i,i+1,iteration,mat2str(converged));
end

%% 6. Calculate relative motion errors

translationErrors = sqrt(sum( ...
    (estimatedRelative(:,1:2)-trueRelative(:,1:2)).^2,2));

rotationDifference = estimatedRelative(:,3)-trueRelative(:,3);

% Wrap angle errors to [-pi, pi].
rotationDifference = atan2( ...
    sin(rotationDifference),cos(rotationDifference));

rotationErrorsDeg = abs(rad2deg(rotationDifference));

%% 7. Accumulate relative motion estimates

estimatedPoses = zeros(numberOfPoses,3);
estimatedPoses(1,:) = truePoses(1,:);

for i = 1:numberOfPoses-1

    theta = estimatedPoses(i,3);

    localTranslation = estimatedRelative(i,1:2)';

    worldRotation = [cos(theta), -sin(theta); ...
        sin(theta),  cos(theta)];

    worldTranslation = worldRotation*localTranslation;

    estimatedPoses(i+1,1:2) = ...
        estimatedPoses(i,1:2)+worldTranslation';

    estimatedPoses(i+1,3) = ...
        estimatedPoses(i,3)+estimatedRelative(i,3);

    estimatedPoses(i+1,3) = atan2( ...
        sin(estimatedPoses(i+1,3)), ...
        cos(estimatedPoses(i+1,3)));
end

%% 8. Calculate accumulated pose errors

positionErrors = sqrt(sum( ...
    (estimatedPoses(:,1:2)-truePoses(:,1:2)).^2,2));

headingDifference = estimatedPoses(:,3)-truePoses(:,3);

headingDifference = atan2( ...
    sin(headingDifference),cos(headingDifference));

headingErrorsDeg = abs(rad2deg(headingDifference));

%% 9. Plot true and estimated trajectories

figure;

plot(truePoses(:,1),truePoses(:,2), ...
    'bo-','LineWidth',1.5,'DisplayName','Ground Truth');
hold on;

plot(estimatedPoses(:,1),estimatedPoses(:,2), ...
    'r.--','LineWidth',1.5,'DisplayName','Estimated Odometry');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('LiDAR Odometry: Ground Truth vs Estimate');
legend('Location','best');
axis equal;
grid on;
hold off;

%% 10. Plot relative motion errors

figure;

subplot(1,2,1);

bar(translationErrors);
xlabel('Consecutive Scan Pair');
ylabel('Translation Error (m)');
title('Relative Translation Error');
grid on;

subplot(1,2,2);

bar(rotationErrorsDeg);
xlabel('Consecutive Scan Pair');
ylabel('Rotation Error (degrees)');
title('Relative Rotation Error');
grid on;

%% 11. Display results

fprintf('\nExperiment 18 Results\n');

fprintf('Scan pairs evaluated: %d\n',numberOfPoses-1);
fprintf('Mean relative translation error: %.4f m\n', ...
    mean(translationErrors));
fprintf('Maximum relative translation error: %.4f m\n', ...
    max(translationErrors));
fprintf('Mean relative rotation error: %.4f degrees\n', ...
    mean(rotationErrorsDeg));
fprintf('Maximum relative rotation error: %.4f degrees\n', ...
    max(rotationErrorsDeg));
fprintf('Final accumulated position error: %.4f m\n', ...
    positionErrors(end));
fprintf('Final accumulated heading error: %.4f degrees\n', ...
    headingErrorsDeg(end));

fprintf('\nPair-wise relative motion comparison\n');
disp(table((1:numberOfPoses-1)', ...
    trueRelative(:,1),trueRelative(:,2),rad2deg(trueRelative(:,3)), ...
    estimatedRelative(:,1),estimatedRelative(:,2), ...
    rad2deg(estimatedRelative(:,3)), ...
    translationErrors,rotationErrorsDeg, ...
    'VariableNames',{'Pair','TrueDx_m','TrueDy_m','TrueDtheta_deg', ...
    'EstimatedDx_m','EstimatedDy_m','EstimatedDtheta_deg', ...
    'TranslationError_m','RotationError_deg'}));

fprintf('\nExperiment 18 simulation completed.\n');
