
clc;
clear;
close all;

%% Experiment 17: Robust ICP with Outlier Rejection

rng(17);  % Reproducible random noise

%% 1. Generate a reference scan

% Create an asymmetric 2D environment using line segments.
% Asymmetry helps ICP distinguish orientation and position.

wall1 = [linspace(-1, 4, 120)', ...
         -0.9 * ones(120,1)];

wall2 = [-1 * ones(100,1), ...
         linspace(-0.9, 6.8, 100)'];

wall3 = [linspace(-1, 3.2, 100)', ...
         6.8 * ones(100,1)];

wall4 = [3.2 * ones(80,1), ...
         linspace(6.8, 1.0, 80)'];

wall5 = [linspace(3.2, 2.0, 50)', ...
         1.0 * ones(50,1)];

referenceScan = [wall1; wall2; wall3; wall4; wall5];

%% 2. Create a transformed source scan

trueTranslation = [0.20; -0.12];
trueRotationDeg = 4.0;
trueRotation = deg2rad(trueRotationDeg);

Rtrue = [cos(trueRotation), -sin(trueRotation); ...
         sin(trueRotation),  cos(trueRotation)];

% Source points are transformed relative to the reference scan.
sourceClean = (Rtrue' * ...
    (referenceScan' - trueTranslation))';

%% 3. Add measurement noise and outliers

noiseStd = 0.025;

sourceNoisy = sourceClean + ...
    noiseStd * randn(size(sourceClean));

numberOfOutliers = 35;

outlierPoints = [
    -2 + 7*rand(numberOfOutliers,1), ...
    -2 + 10*rand(numberOfOutliers,1)
];

sourceContaminated = [sourceNoisy; outlierPoints];

fprintf('Reference scan points: %d\n',size(referenceScan,1));
fprintf('Source scan points before outliers: %d\n', ...
    size(sourceNoisy,1));
fprintf('Added outlier points: %d\n',numberOfOutliers);
fprintf('Total source points: %d\n', ...
    size(sourceContaminated,1));

%% 4. Plot scans before alignment

figure;

plot(referenceScan(:,1),referenceScan(:,2), ...
    'b.','DisplayName','Reference Scan');
hold on;

plot(sourceContaminated(:,1),sourceContaminated(:,2), ...
    'r.','DisplayName','Noisy Source + Outliers');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Experiment 17: Scans Before Alignment');
legend('Location','best');
axis equal;
grid on;
hold off;

%% 5. Configure ICP

maxIterations = 60;
convergenceTolerance = 1e-6;

% Standard ICP permits a relatively generous matching distance.
standardMaxDistance = 0.60;

% Robust ICP rejects distant matches and trims large residuals.
robustMaxDistance = 0.35;
trimFraction = 0.80;

% Initial estimate: zero rotation and translation.
initialRotation = eye(2);
initialTranslation = [0; 0];

%% 6. Run standard ICP

[standardAligned, standardR, standardT, ...
    standardIterations, standardConverged, ...
    standardResidual] = runICP( ...
    referenceScan,sourceContaminated, ...
    initialRotation,initialTranslation, ...
    maxIterations,convergenceTolerance, ...
    standardMaxDistance,1.0);

%% 7. Run robust ICP

[robustAligned, robustR, robustT, ...
    robustIterations, robustConverged, ...
    robustResidual] = runICP( ...
    referenceScan,sourceContaminated, ...
    initialRotation,initialTranslation, ...
    maxIterations,convergenceTolerance, ...
    robustMaxDistance,trimFraction);

%% 8. Calculate transformation errors

standardRotationDeg = atan2( ...
    standardR(2,1),standardR(1,1))*180/pi;

robustRotationDeg = atan2( ...
    robustR(2,1),robustR(1,1))*180/pi;

% The estimated transformation maps source points into reference frame.
standardTranslationError = norm( ...
    standardT-trueTranslation);

robustTranslationError = norm( ...
    robustT-trueTranslation);

standardRotationError = abs( ...
    standardRotationDeg-trueRotationDeg);

robustRotationError = abs( ...
    robustRotationDeg-trueRotationDeg);

%% 9. Plot aligned scans

figure;

plot(referenceScan(:,1),referenceScan(:,2), ...
    'b.','DisplayName','Reference Scan');
hold on;

plot(standardAligned(:,1),standardAligned(:,2), ...
    'r.','DisplayName','Standard ICP');
plot(robustAligned(:,1),robustAligned(:,2), ...
    'g.','DisplayName','Robust ICP');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Standard ICP vs Robust ICP');
legend('Location','best');
axis equal;
grid on;
hold off;

%% 10. Plot metric comparison

figure;

subplot(1,3,1);

bar([standardTranslationError,robustTranslationError]);
set(gca,'XTickLabel',{'Standard ICP','Robust ICP'});
ylabel('Translation Error (m)');
title('Translation Error');
grid on;

subplot(1,3,2);

bar([standardRotationError,robustRotationError]);
set(gca,'XTickLabel',{'Standard ICP','Robust ICP'});
ylabel('Rotation Error (degrees)');
title('Rotation Error');
grid on;

subplot(1,3,3);

bar([standardResidual,robustResidual]);
set(gca,'XTickLabel',{'Standard ICP','Robust ICP'});
ylabel('Mean Residual (m)');
title('Final Mean Residual');
grid on;

%% 11. Display results

fprintf('\nExperiment 17 Results\n');

fprintf('\nStandard ICP\n');
fprintf('Converged: %s\n',mat2str(standardConverged));
fprintf('Iterations: %d\n',standardIterations);
fprintf('Translation error: %.4f m\n',standardTranslationError);
fprintf('Rotation error: %.4f degrees\n',standardRotationError);
fprintf('Final mean residual: %.4f m\n',standardResidual);

fprintf('\nRobust ICP\n');
fprintf('Converged: %s\n',mat2str(robustConverged));
fprintf('Iterations: %d\n',robustIterations);
fprintf('Translation error: %.4f m\n',robustTranslationError);
fprintf('Rotation error: %.4f degrees\n',robustRotationError);
fprintf('Final mean residual: %.4f m\n',robustResidual);

fprintf('\nExperiment 17 simulation completed.\n');

%% Local function: ICP implementation

function [alignedPoints,R,t,iterations,converged,finalResidual] = ...
    runICP(reference,source,R,t,maxIterations,tolerance, ...
           maxCorrespondenceDistance,trimFraction)

    previousError = inf;
    converged = false;
    iterations = 0;
    finalResidual = inf;

    for iteration = 1:maxIterations

        iterations = iteration;

        % Transform source points using the current estimate.
        transformed = (R*source' + t)';
        
        % Find nearest reference point for every transformed source point.
        nearestIndex = knnsearch(reference,transformed);
        matchedReference = reference(nearestIndex,:);

        residuals = sqrt(sum( ...
            (transformed-matchedReference).^2,2));

        % Reject distant correspondences.
        valid = residuals <= maxCorrespondenceDistance;

        sourceValid = transformed(valid,:);
        referenceValid = matchedReference(valid,:);
        validResiduals = residuals(valid);

        if numel(validResiduals) < 10
            warning('Too few valid correspondences; ICP stopped.');
            break;
        end

        % Optional residual trimming for robust ICP.
        if trimFraction < 1
            residualLimit = quantile(validResiduals,trimFraction);
            keep = validResiduals <= residualLimit;

            sourceValid = sourceValid(keep,:);
            referenceValid = referenceValid(keep,:);
            validResiduals = validResiduals(keep);
        end

        if size(sourceValid,1) < 3
            warning('Too few correspondences after trimming; ICP stopped.');
            break;
        end

        % Compute centroids of corresponding points.
        sourceCentroid = mean(sourceValid,1);
        referenceCentroid = mean(referenceValid,1);

        sourceCentered = sourceValid-sourceCentroid;
        referenceCentered = referenceValid-referenceCentroid;

        % Estimate incremental rigid rotation using SVD.
        H = sourceCentered' * referenceCentered;
        [U,~,V] = svd(H);

        Rdelta = V*U';

        % Correct reflection if necessary.
        if det(Rdelta) < 0
            V(:,end) = -V(:,end);
            Rdelta = V*U';
        end

        tdelta = referenceCentroid' - ...
            Rdelta*sourceCentroid';

        % Compose incremental transform with current transform.
        R = Rdelta*R;
        t = Rdelta*t+tdelta;

        % Calculate residual after updating the transform.
        transformedUpdated = (R*source' + t)';

        nearestUpdated = knnsearch(reference,transformedUpdated);
        matchedUpdated = reference(nearestUpdated,:);

        updatedResiduals = sqrt(sum( ...
            (transformedUpdated-matchedUpdated).^2,2));

        validUpdated = updatedResiduals <= maxCorrespondenceDistance;

        if any(validUpdated)
            finalResidual = mean(updatedResiduals(validUpdated));
        else
            finalResidual = inf;
        end

        % Stop when the residual change becomes sufficiently small.
        if abs(previousError-finalResidual) < tolerance
            converged = true;
            break;
        end

        previousError = finalResidual;
    end

    alignedPoints = (R*source' + t)';
end
