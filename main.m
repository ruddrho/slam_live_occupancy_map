%% main.m
% HYBRID A* + CONTINUOUS LOOK-AHEAD + LIDAR/DWA NAVIGATION
%
% A* computes a collision-free global route after the goal click.
% A monotonic look-ahead follower continuously advances along that route.
% DWA remains the real-time local motion planner and uses the current
% LiDAR scan to generate smooth, dynamically feasible obstacle avoidance.

clear functions;
clear;
clc;
close all;

projectFolder = fileparts(mfilename('fullpath'));

if isempty(projectFolder)
    projectFolder = pwd;
end

addpath(projectFolder);
cd(projectFolder);

cfg = utilities('defaultConfig');

if ~exist(cfg.output.resultsFolder,'dir')
    mkdir(cfg.output.resultsFolder);
end

rng(cfg.environment.randomSeed);

%% Environment and clicked destination
environment = createEnvironment(cfg);

figureHandle = figure( ...
    'Name','Hybrid A* + LIDAR + DWA Robot Navigation', ...
    'Color','w', ...
    'Position',[40 45 1450 850], ...
    'NumberTitle','off');

axesHandle = axes( ...
    'Parent',figureHandle, ...
    'Position',[0.040 0.080 0.660 0.860]);

plotEnvironment(axesHandle,environment,cfg);

goal = goalSelection( ...
    figureHandle,axesHandle,environment,cfg);

%% Global path planning
occupancyGrid = buildOccupancyGrid(environment,cfg);

[rawPath,planningInfo,astarSearchHistory] = ...
    planPathAStar( ...
    occupancyGrid,cfg.robot.startPose(1:2),goal);

if ~planningInfo.success || isempty(rawPath)
    error(['A collision-free path could not be found. ' ...
        'Select another destination or reduce obstacle inflation.']);
end

globalPath = postProcessPath( ...
    rawPath,occupancyGrid,cfg);

% Added visualization only: animate the A* exploration tree and reveal
% the final path before normal robot navigation begins.
animateAStarRoute( ...
    figureHandle,axesHandle,astarSearchHistory, ...
    globalPath,cfg);

pathProgress = 0;
replanCount = 0;
lastReplanTime = -inf;
lastProgressIndex = 0;
lastMeaningfulProgressTime = 0;

%% Initial robot state and display
pose = cfg.robot.startPose(:);
currentCommand = [0;0];

lidarAngles = linspace( ...
    -pi,pi,cfg.lidar.numberOfRays);

initialScan = simulateLidar( ...
    pose,environment,lidarAngles,cfg);

slamState = initializeSlamMap(cfg);
slamState.predictedPose = pose;
slamState = updateSlamMap(slamState,initialScan,cfg,pose);

[localTarget,pathProgress,pathInfo] = ...
    pathFollower(pose,globalPath,pathProgress,cfg,currentCommand(1));

initialDebug.bestScore = 0;
initialDebug.validCandidateCount = 0;
initialDebug.minimumClearance = min(initialScan.ranges);
initialDebug.progressScore = 0;
initialDebug.openingLeft = 0;
initialDebug.openingRight = 0;
initialDebug.recoveryActive = false;
initialDebug.terminalMode = false;
initialDebug.pathProgress = pathInfo.progressFraction;
initialDebug.crossTrackError = pathInfo.crossTrackError;
initialDebug.replanCount = replanCount;
initialDebug.slamExplored = slamState.exploredFraction;

visualHandles = visualization( ...
    'initialize',figureHandle,axesHandle,environment, ...
    pose,goal,initialScan,[],currentCommand,0, ...
    initialDebug,cfg,globalPath,localTarget,pathInfo,slamState);

figureHandle = visualHandles.figure;
axesHandle = visualHandles.axes;

%% Video
videoObject = [];
temporaryImage = '';
targetVideoSize = [];

if cfg.output.recordVideo
    videoObject = VideoWriter( ...
        fullfile(cfg.output.resultsFolder, ...
        'hybrid_astar_dwa_navigation.avi'), ...
        'Motion JPEG AVI');

    videoObject.FrameRate = cfg.output.videoFrameRate;
    open(videoObject);
    temporaryImage = [tempname '.png'];
end

%% Logs
maximumSteps = ceil( ...
    cfg.simulation.maximumTime/ ...
    cfg.simulation.timeStep);

timeLog = zeros(maximumSteps,1);
poseLog = zeros(maximumSteps,3);
linearVelocityLog = zeros(maximumSteps,1);
angularVelocityLog = zeros(maximumSteps,1);
leftWheelVelocityLog = zeros(maximumSteps,1);
rightWheelVelocityLog = zeros(maximumSteps,1);
goalDistanceLog = zeros(maximumSteps,1);
minimumClearanceLog = zeros(maximumSteps,1);
candidateCountLog = zeros(maximumSteps,1);
bestScoreLog = zeros(maximumSteps,1);
crossTrackErrorLog = zeros(maximumSteps,1);
pathProgressLog = zeros(maximumSteps,1);
replanCountLog = zeros(maximumSteps,1);

recoveryState.active = false;
recoveryState.preferredTurnDirection = 1;
recoveryState.activationTime = -inf;

goalReached = false;
collisionDetected = false;
sampleCount = 0;
lowMotionCounter = 0;

try
    for step = 1:maximumSteps
        simulationTime = ...
            (step-1)*cfg.simulation.timeStep;

        distanceToGoal = norm(pose(1:2)'-goal);

        if distanceToGoal <= cfg.goal.tolerance
            goalReached = true;
            currentCommand = [0;0];
            break;
        end

        scan = simulateLidar( ...
            pose,environment,lidarAngles,cfg);

        % Correct the noisy odometry estimate and update the live map.
        slamState = updateSlamMap(slamState,scan,cfg,pose);

        %% Continuous path following
        [localTarget,pathProgress,pathInfo] = ...
            pathFollower( ...
            pose,globalPath,pathProgress,cfg,currentCommand(1));

        if pathProgress >= ...
                lastProgressIndex+ ...
                cfg.path.progressIndexThreshold
            lastProgressIndex = pathProgress;
            lastMeaningfulProgressTime = simulationTime;
        end

        %% Replan if the robot leaves the route or stops progressing
        replanRequired = ...
            pathInfo.crossTrackError > ...
            cfg.path.maximumCrossTrackError || ...
            simulationTime-lastMeaningfulProgressTime > ...
            cfg.path.replanTimeout;

        if replanRequired && ...
                simulationTime-lastReplanTime > ...
                cfg.path.minimumReplanInterval

            [newRawPath,newPlanningInfo] = ...
                planPathAStar( ...
                occupancyGrid,pose(1:2)',goal);

            if newPlanningInfo.success && ...
                    ~isempty(newRawPath)
                globalPath = postProcessPath( ...
                    newRawPath,occupancyGrid,cfg);

                pathProgress = 0;
                lastProgressIndex = 0;
                lastMeaningfulProgressTime = simulationTime;
                lastReplanTime = simulationTime;
                replanCount = replanCount+1;

                [localTarget,pathProgress,pathInfo] = ...
                    pathFollower( ...
                    pose,globalPath,pathProgress,cfg,currentCommand(1));
            else
                % Continue local recovery instead of stopping.
                recoveryState.active = true;
                recoveryState.activationTime = simulationTime;
            end
        end

        %% Terminal docking only when the final corridor is clear
        [terminalCommand,terminalTrajectory, ...
            terminalDebug,terminalModeActive] = ...
            terminalGoalController( ...
            pose,currentCommand,goal,scan,cfg);

        if terminalModeActive && ...
                pathInfo.remainingDistance < ...
                1.5*cfg.goal.terminalModeRadius

            selectedCommand = terminalCommand;
            predictedTrajectory = terminalTrajectory;
            dwaDebug = terminalDebug;
        else
            % Apply a curvature-dependent path speed limit.
            cycleConfig = cfg;
            cycleConfig.robot.maximumLinearVelocity = min( ...
                cfg.robot.maximumLinearVelocity, ...
                pathInfo.speedLimit);

            % Supply the exact projected path centreline to DWA so its
            % predicted trajectories are rewarded for remaining close to
            % the planned route instead of cutting corners.
            cycleConfig.path.guidanceActive = true;
            cycleConfig.path.referencePoint = ...
                pathInfo.projectedPoint;
            cycleConfig.path.referenceTangent = ...
                pathInfo.pathTangent;
            cycleConfig.path.referencePath = ...
                pathInfo.referencePath;
            cycleConfig.path.referenceProgressPoint = ...
                localTarget;
            cycleConfig.path.currentCrossTrackError = ...
                pathInfo.crossTrackError;

            [selectedCommand,predictedTrajectory,dwaDebug] = ...
                dynamicWindowApproach( ...
                pose,currentCommand,localTarget,scan, ...
                recoveryState,cycleConfig);
        end

        dwaDebug.pathProgress = pathInfo.progressFraction;
        dwaDebug.crossTrackError = pathInfo.crossTrackError;
        dwaDebug.replanCount = replanCount;
        dwaDebug.localTarget = localTarget;
        dwaDebug.slamExplored = slamState.exploredFraction;

        %% Prevent an unexpected stationary command
        if selectedCommand(1) < 0.04 && ...
                abs(selectedCommand(2)) < 0.10 && ...
                distanceToGoal > cfg.goal.slowingRadius

            targetHeading = atan2( ...
                localTarget(2)-pose(2), ...
                localTarget(1)-pose(1));

            headingError = utilities( ...
                'wrapAngle',targetHeading-pose(3));

            turnDirection = sign(headingError);

            if turnDirection == 0
                if dwaDebug.openingLeft >= ...
                        dwaDebug.openingRight
                    turnDirection = 1;
                else
                    turnDirection = -1;
                end
            end

            selectedCommand = [ ...
                0;
                turnDirection*min( ...
                0.85,cfg.robot.maximumAngularVelocity)];
        end

        %% Wheel speeds and kinematics
        leftWheelVelocity = ...
            (selectedCommand(1)- ...
            selectedCommand(2)* ...
            cfg.robot.wheelBase/2) / ...
            cfg.robot.wheelRadius;

        rightWheelVelocity = ...
            (selectedCommand(1)+ ...
            selectedCommand(2)* ...
            cfg.robot.wheelBase/2) / ...
            cfg.robot.wheelRadius;

        proposedPose = robotKinematics( ...
            pose,selectedCommand, ...
            cfg.simulation.timeStep);

        %% Physical collision guard with non-stopping recovery
        if collisionCheck(proposedPose,environment,cfg)
            selectedCommand(1) = 0;

            if dwaDebug.openingLeft >= ...
                    dwaDebug.openingRight
                turnDirection = 1;
            else
                turnDirection = -1;
            end

            selectedCommand(2) = ...
                turnDirection* ...
                cfg.safety.emergencyTurnRate;

            proposedPose = robotKinematics( ...
                pose,selectedCommand, ...
                cfg.simulation.timeStep);

            % A circular robot can rotate at the current free center.
            % If numerical contact remains, hold position for one cycle
            % instead of terminating the mission.
            if collisionCheck(proposedPose,environment,cfg)
                proposedPose = pose;
                selectedCommand = [0;0];
                lowMotionCounter = lowMotionCounter+1;
            end
        end

        pose = proposedPose;
        currentCommand = selectedCommand;

        % Simulated wheel odometry prediction for the next SLAM cycle.
        odometryCommand = currentCommand;
        odometryCommand(1) = odometryCommand(1)+ ...
            cfg.slam.linearOdometryNoiseStd*randn;
        odometryCommand(2) = odometryCommand(2)+ ...
            cfg.slam.angularOdometryNoiseStd*randn;

        slamState.predictedPose = robotKinematics( ...
            slamState.estimatedPose,odometryCommand, ...
            cfg.simulation.timeStep);

        if norm(pose(1:2)'-goal) <= ...
                cfg.goal.tolerance
            goalReached = true;
            currentCommand = [0;0];
        end

        if norm(currentCommand) < 0.06
            lowMotionCounter = lowMotionCounter+1;
        else
            lowMotionCounter = max(0,lowMotionCounter-2);
        end

        % Persistent low motion forces a fresh A* route.
        if lowMotionCounter > round( ...
                2.5/cfg.simulation.timeStep) && ...
                simulationTime-lastReplanTime > ...
                cfg.path.minimumReplanInterval

            [newRawPath,newPlanningInfo] = ...
                planPathAStar( ...
                occupancyGrid,pose(1:2)',goal);

            if newPlanningInfo.success
                globalPath = postProcessPath( ...
                    newRawPath,occupancyGrid,cfg);

                pathProgress = 0;
                lastProgressIndex = 0;
                lastMeaningfulProgressTime = simulationTime;
                lastReplanTime = simulationTime;
                replanCount = replanCount+1;
                lowMotionCounter = 0;
            end
        end

        sampleCount = sampleCount+1;

        timeLog(sampleCount) = simulationTime;
        poseLog(sampleCount,:) = pose';
        linearVelocityLog(sampleCount) = currentCommand(1);
        angularVelocityLog(sampleCount) = currentCommand(2);
        leftWheelVelocityLog(sampleCount) = leftWheelVelocity;
        rightWheelVelocityLog(sampleCount) = rightWheelVelocity;
        goalDistanceLog(sampleCount) = ...
            norm(pose(1:2)'-goal);
        minimumClearanceLog(sampleCount) = ...
            dwaDebug.minimumClearance;
        candidateCountLog(sampleCount) = ...
            dwaDebug.validCandidateCount;
        bestScoreLog(sampleCount) = dwaDebug.bestScore;
        crossTrackErrorLog(sampleCount) = ...
            pathInfo.crossTrackError;
        pathProgressLog(sampleCount) = ...
            pathInfo.progressFraction;
        replanCountLog(sampleCount) = replanCount;

        if mod(step,cfg.visualization.updateStride) == 0 || ...
                goalReached

            visualHandles = visualization( ...
                'update',figureHandle,axesHandle,environment, ...
                pose,goal,scan,poseLog(1:sampleCount,:), ...
                currentCommand,simulationTime,dwaDebug,cfg, ...
                visualHandles,predictedTrajectory, ...
                globalPath,localTarget,pathInfo,slamState);

            figureHandle = visualHandles.figure;
            axesHandle = visualHandles.axes;
            drawnow;
        end

        if cfg.output.recordVideo && ...
                mod(step,cfg.output.videoCaptureStride) == 0

            rgbFrame = utilities( ...
                'captureFigureRGB', ...
                figureHandle,temporaryImage);

            if isempty(targetVideoSize)
                frameHeight = size(rgbFrame,1)- ...
                    mod(size(rgbFrame,1),2);
                frameWidth = size(rgbFrame,2)- ...
                    mod(size(rgbFrame,2),2);

                targetVideoSize = ...
                    [frameHeight frameWidth];
            end

            rgbFrame = utilities( ...
                'fitFrame', ...
                rgbFrame,targetVideoSize);

            writeVideo(videoObject,rgbFrame);
        end

        if goalReached
            break;
        end
    end

    if cfg.output.recordVideo
        close(videoObject);
    end
catch errorInformation
    if cfg.output.recordVideo
        try
            close(videoObject);
        catch
        end
    end

    if ~isempty(temporaryImage) && ...
            exist(temporaryImage,'file')
        delete(temporaryImage);
    end

    rethrow(errorInformation);
end

if ~isempty(temporaryImage) && ...
        exist(temporaryImage,'file')
    delete(temporaryImage);
end

%% Trim logs
timeLog = timeLog(1:sampleCount);
poseLog = poseLog(1:sampleCount,:);
linearVelocityLog = linearVelocityLog(1:sampleCount);
angularVelocityLog = angularVelocityLog(1:sampleCount);
leftWheelVelocityLog = leftWheelVelocityLog(1:sampleCount);
rightWheelVelocityLog = rightWheelVelocityLog(1:sampleCount);
goalDistanceLog = goalDistanceLog(1:sampleCount);
minimumClearanceLog = minimumClearanceLog(1:sampleCount);
candidateCountLog = candidateCountLog(1:sampleCount);
bestScoreLog = bestScoreLog(1:sampleCount);
crossTrackErrorLog = crossTrackErrorLog(1:sampleCount);
pathProgressLog = pathProgressLog(1:sampleCount);
replanCountLog = replanCountLog(1:sampleCount);

%% Final display
if goalReached
    statusMessage = 'Goal Reached Successfully';
elseif collisionDetected
    statusMessage = 'Navigation stopped by safety guard';
else
    statusMessage = 'Maximum simulation time reached';
end

finalScan = simulateLidar( ...
    pose,environment,lidarAngles,cfg);

slamState = updateSlamMap(slamState,finalScan,cfg,pose);

finalDebug.bestScore = ...
    utilities('safeLast',bestScoreLog,0);
finalDebug.minimumClearance = ...
    utilities('safeLast',minimumClearanceLog, ...
    min(finalScan.ranges));
finalDebug.validCandidateCount = ...
    utilities('safeLast',candidateCountLog,0);
finalDebug.progressScore = 0;
finalDebug.openingLeft = 0;
finalDebug.openingRight = 0;
finalDebug.recoveryActive = false;
finalDebug.terminalMode = goalReached;
finalDebug.pathProgress = ...
    utilities('safeLast',pathProgressLog,0);
finalDebug.crossTrackError = ...
    utilities('safeLast',crossTrackErrorLog,0);
finalDebug.replanCount = replanCount;
finalDebug.slamExplored = slamState.exploredFraction;

if isempty(poseLog)
    finalHistory = pose';
else
    finalHistory = poseLog;
end

[localTarget,~,pathInfo] = ...
    pathFollower( ...
    pose,globalPath,pathProgress,cfg,0);

visualHandles = visualization( ...
    'finalize',figureHandle,axesHandle,environment, ...
    pose,goal,finalScan,finalHistory,[0;0], ...
    utilities('safeLast',timeLog,0), ...
    finalDebug,cfg,visualHandles,[], ...
    globalPath,localTarget,pathInfo,slamState,statusMessage);

figureHandle = visualHandles.figure;
drawnow;

%% Metrics
if sampleCount > 1
    travelledDistance = sum(sqrt(sum( ...
        diff(poseLog(:,1:2),1,1).^2,2)));
else
    travelledDistance = 0;
end

metrics.GoalReached = goalReached;
metrics.CollisionDetected = collisionDetected;
metrics.MissionTime_s = ...
    utilities('safeLast',timeLog,0);
metrics.TravelledDistance_m = travelledDistance;
metrics.PlannedPathLength_m = planningInfo.pathLength;
metrics.FinalGoalDistance_m = ...
    norm(pose(1:2)'-goal);
metrics.PathErrorRMSE_m = ...
    utilities('safeRMS',crossTrackErrorLog,0);
metrics.MaximumPathError_m = ...
    utilities('safeMax',crossTrackErrorLog,0);
metrics.ReplanCount = replanCount;
metrics.AverageSpeed_mps = ...
    utilities('safeMean',linearVelocityLog,0);
metrics.MinimumClearance_m = ...
    utilities('safeMin',minimumClearanceLog,NaN);

metricsTable = struct2table(metrics);
disp(' ');
disp('===== HYBRID A* + DWA NAVIGATION METRICS =====');
disp(metricsTable);

writetable(metricsTable, ...
    fullfile(cfg.output.resultsFolder, ...
    'hybrid_navigation_metrics.csv'));

%% Performance plots
performanceFigure = figure( ...
    'Color','w', ...
    'Position',[120 80 1150 920], ...
    'Name','Hybrid Navigation Performance');

subplot(4,1,1);
plot(timeLog,linearVelocityLog,'LineWidth',1.8);
hold on;
plot(timeLog,angularVelocityLog,'LineWidth',1.5);
grid on;
ylabel('Command');
legend('v (m/s)','\omega (rad/s)', ...
    'Location','best');
title('Robot Motion Commands');

subplot(4,1,2);
plot(timeLog,goalDistanceLog,'LineWidth',1.8);
grid on;
ylabel('Distance (m)');
title('Distance to Goal');

subplot(4,1,3);
plot(timeLog,crossTrackErrorLog,'LineWidth',1.8);
grid on;
ylabel('Error (m)');
title('Continuous Path-Following Error');

subplot(4,1,4);
plot(timeLog,100*pathProgressLog,'LineWidth',1.8);
grid on;
xlabel('Time (s)');
ylabel('Progress (%)');
title('Global Path Progress');

print(performanceFigure, ...
    fullfile(cfg.output.resultsFolder, ...
    'hybrid_navigation_performance.png'), ...
    '-dpng','-r200');


% Show the performance window immediately before running slower export
% operations such as high-resolution map rendering and video composition.
drawnow;

try
    set(visualHandles.headerText, ...
        'String', ...
        [statusMessage ' — performance ready; generating output files'], ...
        'Color',[0.10 0.25 0.70]);
catch
end
drawnow;

%% Save remaining project outputs after the performance figure is visible
save(fullfile(cfg.output.resultsFolder, ...
    'hybrid_navigation_results.mat'), ...
    'cfg','environment','occupancyGrid', ...
    'goal','rawPath','globalPath', ...
    'planningInfo','metrics', ...
    'timeLog','poseLog','linearVelocityLog', ...
    'angularVelocityLog','leftWheelVelocityLog', ...
    'rightWheelVelocityLog','goalDistanceLog', ...
    'minimumClearanceLog','candidateCountLog', ...
    'bestScoreLog','crossTrackErrorLog', ...
    'pathProgressLog','replanCountLog','slamState');

print(figureHandle, ...
    fullfile(cfg.output.resultsFolder, ...
    'final_hybrid_navigation.png'), ...
    '-dpng','-r180');

exportSlamMap( ...
    slamState,globalPath,goal, ...
    poseLog,goalReached,cfg);

% Combined video is intentionally generated last. The optimized composer
% reads both videos sequentially and renders its title strip only once.
if cfg.output.combinedVideo.enabled
    combineProjectVideos(cfg);
end

try
    set(visualHandles.headerText, ...
        'String',statusMessage, ...
        'Color',[0.85 0.05 0.05]);
catch
end
drawnow;

fprintf('\n===== FINAL STATUS =====\n');
fprintf('%s\n',statusMessage);
fprintf('Replans: %d\n',replanCount);
fprintf('Final goal distance: %.3f m\n', ...
    norm(pose(1:2)'-goal));
fprintf('Results folder: %s\n', ...
    cfg.output.resultsFolder);
