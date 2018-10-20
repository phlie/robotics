filePath = fullfile(fileparts(which('OfflineSLAMExample')), 'data', 'offlineSlamData.mat');
load(filePath);

maxLidarRange = 8;  % The max range of the sensors, slightly less than their actual max
mapResolution = 20;
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange)  % start the robotics slam algorithm

slamAlg.LoopClosureThreshold = 210;
slagAlg.LoopClosureSearchRadius = 8;

% The inital 10 scans, used to print a simple map
for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

firstTimeLCDetected = false;

figure;
% Keep looping for all of the scans and add to the graph every accepted
% scan
for i=10:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
%    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph);
        hold off;
        firstTimeLCDetected = true;
        drawnow
%    end
end

% Display the final map
figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

% Convert to an Occupancy Grid
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

% Display the Occupancy Grid
figure;
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off;
title('Occupancy Grid Map Built Using Lidar SLAM');