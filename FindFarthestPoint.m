function [xyFar] = FindFarthestPoint(locationArray, robotPose)
%FINDFARTHESTPOINT Summary of this function goes here
%   Detailed explanation goes here
for i = 1:3
    temp(i) = norm(locationArray(i,1:2) - robotPose(1:2));
end
% Get point in the middle of the array relative to the robot
[m p] = min(temp)
display(locationArray(p,:))
xyFar = locationArray(p,:)
end

