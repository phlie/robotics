function [xyFar] = FindFarthestPoint(locationArray, robotPose)

% Gets the furthest point for the robot and its current location 
for i = 1:3
    temp(i) = norm(locationArray(i,1:2) - robotPose(1:2));
end
% Get point in the middle of the array relative to the robot
[m p] = min(temp)
display(locationArray(p,:))
xyFar = locationArray(p,:)
end

