robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('simpleMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 100;
prm.ConnectionDistance = 10;

startLocation = [2.0 1.0];
endLocation = [12.0 10.0];
path = findpath(prm, startLocation, endLocation)

show(prm, 'Map', 'off', 'Roadmap', 'off');
% 
% plot(path(:,1), path(:,2), 'k--d')
% xlim([0 13])
% ylim([0 13])
% 
controller = robotics.PurePursuit
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;

robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];

robot.setRobotPose(robotCurrentPose);
 
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

 controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    % Computes the controller outputs, inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simluate the robot using the controller outputs
    drive(robot, v, omega);
    
    % Extract current logcation informaiton
    robotCurrentPose = robot.getRobotPose;
    
    % Re-compute the distnace to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
end