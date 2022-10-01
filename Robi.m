rosshutdown
% Get the environment map
image = imread('environment.png');

% Convert the image to black and white
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Size of the grid with the resolution
xsize = 26.0;
ysize = 21.0;
mapRes = 20.0;

% Initalize the ROS server
rosinit
% Turn the image into a amp
map = robotics.BinaryOccupancyGrid(bwimage);

% For now, use the simpleMap but I can also use, map, the custom image
sim = ExampleHelperRobotSimulator('complexMap');
setRobotPose(sim, [3 1 0]);     % Set the starting location on the map
sim.showTrajectory(true);

% Enables the ROS server for the sim
enableROSInterface(sim, true);

% Subscribe to the bumper sensor and publish to veloctiy
bumperSub = rossubscriber('/mobile_base/sensors/bumper');
scanSub = rossubscriber('scan');
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');

% For later use in exploring the enviroment
tftree = rostf;

% Pause for a second to allow tranformation tree object to finish init
pause(1);

% controller = robotics.PurePursuit('Waypoints', path);
% controller.DesiredLinearVelocity = 10.0;
% Set the control rate at 10HZ
controlRate = robotics.Rate(10);

% Create a map of the unknown regions that is the same size as the sim
mapUnknown = robotics.OccupancyGrid(xsize,ysize,mapRes);

% Setup the plot of the unknown region
figureHandle = figure('Name', 'MapUnknown');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(mapUnknown, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');

% All the initilization of the variables code
robotPose = [0 0 0];
bump = 0;
numBumps = 0;
bumpToggle = false;
startOcc = true;
path = [];

% The main loop that executes all of the robot code, doesn't reach the end

for i = 1:250
    % Keep going for awhile
    startOcc = true;
    while(startOcc == true)
        updateCounter = 1;
        whichWay = -1;      % Start spinning to the left and then it will change
        while(updateCounter < 100 || bumpToggle == true)
            % Get the scan message
            scanMsg = receive(scanSub);
            
            % Get robot pose at time of sensor reading
            pose = getTransform(tftree, 'map', 'robot_base', scanMsg.Header.Stamp, 'Timeout', 2);
            
            % Convert robot pose to 1x3 vector [x y yaw]
            position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
            orientation = quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
            robotPose = [position, orientation(1)];     % Update the robot position and angle
            
            
            % Extract the laser scan
            scan = lidarScan(scanMsg);
            ranges = scan.Ranges; % Scans across the ranges of the lazer
            ranges(isnan(ranges)) = sim.LaserSensor.MaxRange; % If nan replace with its max range
            modScan = lidarScan(ranges, scan.Angles);
            
            % Insert the laser range observation in the map
            insertRay(mapUnknown, robotPose, modScan, sim.LaserSensor.MaxRange);
            
            
            % Get the bumper sensor
            bump = receive(bumperSub);
            % If not touching anything, go in a straight line
            if (bump.Data == false)
                velMsg.Linear.X = 1000.0;
                velMsg.Angular.Z = 0.02;
                % If touching something, spin
            else
                % display(whichWay)
                velMsg.Linear.X = 0.0;
                velMsg.Angular.Z = whichWay * 10.0;
                numBumps = numBumps + 1;
                % velMsg.Angular.Z = 10.0;
            end
            
            % Send the velocity commands to the robot
            send(velPub, velMsg);
            
            % Randomly change direction after the updateCounter reaches 50
            if ~mod(updateCounter, 50)
                if (randi(2) == 1)
                    whichWay = -1;
                else
                    whichWay = 1;
                end
            end
            
            % Used to see if the robot has recently touched a wall
            if ~mod(updateCounter, 10)
                if (numBumps > 0)
                    bumpToggle = true;
                else
                    bumpToggle = false;
                end
                numBumps = 0;
            end
            
            
            
            % Increment the counter
            updateCounter = updateCounter+1;
            % Update the unknown map every 20 updates
            if ~mod(updateCounter,10)
                mapHandle.CData = occupancyMatrix(mapUnknown);
                title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
            end
            % Wait for the robot to begin again
            waitfor(controlRate);
        end
        
        
        %     headTo = 1;
        %     for j = 1:10
        %         while (headTo ~= 0)
        %             headToArray = [];
        %             xRan = rand*xsize;
        %             yRan = rand*ysize;
        %             headTo = checkOccupancy(mapUnknown,[xRan yRan]);
        %         end
        %         headToArray(j,1:2) = [xRan yRan];
        %     end
        %     endLocation = [xRan, yRan]
        %     headToArray
        
        
        % Startup PRM for path finding
        prm = robotics.PRM;
        prm.Map = mapUnknown;
        
        % The number of nodes and max connections
        prm.NumNodes = 250;
        prm.ConnectionDistance = 5;
        path = []
        occ = checkOccupancy(mapUnknown, robotPose(1:2))
        
        % If its current location is unknown or has an obstacle
        if (occ == 1 || occ == -1)
            startOcc = true;
        % Else if it is currently in a free location, switch to PRM Mode
        else
            startOcc = false;
            guessingCounter = 1;
            % If the path is empty, keep guessing untill it is stuck
            while (isempty(path) && guessingCounter < 10)
                startLocation = position;       % The current robot position
                % Finds the next location to use path planning to go to
                locArray = FindNextLocation(mapUnknown, guessingCounter)
                % Finds the furthest point in that square to head towards
                endLocation = FindFarthestPoint(locArray, robotPose);
                guessingCounter = guessingCounter + 1
                distanceToEnd = norm(endLocation - startLocation)
                % If the position that it finds is to far away or null,
                % repeat the loop
                if (endLocation == [0 0])
                    continue
                elseif (15 < distanceToEnd)
                    continue
                end
                % Find a path between the points and show it
                path = findpath(prm, startLocation, endLocation)
                prm.NumNodes = prm.NumNodes + 50;
            end
            % If the guesses are maxed out, go into random exploration mode
            if (guessingCounter == 10)
                startOcc = true;
            end
        end
        % Set the start and end location
    end

    % Show the end map that has been explored
    %     show(mapUnknown, 'Parent', axesHandle);
    %     title(axesHandle, 'OccupancyGrid: Final Map');
    %
    
    %endLocation = [2 2];
    %show(prm)
    
    % Setup the controller to follow the path
    controller = robotics.PurePursuit
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 1000;
    controller.MaxAngularVelocity = 10;
    controller.LookaheadDistance = 0.5;
    
    % Setup the robots position using its last known robotPose
    robotCurrentLocation = path(1,:);
    robotGoal = path(end,:);
    robotCurrentPose = robotPose;
    
    % Sets the robot pose
    sim.setRobotPose(robotCurrentPose);
    
    % Set and measure the goal distance
    goalRadius = 0.5;
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
    
    timeoutCounter = 1
    % While not close to the goal and not timed out, repeat
    while( distanceToGoal > goalRadius && timeoutCounter < 250)
        % Computes the controller outputs, inputs to the robot
        [v, omega] = controller(sim.getRobotPose);
        
        
        % Get the bumper sensor
        bump = receive(bumperSub);
        % If not touching anything, go in a straight line
        if (bump.Data == false)
             % Simluate the robot using the controller outputs
            drive(sim, v, omega);
        else
            drive(sim, 0, 10);
            % velMsg.Angular.Z = 10.0;
        end
        
        % Extract current logcation informaiton
        robotCurrentPose = sim.getRobotPose;
        
        % Get the scan message
        scanMsg = receive(scanSub);
        
        % Extract the laser scan
        scan = lidarScan(scanMsg);
        ranges = scan.Ranges; % Scans across the ranges of the lazer
        ranges(isnan(ranges)) = sim.LaserSensor.MaxRange; % If nan replace with its max range
        modScan = lidarScan(ranges, scan.Angles);
        
        % Insert the laser range observation in the map
        insertRay(mapUnknown, robotCurrentPose, modScan, sim.LaserSensor.MaxRange);
        
        % Re-compute the distnace to the goal
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
        
        timeoutCounter = timeoutCounter + 1;
        
        % Every 10 timeslices update the map to show the robots current
        % knowledge
        if ~mod(timeoutCounter,10)
            mapHandle.CData = occupancyMatrix(mapUnknown);
            title(axesHandle, ['OccupancyGrid: PRM ' num2str(timeoutCounter)]);
        end
        
        % Wait for the robots next turn
        waitfor(controlRate);
    end
    controller = false;
end