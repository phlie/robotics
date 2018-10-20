image = imread('maze.png');

% Convert the image to black and white
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Load the map
robotRadius = 0.2;
map = robotics.BinaryOccupancyGrid(bwimage)

% Display the map
show(map)

% Make the map bigger to account for robot radius
mapInflated = copy(map);
inflate(mapInflated,robotRadius);

show(mapInflated)

% Load the bigger map into the prm Map slot
prm = robotics.PRM
prm.Map = mapInflated;

% Set starting number of nodes
prm.NumNodes = 50;

% The max length the robot can travel
prm.ConnectionDistance = 1000;

% Start and end condition
startLocation = [10 100];
endLocation = [10 950];

% Attempt to find the path with the current setup, if not loop
path = findpath(prm, startLocation, endLocation)
while isempty(path)
    % Slowly increment the number of prm Nodes
    prm.NumNodes = prm.NumNodes + 10;
    % Update PRM
    update(prm)
    path = findpath(prm, startLocation, endLocation);
end
% Show the found path
show(prm)
display(prm.NumNodes)
