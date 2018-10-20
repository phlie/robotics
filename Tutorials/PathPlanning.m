filePath = fullfile(fileparts(which('PathPlanningExample')),'data','exampleMaps.mat');
load(filePath);
robotRadius = 0.2;
map = robotics.BinaryOccupancyGrid(complexMap, 1)

show(map)

mapInflated = copy(map);
inflate(mapInflated,robotRadius);

show(mapInflated)

prm = robotics.PRM
prm.Map = mapInflated;

prm.NumNodes = 500;

prm.ConnectionDistance = 5;

startLocation = [3 3];
endLocation = [5 35];

path = findpath(prm, startLocation, endLocation)

show(prm)
