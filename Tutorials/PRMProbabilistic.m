filePath = fullfile(fileparts(which('PathPlanningExample')),'data','exampleMaps.mat');
load(filePath);
robotRadius = 0.2;
map = robotics.BinaryOccupancyGrid(ternaryMap, 20)

show(map)

mapInflated = copy(map);
inflate(mapInflated,robotRadius);

show(mapInflated)

prm = robotics.PRM
prm.Map = mapInflated;

prm.NumNodes = 60;

prm.ConnectionDistance = 5;

startLocation = [7 22];
endLocation = [20 5];

path = findpath(prm, startLocation, endLocation)
while isempty(path)
    prm.NumNodes = prm.NumNodes + 10;
    update(prm)
    path = findpath(prm, startLocation, endLocation);
end
show(prm)
display(prm.NumNodes)
