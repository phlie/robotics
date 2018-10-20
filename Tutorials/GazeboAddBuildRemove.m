ipaddress = '192.168.17.129'
rosinit(ipaddress)
gazebo = ExampleHelperGazeboCommunicator();

ball = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball,'sphere',1,'color',[0 0 1 1])

spawnModel(gazebo,ball,[8.5,0,1]);

x = [1.5    1.5    1.5   1.5   2.5     2.5  2.5    3.5    3.5   4.5];
y = [-1.5  -0.5    0.5   1.5   -1      0    1     -0.5    0.5   0];

pin = ExampleHelperGazeboModel('BowlPin');

link1 = addLink(pin,'cylinder',[1 0.2],'position',[0,0,0.5])
link2 = addLink(pin,'sphere',0.2,'position',[0,0, 1.2],'color',[0.7 0 0.2 1])

joint = addJoint(pin,link1,link2,'revolute',[0 0],[0 0 1])

for i = 1:10
    spawnModel(gazebo,pin,[x(i),y(i),0.7]);
    pause(1);
end

if ismember('mobile_base',getSpawnedModels(gazebo))
    removeModel(gazebo,'mobile_base');
end

barrier = ExampleHelperGazeboModel('jersey_barrier','gazeboDB');

spawnModel(gazebo,barrier,[1.5,-3,0]); % Right barrier
pause(1);
spawnModel(gazebo,barrier,[1.5,3,0]); % Left barrier

spawnedBall = ExampleHelperGazeboSpawnedModel(ball.Name,gazebo)

duration = 1; % Seconds
forcevec = [-75 0 0]; % Newtons

applyForce(spawnedBall, spherelink, duration, forcevec);

