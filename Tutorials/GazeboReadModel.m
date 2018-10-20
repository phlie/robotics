ipaddress = '192.168.17.129'
rosinit(ipaddress)
rostopic list

gazebo = ExampleHelperGazeboCommunicator();
phys = readPhysics(gazebo)

phys.Gravity = [0 0 0.1];
setPhysics(gazebo,phys);
resetSim(gazebo);
pause(5);

pauseSim(gazebo);
pause(3);

pauseSim(gazebo);
phys = readPhysics(gazebo);
phys.UpdateRate = phys.UpdateRate/8;
setPhysics(gazebo,phys);

phys.Gravity = [0 0 -9.8];   % Set gravity back to normal value (m/s^2)
setPhysics(gazebo,phys);
resumeSim(gazebo);
pause(5);

phys.UpdateRate = phys.UpdateRate*8;
setPhysics(gazebo,phys);
pause(3);

resetSim(gazebo);

models = getSpawnedModels(gazebo)

kobuki = ExampleHelperGazeboSpawnedModel('mobile_base',gazebo)
[kobukiLinks, kobukiJoints] = getComponents(kobuki)

[position, orientation, velocity] = getState(kobuki)

