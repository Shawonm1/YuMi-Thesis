%%Load Robot Visualization and Build Environment
clear
clc
robot = loadrobot('abbYumi', 'DataFormat','row','Gravity', [0 0 -9.81]);
ax = gca
%Define Waypoints for a Trajectory
load abbYumiSaveTrajectoryWaypts.mat
currentRobotJConfig = startingConfig;
 
%Get numbe of joints
numJoints = numel(currentRobotJConfig);
endEffector = "gripper_r_base";

%Specify the time step and tool speed
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s

%Set the inital and end pose of end-effector
jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit',endEffector);

%%Object position and orientation 
 %plane = collisionBox(0.5,1,0.05);
 %plane.Pose = trvec2tform([0.25 0 -0.025]);
 %show(plane,'Parent', ax);
object = collisionBox(0.25,0.1,0.2);
object.Pose = trvec2tform([0.3 -.65 0.1]);
[~, patchObj] = show(object,'Parent',ax);
patchObj.FaceColor = [0 0 1];
%taskFinal = trvec2tform([0.4,0,0.6])*axang2tform([0 1 0 pi]);
taskFinal = object.Pose*axang2tform([0 1 0 pi/2]);

%%Genertae a task space trajectory
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

%Define trajectory time based on distance and speed
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

%Interpolate the waypoints
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 

%%Control task-space motion
%Create a motion model for PD control on the joints
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','gripper_r_base');

%Set the proportional and derivitive gains on the orientation to zero
tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;

%Define the inital state
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));
%ODE Solver to update the interpolate inputs to the state derivative at
%each instant
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);

%%Visualize robot trajectories
%Show inital configuration of the robot
show(robot,currentRobotJConfig','PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);

%Interpolate based on the current time and iterate through the staeTask
for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20)
    drawnow;
end
