%% Plan and Execute Task- and Joint-Space Trajectories Using KINOVA Gen3 Manipulator
% This example shows how to generate and simulate interpolated joint trajectories 
% to move from an initial to a desired end-effector pose. The timing of the trajectories 
% is based on an approximate desired end of arm tool (EOAT) speed.
%% 
% Load the KINOVA Gen3 rigid body tree (RBT) robotL model.

robot = loadrobot('abbYumi','DataFormat','row','Gravity',[0 0 -9.81]);


robotL=robot.copy
robotL.removeBody('yumi_link_1_r')

robotL.getBody('gripper_l_base')

%%
% Define the environment, Create a visualization to replay simulated trajectories.
iviz = interactiveRigidBodyTree(robotL);
ax = gca;
exampleHelperSetupWorkspace(ax);

%predefined configurations, configSequence, as robot states
load abbSavedConfigs.mat configSequence
%Define initial state of the robot postion, velocity, and acceleration of each joint.

%% 
% Set current robotL joint configuration.

%currentrobotLJConfig = homeConfiguration(robotL); %Original
 currentrobotLJConfig = configSequence([10:18]);

%a=[0.0333,-1.7780,-0.0406,-0.7803,-0.0151,2.4086,0.0186,0,0,-0.0480,-0.2782,-0.0490 ,-0.3638, -0.0286,0.5147, -0.0060,0,0];
%currentrobotLJConfig=a;


%% 
% Get number of joints and the end-effector RBT frame.

numJoints = numel(currentrobotLJConfig);
endEffector = "gripper_l_base";
%% 
% Specify the trajectory time step and approximate desired tool speed.

timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s
%% 
% Set the initial and final end-effector pose.

jointInit = currentrobotLJConfig;
taskInit = getTransform(robotL,jointInit,endEffector);



%get cuurent pose of end efector
cuurentpos=tform2trvec(getTransform(robotL,currentrobotLJConfig,'gripper_l_base'));





%calculate new pose
newpose=cuurentpos;
newpose(3)=cuurentpos(3)-0.4;
newpose(2)=cuurentpos(2)-0.2;
newpose(1)=cuurentpos(1)-0.2;


%taskFinal = trvec2tform(newpose)*axang2tform([0 1 0 pi]);

taskFinal = trvec2tform([0.3 0.65 0.425])*axang2tform([0 1 0 pi]); %position of rightWidget and add 0.2 on Z value

%Initial state
% q0 = configSequence([10:18])'; % Position
% qd0 = zeros(size(q0)); % Velocity
% ddq0 = zeros(size(q0)); % Acceleration

%%Add controller
% open_system('ControllerAndBasicRobotDynamics.slx');


%%Simulate and visualize the results using the new model.
% simout = sim('ControllerAndBasicRobotDynamics.slx');
% 
% % Visualize the motion using the interactiveRigidBodyTree
% iviz.ShowMarker = false;
% iviz.showFigure;
% rateCtrlObj = rateControl(length(simout.tout)/(max(simout.tout)));
% for i = 1:length(simout.tout)
%     iviz.Configuration = simout.yout{1}.Values.Data(i,:);
%     waitfor(rateCtrlObj);
% end
%% Generate Task-Space Trajectory
% Compute task-space trajectory waypoints via interpolation.
% 
% First, compute tool traveling distance.

distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%% 
% Next, define trajectory times based on traveling distance and desired tool 
% speed.

initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];
%% 
% Interpolate between |taskInit| and |taskFinal| to compute intermediate task-space 
% waypoints.

[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
%% Control Task-Space Motion
% Create a joint space motion model for PD control on the joints. The |taskSpaceMotionModel| 
% object models the motion of a rigid body tree model under task-space proportional-derivative 
% control.

tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robotL,'EndEffectorName','gripper_l_base');
%% 
% Set the proportional and derivative gains on orientation to zero, so that 
% controlled behavior just follows the reference positions:

tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;
%% 
% Define the initial states (joint positions and velocities). 

q0 = currentrobotLJConfig;  %Original

dq0 = zeros(size(q0));  %Original


%% 
% Use |ode15s| to simulate the robotLR motion. For this problem, the closed-loop 
% system is stiff, meaning that there is a difference in scaling somewhere in 
% the problem. As a result, the integrator is sometimes forced to take exceedingly 
% small steps, and a non-stiff ODE solver such as |ode45| will take much longer 
% to solve the same problem. For more information, refer to <docid:matlab_math#bu1mo3d-1 
% Choose an ODE Solver> and <docid:matlab_math#bu22m86 Solve Stiff ODEs> in the 
% documentation.
% 
% Since the reference state changes at each instant, a wrapper function is required 
% to update the interpolated trajectory input to the state derivative at each 
% instant. Therefore, an example helper function is passed as the function handle 
% to the ODE solver. The resultant manipulator states are output in |stateTask|. 

[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; dq0]);
%% Generate Joint-Space Trajectory
% Create a inverse kinematics object for the robotLR.

ik = inverseKinematics('RigidBodyTree',robotL);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];


%% 
% Calculate the initial and desired joint configurations using inverse kinematics. 
% Wrap the values to pi to ensure that interpolation is over a minimal domain. 

initialGuess = jointInit;
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
%% 
% By default, the IK solution respects joint limits. However for continuous 
% joints (revolute joints with infinite range), the resultant values may be unnecessarily 
% large and can be wrapped to |[-pi, pi]| to ensure that the final trajectory 
% covers a minimal distance. Since non-continuous joints for the Gen3 already 
% have limits within this interval, it is sufficient to simply wrap the joint 
% values to |pi|. The continuous joints will be mapped to the interval |[-pi, 
% pi]|, and the other values will remain the same.

wrappedJointFinal = wrapToPi(jointFinal);
%% 
% Interpolate between them using a cubic polynomial function to generate an 
% array of evenly-spaced joint configurations. Use a B-spline to generate a smooth 
% trajectory. 

ctrlpoints = [jointInit',wrappedJointFinal'];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
jointWaypoints = bsplinepolytraj(jointConfigArray,timeInterval,1);
%% Control Joint-Space Trajectory
% Create a joint space motion model for PD control on the joints. The |jointSpaceMotionModel| 
% object models the motion of a rigid body tree model and uses proportional-derivative 
% control on the specified joint positions.

jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robotL,'MotionType','PDControl');
%% 
% Set initial states (joint positions and velocities). 

q0 = currentrobotLJConfig; 
dq0 = zeros(size(q0));
%% 
% Use |ode15s| to simulate the robotL motion. Again, an example helper function 
% is used as the function handle input to the ODE solver in order to update the 
% reference inputs at each instant in time. The joint-space states are output 
% in |stateJoint|.

[tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; dq0]);
%% Visualize and Compare robotL Trajectories
% Show the initial configuration of the robotL.

show(robotL,currentrobotLJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
%% 
% Visualize the task-space trajectory. Iterate through the |stateTask| states 
% and interpolate based on the current time.

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robotL,configNow,endEffector);
    show(robotL,configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end
%% 
% Visualize the joint-space trajectory. Iterate through the |jointTask| states 
% and interpolate based on the current time.

% Return to initial configuration
show(robotL,currentrobotLJConfig,'PreservePlot',false,'Frames','off');

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tJoint,stateJoint(:,1:numJoints),tNow);
    poseNow = getTransform(robotL,configNow,endEffector);
    show(robotL,configNow,'PreservePlot',false,'Frames','off');
    jointSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20);
    drawnow;
end

% Add a legend and title
legend([taskSpaceMarker jointSpaceMarker], {'Defined in Task-Space', 'Defined in Joint-Space'});
title('Manipulator Trajectories')
%% 
% _Copyright 2019-2020 The MathWorks, Inc._
% 
%