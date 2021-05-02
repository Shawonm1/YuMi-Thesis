%%Load Robot
robot = loadrobot('abbYumi', 'Gravity', [0 0 -9.81])
iviz = interactiveRigidBodyTree(robot);
ax = gca;
%%Initialize Shared Simulation Parameters
load abbSavedConfigs.mat configSequence
% Define initial state
q0 = configSequence(:,1); % Position
dq0 = zeros(size(q0)); % Velocity
ddq0 = zeros(size(q0)); % Acceleration
%% Create a set of desired waypoints
wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02];
wayPointVels = [0 0 0;0 0.1 0;0 0 0];
plot(wayPoints);
%% Create a smooth trajectory from the waypoints
waypointTime = 4;
wpTimes = (0:size(wayPoints,1)-1)*waypointTime;

trajTimes = linspace(0,wpTimes(end),numTotalPoints);
        trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     wayPointVels');
% Plot trajectory spline and waypoints
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
%% Simulate the Model
 simout = sim('modelWithSimplifiedSystemDynamics.slx');
% Visualize the motion using the interactiveRigidBodyTree object.
iviz.ShowMarker = false;
iviz.showFigure;
rateCtrlObj = rateControl(length(simout.tout)/(max(simout.tout)));
for i = 1:length(simout.tout)
    iviz.Configuration = simout.yout{1}.Values.Data(i,:);
    waitfor(rateCtrlObj);
end
%JointTourque
%load robot lbr
robot.DataFormat = 'row';
%robot.Gravity = [0 0 -9.81];
q = randomConfiguration(robot);
tau = inverseDynamics(robot,q);
plot(tau);