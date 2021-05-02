%%Load Robot
robot = loadrobot('abbYumi', 'Gravity', [0 0 -9.81]);
%%Initialize Shared Simulation Parameters
load abbSavedConfigs.mat configSequence
% Define initial state
q0 = configSequence(:,1); % Position
dq0 = zeros(size(q0)); % Velocity
ddq0 = zeros(size(q0)); % Acceleration
%%Waypoints
wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2;0.2 0.2 0.2]
hold on
%exampleHelperPlotWaypoints(wayPoints);
%Create a smooth cuve from the waypoints to serve as trajectory
numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;
trajType = 'cubic'; % or 'trapezoidal'
switch trajType
    case 'trapezoidal'
        trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
    case 'cubic'
        wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
        trajTimes = linspace(0,wpTimes(end),numTotalPoints);
        trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     );
end
% Plot trajectory spline and waypoints
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

%%Plot trajectory spline  7:42
fnplt(trajectory,'r',2);
%% Perform Inverse Kinematics for a point in space
%Add end effector frame, offset from the grip link frame
eeOffset = 0.12;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
addBody(robot,eeBody,'link5');
%% Perform Inverse Kinematics
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;
