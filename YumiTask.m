%%Load robot
clc
close all
robot = loadrobot('abbYumi','Gravity', [0 0 -9.81]);
iviz = interactiveRigidBodyTree(robot);
% ax = gca;
% showdetails(robot)
% config = randomConfiguration(robot)
% tform = getTransform(robot,config,'gripper_l_base','yumi_link_1_l')
% 
% %Define the Trajectory
% t = (0:0.2:10)'; % Time
% count = length(t);
% center = [0.3 0.1 0];
% radius = 0.15;
% theta = t*(2*pi/t(end));
% points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
% %Inverse Kinematics Solution
% q0 = homeConfiguration(robot);
% ndof = length(q0);
% qs = zeros(count, ndof);
% ik = inverseKinematics('RigidBodyTree', robot);
% weights = [0, 0, 0, 1, 1, 0];
% endEffector = 'gripper_l_base';
% qInitial = q0; % Use home configuration as the initial guess
% for i = 1:count
%     % Solve for the configuration satisfying the desired end effector
%     % position
%     point = points(i,:);
%     configSol = ik(endEffector,trvec2tform(point),weights,qInitial);
%     % Store the configuration
%     qs(i,:) = configSol(:).JointPosition;
%     % Start from prior solution
%     qInitial = configSol;
% end
% figure
% % show(robot,qs(1,:)');
% % view(2)
%  ax = gca;
%  ax.Projection = 'orthographic';
%  hold on
%  plot(points(:,1),points(:,2),'k')
%  axis([-0.1 0.7 -0.3 0.5])
%  framesPerSecond = 15;
%  r = rateControl(framesPerSecond);
%  for i = 1:count
%      show(robot,qs(i,:)','PreservePlot',false);
%      drawnow
%      waitfor(r);
%  end
