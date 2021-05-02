%%FK & IK
function q = ik (A06)
clc
clear
% declaration of the dh parameters, you can give your own dh paras or edit
% it
a1 = 30; d1 = 166; alpha1 = -pi/2; th1 = 0;
a2 = 30; d2 = 0; alpha2 = -pi/2; th2 = 0;
a3 = 40.5; d3 = 251.5; alpha3 = pi/2; th3 = 0;
a4 = 40.5; d4 = 0; alpha4 = pi/2; th4 = 0;
a5 = 27; d5 = 265; alpha5 = pi/2; th5 = 0;
a6 = 27; d6 = 0; alpha6 = pi/2; th6 = 0;
a7 = 0; d7 =120; alpha7 = 0;th7 = ;
k = cos(alpha2);
phi = pi/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Limits of th2, th3 & th5
th1_min = 0;
th1_max = 2*pi;
th2_min = 0;
th2_max = 160*pi/180;
th3_min = 0;
th3_max = 160*pi/180;
th4_min = 0;
th4_max = 160*pi/180; 
th5_min = 0;
th5_max = 300*pi/180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Homogenous Transformation Matrix
for th1 = [th1_min:70*pi/180:th1_max]
for th2 = [th2_min:70*pi/180:th2_max]
for th3 = [th3_min:70*pi/180:th3_max]
for th4 = [th4_min:70*pi/180:th4_max]
for th5 = [th5_min:70*pi/180:th5_max]
A01 = [cos(th1) -cosd(alpha1)*sin(th1) sind(alpha1)*sin(th1) a1*cos(th1);
sin(th1) cosd(alpha1)*cos(th1) -sind(alpha1)*cos(th1) a1*sin(th1);
0 sind(alpha1) cosd(alpha1) d1;
0 0 0 1];
A12 = [cos(th2) -cosd(alpha2)*sin(th2) sind(alpha2)*sin(th2) a2*cos(th2);
sin(th2) cosd(alpha2)*cos(th2) -sind(alpha2)*cos(th2) a2*sin(th2);
0 sind(alpha2) cosd(alpha2) d2;
0 0 0 1];
A23 = [cos(th3) -cosd(alpha3)*sin(th3) sind(alpha3)*sin(th3) a3*cos(th3);
sin(th3) cosd(alpha3)*cos(th3) -sind(alpha3)*cos(th3) a3*sin(th3);
0 sind(alpha3) cosd(alpha3) d3;
0 0 0 1];
A34 = [cos(th4) -cosd(alpha4)*sin(th4) sind(alpha4)*sin(th4) a4*cos(th4);
sin(th4) cosd(alpha4)*cos(th4) -sind(alpha4)*cos(th4) a4*sin(th4);
0 sind(alpha4) cosd(alpha4) d4;
0 0 0 1];
A45 = [cos(th5) -cosd(alpha5)*sin(th5) sind(alpha5)*sin(th5) a5*cos(th5);
sin(th5) cosd(alpha5)*cos(th5) -sind(alpha5)*cos(th5) a5*sin(th5);
0 sind(alpha5) cosd(alpha5) d5;
0 0 0 1];
A56 = [cos(th6) -cosd(alpha6)*sin(th6) sind(alpha6)*sin(th6) a6*cos(th6);
sin(th6) cosd(alpha6)*cos(th6) -sind(alpha6)*cos(th6) a6*sin(th6);
0 sind(alpha6) cosd(alpha6) d6;
0 0 0 1];
A06=A01*A12*A23*A34*A45*A56;
X=A06(1,4);
Y=A06(2,4);
Z=A06(3,4);
%Forward kinematics
A06
a1=0; % [m]
a2=0.5; % [m]
a3=0.4; % [m]
d1=0.1; % [m]
alfa1=pi/2; % [rad]
alfa2=pi; % [rad]
alfa3=-pi/2; % [rad]
alfa4=pi/2; % [rad]
alfa5=-pi/2; % [rad]
alfa6=pi/2; % [rad]
% dk=[n s a p; 0 0 0 1] 
% n, s, a: They are 3 vector for the end-effector's orientation
dk=A06; % Position and orientation of end-effector
n=dk(1:3,1);
s=dk(1:3,2);
a=dk(1:3,3);
R=[n s a];
dk=A06; % Direct kinematics matrix
% Inverse Kinematic
p_ot=dk(1:3,4); % End-effector's position
pw=p_ot-d1*a; % Wrist's position
pw_x=dk(1,4); % Vector's components that representes the wrist's position
pw_y=dk(2,4);
pw_z=dk(3,4);
c3=(pw_x^2+pw_y^2+pw_z^2-a2^2-a3^2)/(2*a2*a3); % cos(teta3)
s3=-sqrt(1-c3^2); % sin(teta3)
teta3=atan2(real(s3),real(c3));
c2=(sqrt(pw_x^2+pw_y^2)*(a2+a3*c3)+pw_z*a3*s3)/(a2^2+a3^2+2*a2*a3*c3); % cos(teta2)
s2=(pw_z*(a2+a3*c3)-sqrt(pw_x^2+pw_y^2)*a3*s3)/(a2^2+a3^2+2*a2*a3*c3); % sin(teta2)
teta2=atan2(real((a2+a3*c3)*pw_z-a3*s3*sqrt(pw_x^2+pw_y^2)),real((a2+a3*c3)*sqrt(pw_x^2+pw_y^2)+a3*s3*pw_z));
teta1=atan2(pw_y,pw_x);
R3_0=[cos(teta1)*cos(teta2+teta3) -cos(teta1)*sin(teta2+teta3) sin(teta1); 
sin(teta1)*cos(teta2+teta3) -sin(teta1)*sin(teta2+teta3) -cos(teta1);
sin(teta2+teta3) cos(teta2+teta3) 0];
R6_3=R3_0*R; % Matrix for the Euler's angle of spherical wrist
% Inverse kinematic for the spherical wrist
teta4=atan2(R6_3(2,3),R6_3(1,3));
teta5=atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3));
teta6=atan2(R6_3(3,2),R6_3(3,1));
q=[teta1 teta2 teta3 teta4 teta5 teta6] 
end
end
end
end
end
end