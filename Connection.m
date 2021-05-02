%% Example communication

tc=tcpip('193.40.0.12',55000,'NetworkRole', 'server');

%Opening the communication
%fopen(tc);

%Receive a message from the robot

% message=fread(tc);

%Send a message to the robot

% fvrite(tc,'Hello server!');