close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
%% setting up robot and showing volume and max reach
workspace = [-3 3 -3 3 -0.6 2];

% Table = Environment('table',workspace,transl(0,0,-0.42));
% hold on;
% Fence = Environment('fence',workspace, transl(0,2,0));
% Fence2 = Environment('fencerot',workspace, transl(-2,0,0));
% Fence3 = Environment('fence2',workspace, transl(0,-2,0));
% Fence4 = Environment('fencerot2',workspace, transl(2,0,0));
tophousing = Environment('top', workspace, transl(-0.42, 0.46, 0));
bottomhousing = Environment('bottom', workspace, transl(0.42, 0.46,0));
circircuitboard = Environment('circuit', workspace, transl(0.3, 0.3, 0));
pause(0.01);
hold on;
location1 = transl(-0.23, 0.23, 0);
UR3_1 = UR3Model(workspace, location1)
hold on;
disp('press enter to load arm 2')
pause;

location2 = transl(0.23, 0.23, 0);
UR3_2 = UR3Model(workspace, location2)
hold on;
% disp('calculating volume point cloud...')

% UR3_1.getVolume()
%  disp('Press enter to continue to max reach');
%  pause;
% UR3_1.getReach()
% disp('Press enter to continue');
% pause;
% delete(UR3_1.pointCloudPlot)
%% Both arms move

robotQ = zeros(1,6);
UR3_1.model.plot(robotQ);
UR3_2.model.plot(robotQ);
pause(0.01)
housingTop = transl(-0.42,0.46,0)*trotx(pi);  %translation matrix
housingTopQ = UR3_1.model.ikcon(housingTop, [1,1,1,0,0,0]);   %joint angles required for top circuit board position

housingBot = transl(-0.2,-0.4,0)*trotx(pi);   
housingBotQ = UR3_1.model.ikcon(housingBot, [1,1,1,0,0,0]);

circuitBoard = transl(0.3, 0.3,0)*trotx(pi);   
circuitBoardQ = UR3_2.model.ikcon(circuitBoard, [1,1,1,0,0,0]);

UR3_1pos = UR3_1.model.getpos()
UR3_2pos = UR3_2.model.getpos()

%ur3_1 pickup top housing
topmove = movement()
topmove.move(UR3_1pos, housingTopQ, 10, UR3_1.model)
UR3_1.model.fkine(UR3_1.model.getpos())
disp('Top Housing Picked up press enter to continue')
pause;

%ur3_2 pickup circuit board
circircuitmove = movement()
circircuitmove.move(UR3_2pos, circuitBoardQ, 10, UR3_2.model)
UR3_2.model.fkine(UR3_2.model.getpos())
disp('Circuit Board Picked up, press enter to continue')
pause;

%meet halfway
% halfwayPoint = ((UR3_1.model.base*UR3_2.model.base)/2)
% halfwayCoords = halfwayPoint*transl(0,0,0.8)
% halfwayMove = movement()
% halfwayMove.partmove(UR3_1.model.fkine(UR3_1.model.getpos()), UR3_1.model, housingTopQ, 20)
% UR3_1.model.fkine(UR3_1.model.getpos())

%pickup bottom
%put in box

