close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
%% setting up robot and showing volume and max reach
workspace = [-2 2 -2 2 -0.6 2];

tableloc = transl(0, 0, -0.4154)

model = Environment('table', workspace, tableloc)

location1 = transl(0, 0, 0);
UR3_1 = UR3Model(workspace, location1)
pause(0.01)
hold on;

location2 = transl(0.5, 0.5, 0);
UR3_2 = UR3Model(workspace, location2)
disp('calculating volume point cloud...')

UR3_1.getVolume()
 disp('Press enter to continue to max reach');
 pause;
UR3_1.getReach()
 disp('Press enter to continue');
pause;
delete(UR3_1.pointCloudPlot)
%% Both arms move
% [Q,ERR,EXITFLAG] = robot.ikcon(T, Q0) as above but specify the
%  initial joint coordinates Q0 used for the minimisation.
 robotQ = zeros(1,6);
%UR3_1.plot(robotQ);

housingTop = transl(0.-0.1,0.3,0)*trotx(pi);  %translation matrix
housingTopQ = UR3_1.ikcon(housingTop);   %joint angles required for top circuit board position

housingBot = transl(-0.2,-0.4,0)*trotx(pi);   
housingBotQ = UR3_1.ikcon(housingBot);

circuitBoard = transl(0.1, 0.2,0)*trotx(pi);   
circuitBoardQ = UR3_1.ikcon(circuitBoard);




