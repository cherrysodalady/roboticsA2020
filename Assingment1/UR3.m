close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
%% setting up robot and showing volume and max reach
workspace = [-4 4 -4 4 -0.6 5];

Table = Objects('table',workspace,transl(0,0,-0.42));
hold on;
Fence = Objects('fence',workspace, transl(0,2,0));
Fence2 = Objects('fencerot',workspace, transl(-2,0,0));
Fence3 = Objects('fence2',workspace, transl(0,-2,0))
Fence4 = Objects('fencerot2',workspace, transl(2,0,0));
pause;

location1 = transl(0, 0, 0);
UR3_1 = UR3Model(workspace, location1)
pause(0.01)

location2 = transl(0.5, 0.5, 0);
UR3_2 = UR3Model(workspace, location2)
disp('calculating volume point cloud...')

% UR3_1.getVolume()
%  disp('Press enter to continue to max reach');
%  pause;
% UR3_1.getReach()
disp('Press enter to continue');
pause;
% delete(UR3_1.pointCloudPlot)
%% Both arms move

robotQ = zeros(1,6);
UR3_1.model.plot(robotQ);
pause
housingTop = transl(0.-0.1,0.3,0.1)*trotx(pi);  %translation matrix
housingTopQ = UR3_1.model.ikcon(housingTop, [1,1,1,0,0,0]);   %joint angles required for top circuit board position

housingBot = transl(-0.2,-0.4,0)*trotx(pi);   
housingBotQ = UR3_1.model.ikcon(housingBot, [1,1,1,0,0,0]);

circuitBoard = transl(0.1, 0.2,0)*trotx(pi);   
circuitBoardQ = UR3_1.model.ikcon(circuitBoard, [1,1,1,0,0,0]);

UR3_1pos = UR3_1.model.getpos()

moveUR3_1 = movement()
moveUR3_1.move(UR3_1pos, housingTopQ, 15, UR3_1.model)

disp('actually moved yass')

