close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
%% setting up robot and showing volume and max reach
workspace = [-4 4 -4 4 -0.6 3];

% doCameraSpin = false;
% [f,v,data] = plyread('table.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

tableloc = transl(0, 0, -0.42)
barrierloc = transl (2, 2, 0)

%[table] = PlaceObject('table',tableloc)
table = Environment('table', workspace, tableloc)
hold on;
%barrier = Environment('barrier', workspace, 2, barrierloc)

location1 = transl(0, 0, 0);
UR3_1 = UR3Model(workspace, location1)
pause(0.01)
hold on;

location2 = transl(0.5, 0.5, 0);
UR3_2 = UR3Model(workspace, location2)
%disp('calculating volume point cloud...')

% UR3_1.getVolume()
%  disp('Press enter to continue to max reach');
%  pause;
% UR3_1.getReach()
%  disp('Press enter to continue');
% pause;
% delete(UR3_1.pointCloudPlot)
%% Both arms move
% [Q,ERR,EXITFLAG] = robot.ikcon(T, Q0) as above but specify the
%  initial joint coordinates Q0 used for the minimisation.
 robotQ = zeros(1,6);
UR3_1.model.plot(robotQ);
pause
housingTop = transl(0.-0.1,0.3,0.1)*trotx(pi);  %translation matrix
housingTopQ = UR3_1.model.ikcon(housingTop);   %joint angles required for top circuit board position

housingBot = transl(-0.2,-0.4,0)*trotx(pi);   
housingBotQ = UR3_1.model.ikcon(housingBot);

circuitBoard = transl(0.1, 0.2,0)*trotx(pi);   
circuitBoardQ = UR3_1.model.ikcon(circuitBoard);

UR3_1pos = UR3_1.model.getpos()

moveUR3_1 = movement()
moveUR3_1.move(UR3_1pos, housingTopQ, 15, UR3_1.model)

disp('actually moved yass')

