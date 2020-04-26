close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
%% setting up robot and showing volume and max reach
workspace = [-2 2 -2 2 0 2];
% x1 = input('UR3-1 base x coordinate')
% y1 = input('UR3-1 base y coordinate')
% z1 = input('UR3-1 base z coordinate')
% 
% location1 = transl(x1, y1, z1);
location1 = transl(0, 0, 0);
UR3_1 = UR3Model(workspace, location1)
pause(0.01)
hold on;
UR3_1.getVolume()
 display('Press enter to continue to max reach');
 pause;
UR3_1.getReach()
 display('Press enter to continue');
pause;
%% Both arms setup and move
% how to clear just the point cloud and move on?
%  
% x2 = input('UR3-2 base x coordinate')
% y2 = input('UR3-2 base y coordinate')
% z2 = input('UR3-2 base z coordinate')
% 
% location2 = transl(x2, y2, z1)
%UR3_2 = UR3Model(workspace, location2)
% start = ur3_1.self.getpos();
% bottom = (0.1, 0.2, 0)
% steps = 30
% UR3_1.move(start, bottom, steps)
% 
% Move(robotLocation,partLocation, steps)


%display('Press enter to continue');
%pause;