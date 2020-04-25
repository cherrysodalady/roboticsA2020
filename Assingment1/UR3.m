close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear
%% 
workspace = [-2 2 -2 2 -0.3 2]; 
location1 = transl(0, 0, 0);
UR3_1 = UR3Model(workspace, location1)
hold on;
UR3_1.getVolume()

%% DH model for UR3

% % 
% x1 = input('UR3-1 base x coordinate')
% y1 = input('UR3-1 base y coordinate')
% z1 = input('UR3-1 base z coordinate')
% 
% myRobot.base = transl(x1, y1, z1);


% myRobot2 = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR3_2')
% myRobot2.base = transl(0.5, 0.5, 0)
% myRobot2.plot(q)
%display('Press enter to continue');
%pause;
%% Show max reach + approximate volume
% sphereCenter = [1,1,0];
% sphereL1Transform = myRobot.base() + myRobot.A(1,q) %position of link 1(in terms of base) + global distance of base
% sphereCenter = [sphereL1Transform(1:3,4)]
% radius = 0.57275
% [X,Y,Z] = sphere(10);
% X = X * radius + sphereCenter(1);
% Y = Y * radius + sphereCenter(2);
% Z = Z * radius + sphereCenter(3);
% 
% tri = delaunay(X,Y,Z);
% sphereTri_h = trimesh(tri,X,Y,Z);
% set(sphereTri_h, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.2)
% 
% drawnow();
% view(3)
% axis equal
% baseTransform = myRobot.base(3,4)+ 0.57275
% volume = (4/3*pi*baseTransform^3)/2

%% animate the arm in the bubble to some positions
% [Q,ERR,EXITFLAG] = robot.ikcon(T, Q0) as above but specify the
%  initial joint coordinates Q0 used for the minimisation.
robotQ = zeros(1,6)
myRobot.plot(robotQ);
housingTop = transl(0.3,0.5,0)*trotx(pi)  %translation matrix
housingTopQ = myRobot.ikcon(housingTop) %joint angles required for top circuit board position
housingBot = transl(-0.2,-0.4,0)*trotx(pi)   
housingBotQ = myRobot.ikcon(housingBot)
circuitBoard = transl(0.1, 0.2,0)*trotx(pi)   
circuitBoardQ = myRobot.ikcon(circuitBoard)

qMatrix = jtraj(myRobot.getpos(),circuitBoardQ,30) %creates a big matrix with positions between start and end

for trajStep = 1:size(qMatrix,1)
    q = qMatrix(trajStep,:);    %each joint position line by line
    myRobot.animate(q);
    pause(0.01);
    
%     goalQ = [0,pi/2,0,jointMidRadians(4:6) + 0.5 * (rand(1,3)-0.5) .* (qlim(4:end,2)' - qlim(4:end,1)')];
%     
%     % Get a trajectory
%     jointTrajectory = jtraj(densoRobot.getpos(),goalQ,20);
%     for trajStep = 1:size(jointTrajectory,1)
%         q = jointTrajectory(trajStep,:);
%         densoRobot.animate(q);    
%             
%         blastStartTr = densoRobot.fkine(q);
%         blastStartPnt = blastStartTr(1:3,4)';
%         blastEndTr = densoRobot.fkine(q) * transl(0,0,1);
%         blastEndPnt = blastEndTr(1:3,4)';
%     
%     %then use fkine to confirm the end effector got to the desird position
end
%     
% for trajStep = 1:size(qMatrix,1)
%     q = qMatrix(trajStep,:);
%     myRobot.animate(q);
%     pause(0.01);
%     trajStep
%     %then use fkine to confirm the end effector got to the desird position
% end
% 
% for trajStep = 1:size(qMatrix,1)
%     q = qMatrix(trajStep,:);
%     myRobot.animate(q);
%     pause(0.01);
%     trajStep
%     %then use fkine to confirm the end effector got to the desird position
% end
% 
% for trajStep = 1:size(qMatrix,1)
%     q = qMatrix(trajStep,:);
%     myRobot.animate(q);
%     pause(0.01);
%     trajStep
%     %then use fkine to confirm the end effector got to the desird position
 %end



%% 

