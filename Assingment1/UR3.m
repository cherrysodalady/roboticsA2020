close all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf
clear

%% DH model for UR3
% https://github.com/petercorke/robotics-toolbox-matlab/blob/master/models/mdl_ur3.m?fbclid=IwAR0oCubw8Y2PEoGO9inW7-8GlEHgH0a37CPgMK8oKjSJIEADVc8BU8d-NcY

% workspace = [-2 2 -2 2 -0.3 2]; 
% myRobot = UR3Model(workspace,transl(0,0,0));

L1 = Link('d',0.1519,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset',0);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',deg2rad([-360 360]),'offset',0);
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',deg2rad([-360 360]),'offset',0);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset',0);
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset',0);
L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360 360]),'offset',0);
% 
myRobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR3');
q = zeros(1,6);
% 
x1 = input('UR3-1 base x coordinate')
y1 = input('UR3-1 base y coordinate')
z1 = input('UR3-1 base z coordinate')

myRobot.base = transl(x1, y1, z1);
myRobot.plot(q)
%myRobot.teach;
hold on;
% load('pcloudmatrix.mat')
% 
% [k, totalVol] = convhull(pointCloud);
% totalVol
% 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
% stepRads = deg2rad(30);
% qlim = myRobot.qlim;
% % Don't need to worry about joint 6
% pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
% pointCloud = zeros(pointCloudeSize,3);
% counter = 1;
% tic

% for q1 = qlim(1,1):stepRads:qlim(1,2)   %first row is first joints limits
%     for q2 = qlim(2,1):stepRads:qlim(2,2) %second row is for second joints limits
%         for q3 = qlim(3,1):stepRads:qlim(3,2)
%             for q4 = qlim(4,1):stepRads:qlim(4,2)
%                 for q5 = qlim(5,1):stepRads:qlim(5,2)
%                     % Don't need to worry about joint 6, just assume it=0
%                     q6 = 0;
% %                     for q6 = qlim(6,1):stepRads:qlim(6,2)
%                         q = [q1,q2,q3,q4,q5,q6];
%                         tr = myRobot.fkine(q);                        
%                         pointCloud(counter,:) = tr(1:3,4)';
%                         counter = counter + 1; 
%                         if mod(counter/pointCloudeSize * 100,1) == 0
%                             display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
%                         end
% %                     end
%                 end
%             end
%         end
%     end
% end

% 2.6 Create a 3D model showing where the end effector can be over all these samples.  
%plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

% myRobot2 = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR3_2')
% myRobot2.base = transl(0.5, 0.5, 0)
% myRobot2.plot(q)
display('Press enter to continue');
pause;
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

