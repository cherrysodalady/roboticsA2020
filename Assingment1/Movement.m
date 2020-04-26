classdef Movement
    %this class allows for control of the movement of the robot
    
    properties
        
    end
    % for trajStep = 1:size(qMatrix,1)
    %     q = qMatrix(trajStep,:);    %each joint position line by line
    %     myRobot.animate(q);
    %     pause(0.01);
    methods
        function Move(robotLocation,partLocation, steps)
            qMatrix = jtraj(robotLocation, partLocation, steps)
            for trajstep = 1:size(qMatrix, 1)
                q = qMatrix(trajstep,:); %each joint position line by line
                self.animate(q);
                pause(0.01);
            end
        end
        
        
    end
end

%% animate the arm in the bubble to some positions
% [Q,ERR,EXITFLAG] = robot.ikcon(T, Q0) as above but specify the
%  initial joint coordinates Q0 used for the minimisation.
% robotQ = zeros(1,6)
% myRobot.plot(robotQ);
% housingTop = transl(0.3,0.5,0)*trotx(pi)  %translation matrix
% housingTopQ = myRobot.ikcon(housingTop) %joint angles required for top circuit board position
% housingBot = transl(-0.2,-0.4,0)*trotx(pi)   
% housingBotQ = myRobot.ikcon(housingBot)
% circuitBoard = transl(0.1, 0.2,0)*trotx(pi)   
% circuitBoardQ = myRobot.ikcon(circuitBoard)
% 
% qMatrix = jtraj(myRobot.getpos(),circuitBoardQ,30) %creates a big matrix with positions between start and end
% 
% for trajStep = 1:size(qMatrix,1)
%     q = qMatrix(trajStep,:);    %each joint position line by line
%     myRobot.animate(q);
%     pause(0.01);

% for trajStep = 1:size(qMatrix,1)
%     q = qMatrix(trajStep,:);
%     myRobot.animate(q);
%     pause(0.01);
%     trajStep
%     %then use fkine to confirm the end effector got to the desird position
% end