classdef movement
    %this class allows for control of the movement of the robot
    
    properties
        
    end
    
    methods
        function self = movement()
        end
        
        function move(self, robotLocation,partLocation, steps, robotModel)
            qMatrix = jtraj(robotLocation, partLocation, steps);
            for trajstep = 1:size(qMatrix, 1)
                q = qMatrix(trajstep,:); %each joint position line by line
                robotModel.animate(q);
                pause(0.01);
            end
        end
        function partmove(self, robotLocation, partLocation, steps, robotModel, partModel)
            qMatrix = jtraj(robotLocation, partLocation, steps);
            for trajstep = 1:size(qMatrix, 1)
                q = qMatrix(trajstep,:); %each joint position line by line
                robotModel.animate(q);
                partModel.model.base = robotModel.model.fkine(robotModel.model.getpos())
                partModel.animate(0); % just a one link robot
                pause(0.01);
            end
            
        end
        
    end
end
