classdef UR3Model < handle % setup and move the UR3 robot, as well as log its transforms
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData; 
        pointCloud;
        pointCloudPlot;
    end
    
    methods
        function self = UR3Model(workspace,location)
            self.workspace = workspace;
            self.GetRobot();
            self.currentJoints = zeros(1,6);
            self.model.base = location;
            self.PlotAndColour(self.location);
           % self.getVolume();
           % self.getReach();
        end
        
        function [totalVol] = getVolume(self)
            stepRads = deg2rad(30);
            qlim = self.model.qlim;
            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            pointCloud = zeros(pointCloudeSize,3);
            counter = 1;
            tic
            
            for q1 = qlim(1,1):stepRads:qlim(1,2)   %first row is first joints limits
                for q2 = qlim(2,1):stepRads:qlim(2,2) %second row is for second joints limits
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                % Don't need to worry about joint 6, just assume it=0
                                q6 = 0;
                                % for q6 = qlim(6,1):stepRads:qlim(6,2)
                                q = [q1,q2,q3,q4,q5,q6];
                                tr = self.model.fkine(q);
                                if(tr(3,4)>0)
                                    pointCloud(counter,:) = tr(1:3,4)';
                                end
                                counter = counter + 1;
                                if mod(counter/pointCloudeSize * 100,1) == 0
                                    display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                end
                                %                     end
                            end
                        end
                    end
                end
            end
            
            % 2.6 Create a 3D model showing where the end effector can be over all these samples.           
            self.pointCloudPlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            [k, totalVol] = convhull(pointCloud);
            self.pointCloud = pointCloud
        end
        
         function [maxReach] = getReach(self)
             zVals = self.pointCloud(:,3);
             maxZVal = max(zVals);
             maxReach = maxZVal;
         end
        
        function PlotAndColour(self,location)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.currentJoints,'workspace',self.workspace,'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'));
                camlight;
            end
            
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end    
        end
        
        function GetRobot(self) % Setup Robot Parameters
            pause(0.001);
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]));
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',deg2rad([-160 160]));
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',deg2rad([-180 180]));
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]));
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180 180]));
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360 360]));
            
            pause(0.0001)
            name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);
            
        end
    end
end