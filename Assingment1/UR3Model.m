classdef UR3Model < handle % setup and move the UR3 robot, as well as log its transforms
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;   
    end
    
    methods
        function self = UR3Model(workspace,location)
            self.workspace = workspace;
            self.GetRobot();
            self.currentJoints = zeros(1,6);
            self.model.base = location;
            self.PlotAndColour(self.location);
            
            
        end
        function PlotAndColour(self,location)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.currentJoints,'workspace',self.workspace,'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
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
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',deg2rad([-360 360]));
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',deg2rad([-360 360]));
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]));
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360 360]));
            
            pause(0.0001)
            name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);             
        end
    end
end