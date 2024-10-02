classdef GripperBoxRight < RobotBaseClass
    %% Gripper model by Carter-Davies modified by Bihan Yethmin Sudusinghe 14540468
    % Reference:
    % Carter-Davies, D. Electric Robotic Gripper - UR3e. GRABCAD. https://grabcad.com/library/electric-robotic-gripper-ur3e-1 

    properties(Access = public)   
        plyFileNameStem = 'GripperBoxRight';
    end

    methods

%% Constructor
        function self = GripperBoxRight(baseTr)
            hold on;
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)

            link(1) = Link('d',0.05,'a',0,'alpha',pi/2,'qlim',[0 0],'offset',0);
            link(2) = Link('d',0,'a',0.03,'alpha',0,'qlim',[0 pi/4],'offset',0);
            
            link(1).qlim = [0 0];
            link(2).qlim = [-pi/4 pi/2];

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end


