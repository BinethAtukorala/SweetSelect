classdef GripperCandyRight < RobotBaseClass


    properties(Access = public)   
        plyFileNameStem = 'GripperCandyRight';
    
    end
% Gripper by jjrobots(2019) modified by Binada Nethmin Sudusinghe, Link - https://www.thingiverse.com/thing:3648782#google_vignette
  
    methods
%% Define robot Function 
        function self = GripperCandyRight(baseTr)
            % figure;
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
            link(1) = Link('d',0.01, 'a',0, 'alpha',pi/2, 'qlim',deg2rad([0 0]), 'offset', 0);
            link(2) = Link('d',0, 'a',0, 'alpha',0, 'qlim', deg2rad([-90 90]), 'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
