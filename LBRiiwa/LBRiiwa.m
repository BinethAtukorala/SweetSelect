classdef LBRiiwa < RobotBaseClass
    %%  Kuka LBR iiwa 7 R800 robot model
    properties(Access = public)   
        plyFileNameStem = 'LBRiiwa';
    end 
    
    methods
%% Constructor
        function self = LBRiiwa(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            % self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();         
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.255,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',pi/2,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0.300,'a',0,'alpha',pi/2,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.300,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',	0,'a',0,'alpha',pi/2,'qlim',deg2rad([-360,360]), 'offset', 0);
            link(7) = Link('d',	0.0945,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            % Incorporate joint limits
            link(1).qlim = [-170 170]*pi/180;
            link(2).qlim = [-120 120]*pi/180;
            link(3).qlim = [-170 170]*pi/180;
            link(4).qlim = [-120 120]*pi/180;
            link(5).qlim = [-170 170]*pi/180;
            link(6).qlim = [-120 120]*pi/180;
            link(7).qlim = [-175 175]*pi/180;
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
