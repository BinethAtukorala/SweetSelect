classdef LBRiiwaClass
    properties
        Robot_LBRiiwa % Property to store the robot object model
    end

    methods
        %% Constructor to initialize the robot model
        function obj = LBRiiwaClass()
            % Initialize the robot model LBRiiwa
            % The robot is placed at a translational offset on top of the table
            obj.Robot_LBRiiwa  = LBRiiwa(transl([0, 0, 0.5]));
        end
    end
end
