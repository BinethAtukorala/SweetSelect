classdef UR3Class
    properties
        Robot_UR3 % Property to store the robot object model
    end

    methods
        %% Constructor to initialize the robot model
        function obj = UR3Class()
            % Initialize the robot model UR3
            % The robot is placed at a translational offset on top of the table
            obj.Robot_UR3  = UR3(transl([1, 0, 0.5]));
        end
    end
end
