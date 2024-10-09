classdef MainProgram
    properties
        r % Property to store the robot object model
    end

    methods
        %% Constructor to initialize the robot model
        function obj = MainProgram()
            % Initialize the robot model UR3e
            % The robot is placed at a translational offset on top of the table
            obj.r  = UR3e();

        end

        function putCandiesInBox(obj, A, B, C)
            fprintf("Candy A: %d\nCandy B: %d\nCandy C: %d\n", A, B, C)
        end

        function result = getQLims(obj)
            result = obj.r.model.qlim;
        end

        function setQValues(obj, q)
            obj.r.model.animate(q);
        end

        function [tr, rpy] = getTrAndRPY(obj, q)
            tr = obj.r.model.fkine(q);
            rpy = tr2rpy(tr);
            tr = tr.t
        end


    end

end
