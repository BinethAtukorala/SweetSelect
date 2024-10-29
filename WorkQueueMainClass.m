classdef WorkQueueMainClass
    properties
        Initial_Poses
        Object
        Final_Poses
        Candy_No
        Is_UR3e
        Is_Front % Front or back
        Is_To_Midway % To or from midway
    end

    methods
        function obj = WorkQueueMainClass(Initial_Poses, Object, Is_UR3e, options)
            arguments
                Initial_Poses
                Object
                isUR3e logical
                options.Is_Front
                options.Is_To_Midway
                options.Final_Poses
                options.Candy_No
            end
            obj.Initial_Poses = Initial_Poses;
            obj.Object = Object;
            obj.Is_UR3e = Is_UR3e;
            if Is_UR3e  % If UR3e
                obj.Final_Poses = options.Final_Poses;
                obj.Candy_No = options.Candy_No;
            else    % If LBRiiwa 
                obj.Is_Front = options.Is_From_Front;
                obj.Is_To_Midway = options.Is_To_Midway;
            end
        end
    end
end




% if LBRiiwaClass
%     UR3e.executeQueue()
% 
% else 
% 
%     LBRiiwa.executeQueue()
% 
% 
% 
% fajskdhfk