classdef GripperCandyClass
    properties
        Candy_Right_Finger  % Property to store the model for the right finger of the gripper
        Candy_Left_Finger  % Property to store the model for the left finger of the gripper
    end

    methods
        %% Constructor to initialize the gripper models
        function obj = GripperCandyClass()
            % Initialize the right and left gripper finger models with base
            % positions at the origin
            obj.Candy_Right_Finger = GripperCandyRight(transl([0, 0, 0]));
            obj.Candy_Left_Finger = GripperCandyLeft(transl([0, 0, 0]));
        end
    end
end
