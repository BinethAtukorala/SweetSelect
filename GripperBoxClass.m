classdef GripperBoxClass
    properties
        Box_Right_Finger  % Property to store the model for the right finger of the gripper
        Box_Left_Finger  % Property to store the model for the left finger of the gripper
    end

    methods
        %% Constructor to initialize the gripper models
        function obj = GripperBoxClass()
            % Initialize the right and left gripper finger models with base
            % positions at the origin
            obj.Box_Right_Finger = GripperBoxRight(transl([0, 0, 0]));
            obj.Box_Left_Finger = GripperBoxLeft(transl([0, 0, 0]));
        end
    end
end
