classdef WorkQueueItemClass
    properties
        Gripper_Status % Desired gripper position
        traj % trajectory of q angles
        Gripper_Object % Object to be grabbed by gripper
        Gripper_Object_Start_Pose
    end
    
    methods
        function obj = WorkQueueItemClass(Gripper_Status, traj, options)
            arguments
                Gripper_Status string
                traj (1, :) double
                options.Gripper_Object = false
                options.Gripper_Object_Start_Pose = false
            end

            obj.Gripper_Status = Gripper_Status;
            obj.traj = traj;
            obj.Gripper_Object = options.Gripper_Object;
            obj.Gripper_Object_Start_Pose = options.Gripper_Object_Start_Pose;
        end

    end
end