classdef WorkQueueRobotClass
    properties
        Gripper_Status % Desired gripper position
        traj % trajectory of q angles
        Gripper_Has_Object % Whether the gripper has an object or not
        Gripper_Object % Object to be grabbed by gripper
        Gripper_Extra_Objects % Extra objects to be taken along (Candies)
        Gripper_Object_Start_Poses
    end
    
    methods
        function obj = WorkQueueRobotClass(Gripper_Status, traj, Gripper_Has_Object, options)
            arguments
                Gripper_Status string
                traj (:, :) double
                Gripper_Has_Object int16
                options.Gripper_Object = false
                options.Gripper_Object_Start_Poses (:, :) = []
                options.Gripper_Extra_Objects = false
            end

            obj.Gripper_Status = Gripper_Status;
            obj.traj = traj;
            obj.Gripper_Has_Object = Gripper_Has_Object;
            if Gripper_Has_Object > 0
                obj.Gripper_Object = options.Gripper_Object;
                if Gripper_Has_Object == 2
                    obj.Gripper_Extra_Objects = options.Gripper_Extra_Objects;
                end
                obj.Gripper_Object_Start_Poses = options.Gripper_Object_Start_Poses;
            else
                % Throw error
            end
        end

    end
end