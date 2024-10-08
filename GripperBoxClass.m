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

        % function animateGripper(obj, Right_Trajectory, Left_Trajectory)
        %     for i = 1:size(Right_Trajectory, 1)
        %         % Animate right finger
        %         obj.Box_Right_Finger.model.animate(Right_Trajectory(i, :));
        %         % Animate left finger
        %         obj.Box_Left_Finger.model.animate(Left_Trajectory(i, :));
        %         pause(0);
        %     end
        % end
        

        function openGripper(obj)
            % Trajectories for fingers to move towards each other
            Right_Trajectory = jtraj(obj.Box_Right_Finger.model.getpos(), [0, -pi/4], 50);
            Left_Trajectory = jtraj(obj.Box_Left_Finger.model.getpos(), [0, pi/4], 50);
            for i = 1:size(Right_Trajectory, 1)
                % Animate right finger
                obj.Box_Right_Finger.model.animate(Right_Trajectory(i, :));
                % Animate left finger
                obj.Box_Left_Finger.model.animate(Left_Trajectory(i, :));
                pause(0);
            end
        end
        

        function closeGripper(obj)
            % Trajectories for fingers to move away from each other
            Right_Trajectory = jtraj(obj.Box_Right_Finger.model.getpos(), [0, -pi/16], 50);
            Left_Trajectory = jtraj(obj.Box_Left_Finger.model.getpos(), [0, pi/16], 50);
            for i = 1:size(Right_Trajectory, 1)
                % Animate right finger
                obj.Box_Right_Finger.model.animate(Right_Trajectory(i, :));
                % Animate left finger
                obj.Box_Left_Finger.model.animate(Left_Trajectory(i, :));
                pause(0);
            end
        end


        function stationaryGripper(obj)

            Right_Trajectory = jtraj(obj.Box_Right_Finger.model.getpos(), obj.Box_Right_Finger.model.getpos(), 2);
            Left_Trajectory = jtraj(obj.Box_Left_Finger.model.getpos(), obj.Box_Left_Finger.model.getpos(), 2);
            
            % Animate both fingers at their current positions
            obj.Box_Right_Finger.model.animate(Right_Trajectory(1,:));
            obj.Box_Left_Finger.model.animate(Left_Trajectory(1,:));

        end
        
        function setGripperBase(obj, EndEffector_Pose)
            obj.Box_Right_Finger.model.base = EndEffector_Pose;
            obj.Box_Left_Finger.model.base = EndEffector_Pose;
        end
    end
end
