classdef LBRiiwaClass
    properties
        Robot_LBRiiwa % Property to store the robot object model
    end

    methods
        function obj = LBRiiwaClass()
            % Initialize the robot model LBRiiwa
            % The robot is placed at a translational offset on top of the table
            obj.Robot_LBRiiwa  = LBRiiwa(transl([0, 0, 0.5]));
        end

        function moveLBRiiwa(obj, LBRiiwa, Box_Gripper, Initial_Box_Pose)

            % Make it so that the end effector grips the box from the side
            Start_Pose = [eye(3), Initial_Box_Pose'; 0, 0, 0, 1] * trotx(pi/2);
            Midway_Pose = % Edit the midway pose where LBRiiwa holds the box here
            Final_Pose = % Edit the final pose where LBRiiwa keeps the box on the counter

            % Get the necessary robot poses
            LBRiiwa_Pose = LBRiiwa.model.getpos();
            Initial_Joint_Angles = LBRiiwa.model.ikcon(Start_Pose);
            Midway_Joint_Angles = LBRiiwa.model.ikcon(Midway_Pose);
            Final_Joint_Angles = LBRiiwa.model.ikcon(Final_Pose);

            % Calculate the trajectories
            Current_To_Initial = jtraj(LBRiiwa_Pose, Initial_Joint_Angles, 30);
            Initial_To_Midway = jtraj(Initial_Joint_Angles, Midway_Joint_Angles, 30);
            Midway_To_Final = jtraj(Midway_Joint_Angles, Final_Joint_Angles, 30);        

            % move robot from current pose to pick up box
            obj.moveWithoutBox(obj,LBRiiwa, Box_Gripper, Current_To_Initial)

            Box_Gripper.closeGripper();

            % move robot from intial pose towards UR3
            obj.moveWithBox(obj,LBRiiwa, Box_Gripper, Initial_To_Midway)

            % move robot from UR3 towards counter
            obj.moveWithBox(obj,LBRiiwa, Box_Gripper, Midway_To_Final)

            Box_Gripper.openGripper();
        end

        function moveWithBox(obj, LBRiiwa, Box_Gripper, Trajectory)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                LBRiiwa.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T;
                
                % Update gripper base to be at the end effector pose
                Box_Gripper.setGripperBase(EndEffector_Pose);

                % Ensure that gripper arms remain stationary while moving
                Box_Gripper.stationaryGripper();

                % Calculate the transformation of the brick and update its position
                Box_Transformation = EndEffector_Pose * inv(Start_Pose);
                Box.updateBoxPosition(Box_Transformation);
                
                drawnow();
                pause(0);
            end

        end

        function moveWithoutBox(obj, LBRiiwa, Box_Gripper, Trajectory)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                LBRiiwa.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T;
                
                % Update gripper base to be at the end effector pose
                Box_Gripper.setGripperBase(EndEffector_Pose);

                % Ensure that gripper arms remain stationary while moving
                Box_Gripper.stationaryGripper();
                
                drawnow();
                pause(0);
            end

        end
    end
end
