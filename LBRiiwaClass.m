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
            Start_Pose = [eye(3), Initial_Box_Pose'; 0, 0, 0, 1]  * troty(-pi/2);

            % Get the necessary robot poses
            LBRiiwa_Pose = LBRiiwa.model.getpos();
            Initial_Joint_Angles = LBRiiwa.model.ikcon(Start_Pose);

            % Waypoints
            First_Waypoint_Joint_Angles = [11, -80, 170, 18, 170, -29, 20] * pi / 180;
            Midway_Pose_Joint_Angles = [170, -80, 170, 18, 170, -29, 20] * pi / 180;
            Second_Waypoint_Joint_Angles = [92, -80, 170, 18, 170, -29, 20] * pi / 180;
            Final_Pose_Joint_Angles = [92, -118, 170, 18, 170, 12, 20] * pi / 180;

            % Calculate the trajectories
            Current_To_Initial = jtraj(LBRiiwa_Pose, Initial_Joint_Angles, 100);
            Initial_To_First = jtraj(Initial_Joint_Angles, First_Waypoint_Joint_Angles, 100);
            First_To_Midway = jtraj(First_Waypoint_Joint_Angles, Midway_Pose_Joint_Angles, 100);
            Midway_To_Second = jtraj(Midway_Pose_Joint_Angles, Second_Waypoint_Joint_Angles, 100);
            Second_To_Final = jtraj(Second_Waypoint_Joint_Angles, Final_Pose_Joint_Angles, 100);       

            % move robot from current pose to pick up box
            obj.moveWithoutBox(obj,LBRiiwa, Box_Gripper, Current_To_Initial)

            Box_Gripper.closeGripper();

            % move robot from intial pose towards UR3
            obj.moveWithBox(obj,LBRiiwa, Box_Gripper, Initial_To_First)

            obj.moveWithBox(obj,LBRiiwa, Box_Gripper, First_To_Midway)

            % move robot from UR3 towards counter
            obj.moveWithBox(obj,LBRiiwa, Box_Gripper, Midway_To_Second)

            obj.moveWithBox(obj,LBRiiwa, Box_Gripper, Second_To_Final)

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
