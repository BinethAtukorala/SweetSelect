classdef LBRiiwaClass
    properties
        LBRiiwa % Property to store the robot object model
    end

    methods
        function obj = LBRiiwaClass(Box_Gripper)
            % Initialize the robot model LBRiiwa
            % The robot is placed at a translational offset on top of the table
            obj.LBRiiwa  = LBRiiwa(transl([-0.3, 1, 0.5]));

            LBRiiwa_EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
            Box_Gripper.setGripperBase(LBRiiwa_EndEffector_Pose);
        end

        function [Start_Pose] = moveFrontToMidway(obj, LBRiiwa, Box_Gripper, Initial_Box_Pose, Box)

            % Make it so that the end effector grips the box from the side
            Start_Pose = [eye(3), Initial_Box_Pose'; 0, 0, 0, 1] * troty(-pi/2);
            Start_Pose(1,4) = Start_Pose(1,4) + 0.19;
            Start_Pose(3,4) = Start_Pose(3,4) + 0.04;

            % Get the necessary robot poses
            LBRiiwa_Pose = obj.LBRiiwa.model.getpos();
            Box_Waypoint = obj.LBRiiwa.model.ikcon(Start_Pose);
            Box_Waypoint(1,7) = Box_Waypoint(1,7) + deg2rad(90);

            % Waypoints for front column of boxes
            Front_First_Waypoint = [0, -80, 180, 18, 180, -28, 90] * pi / 180;
            Front_Midway_Waypoint = [180, -80, 180, 18, 180, -28, 90] * pi / 180;

            % Trajectories for front column of boxes    
            Current_To_First = jtraj(LBRiiwa_Pose, Front_First_Waypoint, 50);
            First_To_Initial = jtraj(Front_First_Waypoint, Box_Waypoint, 50);
            Initial_To_First = jtraj(Box_Waypoint, Front_First_Waypoint, 50);
            First_To_Midway = jtraj(Front_First_Waypoint, Front_Midway_Waypoint, 50);

            % move robot from current pose to pick up box
            obj.moveWithoutBox(LBRiiwa, Box_Gripper, Current_To_First)
           
            obj.moveWithoutBox(LBRiiwa, Box_Gripper, First_To_Initial)

            Box_Gripper.closeGripper();

            % move robot from intial pose towards UR3
            obj.moveWithBox(LBRiiwa, Box_Gripper, Initial_To_First, Box, Start_Pose)

            obj.moveWithBox(LBRiiwa, Box_Gripper, First_To_Midway, Box, Start_Pose)

        end

        function moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Box, Box_Index, Candies,  Box_Start_Pose, Candy_Start_Poses)

            % Waypoints
            Front_Midway_Waypoint = [180, -80, 180, 18, 180, -28, 90] * pi / 180;
            Front_Second_Waypoint = [90, -80, 180, 18, 180, -28, 90] * pi / 180;
            Front_Final_Waypoint = [
                [110, -111, 180, 18, 180, 3.2, 90];
                [100,  -111, 180, 18, 180, 3.2, 90];
            ]* pi / 180;

            % Trajectories for front column of boxes    
            Midway_To_Second = jtraj(Front_Midway_Waypoint, Front_Second_Waypoint, 50);
            Second_To_Final = jtraj(Front_Second_Waypoint, Front_Final_Waypoint(Box_Index,:), 50);
            Final_To_Second = jtraj(Front_Final_Waypoint(Box_Index,:), Front_Second_Waypoint, 50);

            % move robot from UR3 towards counter
            obj.moveWithFilledBox(LBRiiwa, Box_Gripper, Midway_To_Second, Box, Candies, Box_Start_Pose, Candy_Start_Poses)

            obj.moveWithFilledBox(LBRiiwa, Box_Gripper, Second_To_Final, Box, Candies, Box_Start_Pose, Candy_Start_Poses)

            Box_Gripper.openGripper();

            obj.moveWithoutBox(LBRiiwa, Box_Gripper, Final_To_Second)

        end

        function [Start_Pose] = moveBackToMidway(obj, LBRiiwa, Box_Gripper, Initial_Box_Pose, Box)

            % Make it so that the end effector grips the box from the side
            Start_Pose = [eye(3), Initial_Box_Pose'; 0, 0, 0, 1] * troty(-pi/2);
            Start_Pose(1,4) = Start_Pose(1,4) + 0.19;
            Start_Pose(3,4) = Start_Pose(3,4) + 0.1;

            % Get the necessary robot poses
            LBRiiwa_Pose = obj.LBRiiwa.model.getpos();
            Box_Waypoint = obj.LBRiiwa.model.ikcon(Start_Pose);
            Box_Waypoint(1,7) = Box_Waypoint(1,7) + deg2rad(90);

            % Waypoints for back column of boxes
            Back_First_Waypoint = [-30, -80, -180, 15.6, -173, -25.6, 83] * pi / 180;
            Back_Midway_Waypoint = [180, -80, -180, 18, -180, -28, 90] * pi / 180;

            % Trajectories for back column of boxes
            Current_To_First = jtraj(LBRiiwa_Pose, Back_First_Waypoint, 50);
            First_To_Initial = jtraj(Back_First_Waypoint, Box_Waypoint, 50);
            Initial_To_First = jtraj(Box_Waypoint, Back_First_Waypoint, 50);
            First_To_Midway = jtraj(Back_First_Waypoint, Back_Midway_Waypoint, 50);

            % move robot from current pose to pick up box
            obj.moveWithoutBox(LBRiiwa, Box_Gripper, Current_To_First)

            obj.moveWithoutBox(LBRiiwa, Box_Gripper, First_To_Initial)

            Box_Gripper.closeGripper();

            % move robot from intial pose towards UR3
            obj.moveWithBox(LBRiiwa, Box_Gripper, Initial_To_First, Box, Start_Pose)

            obj.moveWithBox(LBRiiwa, Box_Gripper, First_To_Midway, Box, Start_Pose)

        end

        function moveBackFromMidway(obj, LBRiiwa, Box_Gripper, Box, Box_Index, Candies, Box_Start_Pose, Candy_Start_Poses)

            % Waypoints for back column of boxes
            Back_Midway_Waypoint = [180, -80, -180, 18, -180, -28, 90] * pi / 180;
            Back_Second_Waypoint = [90, -80, -180, 18, -180, -28, 90] * pi / 180;
            Back_Final_Waypoint = [
                [80,  -111, -180, 18, -180, 3.2, 90];
                [70,  -111, -180, 18, -180, 3.2, 90];
            ]* pi / 180;

            Box_Index = Box_Index - 2;

            % Trajectories for back column of boxes
            Midway_To_Second = jtraj(Back_Midway_Waypoint, Back_Second_Waypoint, 50);
            Second_To_Final = jtraj(Back_Second_Waypoint, Back_Final_Waypoint(Box_Index,:), 50);
            Final_To_Second = jtraj(Back_Final_Waypoint(Box_Index,:), Back_Second_Waypoint, 50);

            % move robot from UR3 towards counter
            obj.moveWithFilledBox(LBRiiwa, Box_Gripper, Midway_To_Second, Box, Candies, Box_Start_Pose, Candy_Start_Poses)

            obj.moveWithFilledBox(LBRiiwa, Box_Gripper, Second_To_Final, Box, Candies, Box_Start_Pose, Candy_Start_Poses)

            Box_Gripper.openGripper();

            obj.moveWithoutBox(LBRiiwa, Box_Gripper, Final_To_Second)

        end


        function moveWithBox(obj, LBRiiwa, Box_Gripper, Trajectory, Box, Start_Pose)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                obj.LBRiiwa.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
                
                % Update gripper base to be at the end effector pose
                Box_Gripper.setGripperBase(EndEffector_Pose);

                % Ensure that gripper arms remain stationary while moving
                Box_Gripper.stationaryGripper();

                % Calculate the transformation of the box and update its position
                Box_Transformation = EndEffector_Pose * inv(Start_Pose);
                Box.updateBoxPosition(Box_Transformation);
                
                drawnow();
                pause(0);
            end

        end

        function moveWithFilledBox(obj, LBRiiwa, Box_Gripper, Trajectory, Box, Candies, Box_Start_Pose, Candy_Start_Poses)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                obj.LBRiiwa.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
                
                % Update gripper base to be at the end effector pose
                Box_Gripper.setGripperBase(EndEffector_Pose);

                % Ensure that gripper arms remain stationary while moving
                Box_Gripper.stationaryGripper();

                % Calculate the transformation of the box and update its position
                Box_Transformation = EndEffector_Pose * inv(Box_Start_Pose);
                Box.updateBoxPosition(Box_Transformation);

                % Calculate the transformation of the brick and update its position

                % Candy_Transformation = EndEffector_Pose * inv(Candy_Start_Poses(x,:));
                % Candies(x,:).updateCandyPosition(Candy_Transformation);

                for x = 1:3
                    % Determine the starting and ending indices for inv()
                    Start_Index = (x - 1) * 4 + 1;  % 1, 5, 9
                    End_Index = Start_Index + 3;    % 4, 8, 12 (inclusive)
                    
                    % Use the range to extract the corresponding rows for inv()
                    Candy_Transformation = EndEffector_Pose * inv(Candy_Start_Poses(Start_Index:End_Index, :));
                    Candies(x).updateCandyPosition(Candy_Transformation);
                end
                
                drawnow();
                pause(0);
            end

        end

        function moveWithoutBox(obj, LBRiiwa, Box_Gripper, Trajectory)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                obj.LBRiiwa.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
                
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
