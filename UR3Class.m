classdef UR3Class
    properties
        Robot_UR3 % Property to store the robot object model
    end

    methods
        %% Constructor to initialize the robot model
        function obj = UR3Class()
            % Initialize the robot model UR3
            % The robot is placed at a translational offset on top of the table
            obj.Robot_UR3  = UR3(transl([0.8, 1, 0.6]));
        end

        function moveRaspberryUR3(obj, UR3, Candy_Gripper, Initial_Candy_Pose)
            % Calculate start pose for each candy
            Candy_Start_Pose = [eye(3), Initial_Candy_Pose'; 0, 0, 0, 1] * trotx(pi);
            Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height
        
            Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
            Final_Waypoint = [-161, -24, -360, -65.6, 90, -71] * pi / 180;
            
            % Get current robot pose
            UR3_Pose = UR3.model.getpos();
            
            % Calculate inverse kinematics to get initial joint angles
            Candy_Waypoint = UR3.model.ikcon(Candy_Start_Pose);
        
            % Calculate trajectories
            Current_To_Midway = jtraj(UR3_Pose, Midway_Waypoint, 30);
            Midway_To_Initial = jtraj(Midway_Waypoint, Candy_Waypoint, 30);
            Initial_To_Midway = jtraj(Candy_Waypoint, Midway_Waypoint, 30);
            Midway_To_Final = jtraj(Midway_Waypoint, Final_Waypoint, 30);
            
            % Animate the robot for each part of the movement
            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Current_To_Midway)

            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Midway_To_Initial)

            Candy_Gripper.closeGripper();

            % move robot towards LBRiiwa
            obj.moveWithCandy(obj,UR3, Candy_Gripper, Initial_To_Midway)

            obj.moveWithCandy(obj,UR3, Candy_Gripper, Midway_To_Final)

            Candy_Gripper.openGripper();
        end

        function moveBlueberryUR3(obj, UR3, Candy_Gripper, Initial_Candy_Pose)
            % Calculate start pose for each candy
            Candy_Start_Pose = [eye(3), Initial_Candy_Pose'; 0, 0, 0, 1] * trotx(pi);
            Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height
        
            Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
            Final_Waypoint = [-161, -24, -360, -65.6, 90, -71] * pi / 180;
            
            % Get current robot pose
            UR3_Pose = UR3.model.getpos();
            
            % Calculate inverse kinematics to get initial joint angles
            Candy_Waypoint = UR3.model.ikcon(Candy_Start_Pose);
        
            % Calculate trajectories
            Current_To_Midway = jtraj(UR3_Pose, Midway_Waypoint, 30);
            Midway_To_Initial = jtraj(Midway_Waypoint, Candy_Waypoint, 30);
            Initial_To_Midway = jtraj(Candy_Waypoint, Midway_Waypoint, 30);
            Midway_To_Final = jtraj(Midway_Waypoint, Final_Waypoint, 30);
            
            % Animate the robot for each part of the movement
            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Current_To_Midway)

            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Midway_To_Initial)

            Candy_Gripper.closeGripper();

            % move robot towards LBRiiwa
            obj.moveWithCandy(obj,UR3, Candy_Gripper, Initial_To_Midway)

            obj.moveWithCandy(obj,UR3, Candy_Gripper, Midway_To_Final)

            Candy_Gripper.openGripper();
        end

        function moveGreenappleUR3(obj, UR3, Candy_Gripper, Initial_Candy_Pose)
            % Calculate start pose for each candy
            Candy_Start_Pose = [eye(3), Initial_Candy_Pose'; 0, 0, 0, 1] * trotx(pi);
            Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height
        
            Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
            Final_Waypoint = [-161, -24, -360, -65.6, 90, -71] * pi / 180;
            
            % Get current robot pose
            UR3_Pose = UR3.model.getpos();
            
            % Calculate inverse kinematics to get initial joint angles
            Candy_Waypoint = UR3.model.ikcon(Candy_Start_Pose);
        
            % Calculate trajectories
            Current_To_Midway = jtraj(UR3_Pose, Midway_Waypoint, 30);
            Midway_To_Initial = jtraj(Midway_Waypoint, Candy_Waypoint, 30);
            Initial_To_Midway = jtraj(Candy_Waypoint, Midway_Waypoint, 30);
            Midway_To_Final = jtraj(Midway_Waypoint, Final_Waypoint, 30);
            
            % Animate the robot for each part of the movement
            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Current_To_Midway)

            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Midway_To_Initial)

            Candy_Gripper.closeGripper();

            % move robot towards LBRiiwa
            obj.moveWithCandy(obj,UR3, Candy_Gripper, Initial_To_Midway)

            obj.moveWithCandy(obj,UR3, Candy_Gripper, Midway_To_Final)

            Candy_Gripper.openGripper();
        end

        function moveUR3(obj, UR3, Candy_Gripper, Initial_Candy_Pose)

            % Make it so that the end effector grips the Candy from the side
            Start_Pose = [eye(3), Initial_Candy_Pose'; 0, 0, 0, 1] * trotx(pi);
            Final_Pose = % Edit the final pose where Candy is dropped onto box

            % Get the necessary robot poses
            UR3_Pose = UR3.model.getpos();
            Initial_Joint_Angles = UR3.model.ikcon(Start_Pose);
            Final_Joint_Angles = UR3.model.ikcon(Final_Pose);

            % Calculate the trajectories
            Current_To_Initial = jtraj(UR3_Pose, Initial_Joint_Angles, 30);
            Initial_To_Final = jtraj(Initial_Joint_Angles, Final_Joint_Angles, 30);        

            % move robot from current pose to pick up Candy
            obj.moveWithoutCandy(obj,UR3, Candy_Gripper, Current_To_Initial)

            Candy_Gripper.closeGripper();

            % move robot towards LBRiiwa
            obj.moveWithCandy(obj,UR3, Candy_Gripper, Initial_To_Final)

            Candy_Gripper.openGripper();
        end


        function moveWithCandy(obj, UR3, Candy_Gripper, Trajectory, Candy, Start_Pose)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                UR3.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = UR3.model.fkine(UR3.model.getpos()).T;
                
                % Update gripper base to be at the end effector pose
                Candy_Gripper.setGripperBase(EndEffector_Pose);

                % Ensure that gripper arms remain stationary while moving
                Candy_Gripper.stationaryGripper();

                % Calculate the transformation of the brick and update its position
                Candy_Transformation = EndEffector_Pose * inv(Start_Pose);
                Candy.updateCandyPosition(Candy_Transformation);
                
                drawnow();
                pause(0);
            end

        end

        function moveWithoutCandy(obj, UR3, Candy_Gripper, Trajectory)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                UR3.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = UR3.model.fkine(UR3.model.getpos()).T;
                
                % Update gripper base to be at the end effector pose
                Candy_Gripper.setGripperBase(EndEffector_Pose);

                % Ensure that gripper arms remain stationary while moving
                Candy_Gripper.stationaryGripper();
                
                drawnow();
                pause(0);
            end

        end
    end
end
