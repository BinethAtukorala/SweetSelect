classdef UR3eClass
    properties
        Robot_UR3e % Property to store the robot object model
    end

    methods
        %% Constructor to initialize the robot model
        function obj = UR3eClass()
            % Initialize the robot model UR3e
            % The robot is placed at a translational offset on top of the table
            obj.Robot_UR3e  = UR3e(transl([0.8, 1, 0.6]));
        end

        function moveUR3e(obj, UR3e, Candy_Gripper, Initial_Candy_Pose, Candy_Final_Poses)

                % Calculate start pose for each candy
                Start_Pose = [eye(3), Initial_Candy_Pose'; 0, 0, 0, 1] * trotx(pi);
                Start_Pose(3,4) = Start_Pose(3,4) + 0.1;  % Adjust height
            
                % Determine which final candy pose to use, repeating if i > 3
                % Change the i here to the array index of total candy
                index = mod(i-1, size(Candy_Final_Poses, 1)) + 1;
                Candy_Final_Pose = Candy_Final_Poses(index, :);
                Final_Pose = [eye(3), Candy_Final_Pose'; 0, 0, 0, 1] * trotx(pi);
                Final_Waypoint = UR3e.model.ikcon(Final_Pose);
            
                First_Waypoint = [-68, -11, -34, -45, -90, 36] * pi / 180;
                Second_Waypoint = [16, -26, -50, -14, -90, 106] * pi / 180;
            
                % Get current robot pose
                UR3e_Pose = UR3e.model.getpos();
            
                % Calculate inverse kinematics to get initial joint angles
                Candy_Waypoint = UR3e.model.ikcon(Start_Pose);
            
                % Display joint angles
                disp('Candy Initial Joint Angles:');
                disp(rad2deg(Candy_Initial_Joint_Angles));
            
                Current_To_First = jtraj(UR3e_Pose, First_Waypoint, 30);
                First_To_Initial = jtraj(First_Waypoint, Candy_Waypoint, 30);
                Initial_To_First = jtraj(Candy_Waypoint, First_Waypoint, 30);
                First_To_Second = jtraj(First_Waypoint, Second_Waypoint, 30);
                Second_To_Final = jtraj(Second_Waypoint, Final_Waypoint, 30);
                Final_To_Second = jtraj(Final_Waypoint, Second_Waypoint, 30);
       
                % Animate the robot for each part of the movement
                obj.moveWithoutCandy(obj,UR3e, Candy_Gripper, Current_To_First)
    
                obj.moveWithoutCandy(obj,UR3e, Candy_Gripper, First_To_Initial)
    
                Candy_Gripper.closeGripper();
    
                % move robot towards LBRiiwa
                obj.moveWithCandy(obj,UR3e, Candy_Gripper, Initial_To_First, Candy, Start_Pose)
    
                obj.moveWithCandy(obj,UR3e, Candy_Gripper, First_To_Second, Candy, Start_Pose)

                obj.moveWithCandy(obj,UR3e, Candy_Gripper, Second_To_Final, Candy, Start_Pose)
    
                Candy_Gripper.openGripper();

                obj.moveWithoutCandy(obj,UR3e, Candy_Gripper, Final_To_Second)
        end


        function moveWithCandy(obj, UR3e, Candy_Gripper, Trajectory, Candy, Start_Pose)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                UR3e.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
                
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

        function moveWithoutCandy(obj, UR3e, Candy_Gripper, Trajectory)

            for i = 1:size(Trajectory, 1)
                % Animate robot movement
                UR3e.model.animate(Trajectory(i,:));
              
                % Get current end effector pose
                EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
                
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
