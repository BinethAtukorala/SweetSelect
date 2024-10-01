classdef UR3Class
    properties
        Robot_UR3 % Property to store the robot object model
    end

    methods
        %% Constructor to initialize the robot model
        function obj = UR3Class()
            % Initialize the robot model UR3
            % The robot is placed at a translational offset on top of the table
            obj.Robot_UR3  = UR3(transl([1, 0, 0.5]));
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
