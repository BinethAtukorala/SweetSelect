classdef UR3eClass < handle
    properties
        UR3e % Property to store the robot object model
        Candy_Gripper % Gripper
        Gripper_Status % Currect gripper action (open/close)
        Run_Status % Whether the robot is operational
        eStop_Hold % To confirm whether eStop was disengaged
        Work_Queue % Store movements to be completed.
    end

    methods
        %% Constructor to initialize the robot model
        function obj = UR3eClass(Candy_Gripper)
            % Initialize the robot model UR3e
            % The robot is placed at a translational offset on top of the table
            obj.UR3e  = UR3e(transl([0.94, 1.25, 0.8]));

            % initialize properties
            obj.Run_Status = true;
            obj.eStop_Hold = false;
            obj.Work_Queue = [];

            % Initialize Gripper
            UR3e_EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
            obj.Candy_Gripper = Candy_Gripper;
            obj.Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);
            obj.Gripper_Status = "open";
            obj.Candy_Gripper.openGripper();

        end

        % Estop functionality
        function pressEStop(obj)
            if obj.Run_Status == true
                obj.Run_Status = false;
                obj.eStop_Hold = true;
            elseif obj.eStop_Hold == true
                obj.eStop_Hold = false;
            end
        end

        function resumeOperation(obj)
            if obj.eStop_Hold == false
                obj.Run_Status = true;
            end
        end

        % Check whether robot is busy
        function result = isBusy(obj)
            sz = size(obj.Work_Queue);
            result = sz(1, 1) > 0;
        end

        % Execute work queue
        function executeQueue(obj)      
            % For every work item
            while size(obj.Work_Queue, 1) > 0
                % For every joint angle configuration
                while size(obj.Work_Queue(1).traj, 1) > 0
                    % Confirm run status
                    if obj.Run_Status == false
                        return
                    end

                    % Handle Gripper
                    if obj.Gripper_Status ~= obj.Work_Queue(1).Gripper_Status
                        if obj.Work_Queue(1).Gripper_Status == "close"
                            obj.Gripper_Status = "close";
                            obj.Candy_Gripper.closeGripper();
                        elseif obj.Work_Queue(1).Gripper_Status == "open"
                            obj.Gripper_Status = "open";
                            obj.Candy_Gripper.openGripper();
                        end
                    end
                    
                    % Movement
                    if obj.Work_Queue(1).Gripper_Object
                        obj.animateWithCandy(obj.Work_Queue(1).traj(1, :), obj.Work_Queue(1).Gripper_Object, obj.Work_Queue(1).Gripper_Object_Start_Pose);
                    else
                        obj.animateWithoutCandy(obj.Work_Queue(1).traj(1, :));
                    end
                    obj.Work_Queue(1).traj = obj.Work_Queue(1).traj(2:size(obj.Work_Queue(1).traj, 1), :);
                end
                obj.Work_Queue = obj.Work_Queue(2:size(obj.Work_Queue, 1));
            end
        end

        function [Start_Pose] = moveUR3e(obj,Initial_Candy_Pose, Candy_Final_Poses, Candy, Candy_Index)

                % Calculate start pose for each candy
                Start_Pose = [eye(3), Initial_Candy_Pose'; 0, 0, 0, 1] * trotx(pi);
                Start_Pose(3,4) = Start_Pose(3,4) + 0.09;  % Adjust height
            
                % Determine which final candy pose to use, repeating if i > 3
                % Change the i here to the array index of total candy
                index = mod(Candy_Index-1, size(Candy_Final_Poses, 1)) + 1;
                Candy_Final_Pose = Candy_Final_Poses(index, :);
                Final_Pose = [eye(3), Candy_Final_Pose'; 0, 0, 0, 1] * trotx(pi);
                Final_Waypoint = obj.UR3e.model.ikcon(Final_Pose);
            
                % First_Waypoint = [-68, -11, -34, -45, -90, 36] * pi / 180;
                First_Waypoint = [-58, -39.2, -34, -16.2, -90, 36] * pi / 180;
                Second_Waypoint = [16, -26, -50, -14, -90, 106] * pi / 180;
            
                % Get current robot pose
                UR3e_Pose = obj.UR3e.model.getpos();
            
                % Calculate inverse kinematics to get initial joint angles
                Candy_Waypoint = obj.UR3e.model.ikcon(Start_Pose);
            
                Current_To_First = [UR3e_Pose, First_Waypoint, 30];
                First_To_Initial = [First_Waypoint, Candy_Waypoint, 20];
                Initial_To_First = [Candy_Waypoint, First_Waypoint, 20];
                First_To_Second = [First_Waypoint, Second_Waypoint, 30];
                Second_To_Final = [Second_Waypoint, Final_Waypoint, 20];
                Final_To_Second = [Final_Waypoint, Second_Waypoint, 20];

                obj.Work_Queue = [obj.Work_Queue;
                                    WorkQueueItemClass("open", Current_To_First);          
                                    WorkQueueItemClass("open", First_To_Initial);     
                                    WorkQueueItemClass("close", Initial_To_First, Gripper_Object=Candy, Gripper_Object_Start_Pose=Start_Pose);     
                                    WorkQueueItemClass("close", First_To_Second, Gripper_Object=Candy, Gripper_Object_Start_Pose=Start_Pose);    
                                    WorkQueueItemClass("close", Second_To_Final, Gripper_Object=Candy, Gripper_Object_Start_Pose=Start_Pose);
                                    WorkQueueItemClass("open", Final_To_Second);
                                    ];
       
                % % Animate the robot for each part of the movement
                % obj.moveWithoutCandy(Candy_Gripper, Current_To_First)
                % 
                % obj.moveWithoutCandy(Candy_Gripper, First_To_Initial)
                % 
                % Candy_Gripper.closeGripper();
                % 
                % % move robot towards LBRiiwa
                % obj.moveWithCandy(Candy_Gripper, Initial_To_First, Candy, Start_Pose)
                % 
                % obj.moveWithCandy(Candy_Gripper, First_To_Second, Candy, Start_Pose)
                % 
                % obj.moveWithCandy(Candy_Gripper, Second_To_Final, Candy, Start_Pose)
                % 
                % Candy_Gripper.openGripper();
                % 
                % obj.moveWithoutCandy(Candy_Gripper, Final_To_Second)
        end


        function animateWithCandy(obj, q_angles, Candy, Start_Pose)

            % Animate robot movement
            obj.UR3e.model.animate(q_angles);
          
            % Get current end effector pose
            EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
            
            % Update gripper base to be at the end effector pose
            obj.Candy_Gripper.setGripperBase(EndEffector_Pose);

            % Ensure that gripper arms remain stationary while moving
            obj.Candy_Gripper.stationaryGripper();

            % Calculate the transformation of the brick and update its position
            Candy_Transformation = EndEffector_Pose * inv(Start_Pose);
            Candy.updateCandyPosition(Candy_Transformation);
            
            drawnow();
            pause(0);

        end

        function animateWithoutCandy(obj, q_angles)



            % Animate robot movement
            obj.UR3e.model.animate(q_angles);
          
            % Get current end effector pose
            EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
            
            % Update gripper base to be at the end effector pose
            obj.Candy_Gripper.setGripperBase(EndEffector_Pose);

            % Ensure that gripper arms remain stationary while moving
            obj.Candy_Gripper.stationaryGripper();
            
            drawnow();
            pause(0);


        end
    end
end
