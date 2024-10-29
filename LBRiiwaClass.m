classdef LBRiiwaClass < handle
    properties
        LBRiiwa % Property to store the robot object model
        Box_Gripper % Gripper
        Gripper_Status % Currect gripper action (open/close)
        Run_Status % Whether the robot is operational
        eStop_Hold % To confirm whether eStop was disengaged
        Work_Queue % Store movements to be completed.
    end

    methods
        function obj = LBRiiwaClass(Box_Gripper)
            % Initialize the robot model LBRiiwa
            % The robot is placed at a translational offset on top of the table
            obj.LBRiiwa  = LBRiiwa(transl([-0.3, 1, 0.5]));

            % initialize properties
            obj.Run_Status = true;
            obj.eStop_Hold = false;
            obj.Work_Queue = [];
            
            % Initialize Gripper
            LBRiiwa_EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
            obj.Box_Gripper = Box_Gripper;
            obj.Box_Gripper.setGripperBase(LBRiiwa_EndEffector_Pose);
            obj.Gripper_Status = "open";
            obj.Box_Gripper.openGripper();
        end

        % Estop functionality
        function Run_Status = engageEStop(obj)

            obj.Run_Status = false;
            obj.eStop_Hold = true;


            Run_Status = obj.Run_Status();
        end

        function Run_Status = disengageEStop(obj)

            obj.eStop_Hold = false;

            Run_Status = obj.Run_Status();
        end

        function resumeOperation(obj)
            if obj.eStop_Hold == false
                obj.Run_Status = true;
            end
            executeQueue();
        end

        % Check whether robot is busy
        function result = isBusy(obj)
            sz = size(obj.Work_Queue);
            result = sz(1, 1) > 0;
        end

        % Execute work queue
        function executeQueue(obj)    
            disp("Work Queue")
            disp(obj.Work_Queue)
            % For every work item
            while prod(size(obj.Work_Queue)) > 0 
                % For every joint angle configuration
                disp(size(obj.Work_Queue(1).traj))
                while size(obj.Work_Queue(1).traj, 1) > 0
                    % Confirm run status
                    if obj.Run_Status == false
                        return
                    end

                    % Handle Gripper
                    if obj.Gripper_Status ~= obj.Work_Queue(1).Gripper_Status
                        if obj.Work_Queue(1).Gripper_Status == "close"
                            obj.Gripper_Status = "close";
                            obj.Box_Gripper.closeGripper();
                        elseif obj.Work_Queue(1).Gripper_Status == "open"
                            obj.Gripper_Status = "open";
                            obj.Box_Gripper.openGripper();
                        end
                    end
                    
                    % Movement
                    if obj.Work_Queue(1).Gripper_Has_Object == 2
                        
                        obj.moveWithFilledBox(...
                            obj.Work_Queue(1).traj(1, :),...
                            obj.Work_Queue(1).Gripper_Object, ...
                            obj.Work_Queue(1).Gripper_Extra_Objects,...
                            obj.Work_Queue(1).Gripper_Object_Start_Poses(1:4, :),...
                            obj.Work_Queue(1).Gripper_Object_Start_Poses(5:size(obj.Work_Queue(1).Gripper_Object_Start_Poses, 1), :)...
                            );
                    elseif obj.Work_Queue(1).Gripper_Has_Object == 1

                        obj.moveWithBox( ...
                            obj.Work_Queue(1).traj(1, :), ...
                            obj.Work_Queue(1).Gripper_Object, ...
                            obj.Work_Queue(1).Gripper_Object_Start_Poses(1:4, :) ...
                            );

                    else
                        obj.moveWithoutBox(obj.Work_Queue(1).traj(1, :))
                    end
                    obj.Work_Queue(1).traj = obj.Work_Queue(1).traj(2:size(obj.Work_Queue(1).traj, 1), :);
                end
                obj.Work_Queue = obj.Work_Queue(2:size(obj.Work_Queue, 1));
            end
        end

        function [Start_Pose] = moveFrontToMidway(obj, Initial_Box_Pose, Box)

            % Make it so that the end effector grips the box from the side
            Start_Pose = [eye(3), Initial_Box_Pose'; 0, 0, 0, 1] * troty(-pi/2);
            Start_Pose(1,4) = Start_Pose(1,4) + 0.18;
            Start_Pose(3,4) = Start_Pose(3,4) + 0.05;

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
            % obj.moveWithoutBox(Box_Gripper, Current_To_First);
            % 
            % obj.moveWithoutBox(Box_Gripper, First_To_Initial);

            obj.Work_Queue = [obj.Work_Queue; ...
                                WorkQueueItemClass("open", Current_To_First, false);
                                WorkQueueItemClass("open", First_To_Initial, false);
                            ];

            % obj.Box_Gripper.closeGripper();

            % move robot from intial pose towards UR3
            % obj.moveWithBox(Box_Gripper, Initial_To_First, Box, Start_Pose);
            % 
            % obj.moveWithBox(Box_Gripper, First_To_Midway, Box, Start_Pose);

            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueItemClass("close", Initial_To_First, true, Gripper_Object=Box, Gripper_Object_Start_Poses=[Start_Pose]);
                                WorkQueueItemClass("close", First_To_Midway, true, Gripper_Object=Box, Gripper_Object_Start_Poses=[Start_Pose]);
                            ];

            obj.executeQueue();

        end

        function moveFrontFromMidway(obj, Box, Box_Index, Candies,  Box_Start_Pose, Candy_Start_Poses)

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
            % obj.moveWithFilledBox(Box_Gripper, Midway_To_Second, Box, Candies, Box_Start_Pose, Candy_Start_Poses)
            % 
            % obj.moveWithFilledBox(Box_Gripper, Second_To_Final, Box, Candies, Box_Start_Pose, Candy_Start_Poses)
            % 
            % obj.Box_Gripper.openGripper();
            % 
            % obj.moveWithoutBox(Box_Gripper, Final_To_Second)

            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueItemClass("close", Midway_To_Second, 2, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueItemClass("close", Second_To_Final, 2, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueItemClass("open", Final_To_Second, false);
                            ];

            obj.executeQueue();

        end

        function [Start_Pose] = moveBackToMidway(obj, LBRiiwa, Initial_Box_Pose, Box)

            % Make it so that the end effector grips the box from the side
            Start_Pose = [eye(3), Initial_Box_Pose'; 0, 0, 0, 1] * troty(-pi/2);
            Start_Pose(1,4) = Start_Pose(1,4) + 0.18;
            Start_Pose(3,4) = Start_Pose(3,4) + 0.04;

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
            % obj.moveWithoutBox(LBRiiwa, Box_Gripper, Current_To_First)
            % 
            % obj.moveWithoutBox(LBRiiwa, Box_Gripper, First_To_Initial)
            % 
            % obj.Box_Gripper.closeGripper();
            % 
            % % move robot from intial pose towards UR3
            % obj.moveWithBox(LBRiiwa, Box_Gripper, Initial_To_First, Box, Start_Pose)
            % 
            % obj.moveWithBox(LBRiiwa, Box_Gripper, First_To_Midway, Box, Start_Pose)

            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueItemClass("open", Current_To_First, false);
                                WorkQueueItemClass("open", First_To_Initial, false);
                                WorkQueueItemClass("close", Initial_To_First, true, Gripper_Object=Box, Gripper_Object_Start_Poses=[Start_Pose]);
                            ];

            obj.executeQueue();

        end

        function moveBackFromMidway(obj, Box, Box_Index, Candies, Box_Start_Pose, Candy_Start_Poses)

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
            % obj.moveWithFilledBox(Box_Gripper, Midway_To_Second, Box, Candies, Box_Start_Pose, Candy_Start_Poses)
            % 
            % obj.moveWithFilledBox(Box_Gripper, Second_To_Final, Box, Candies, Box_Start_Pose, Candy_Start_Poses)
            % 
            % obj.Box_Gripper.openGripper();
            % 
            % obj.moveWithoutBox(Box_Gripper, Final_To_Second)

            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueItemClass("close", Midway_To_Second, true, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueItemClass("close", Second_To_Final, true, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueItemClass("open", Final_To_Second, false);
                            ];

            obj.executeQueue();

        end


        function moveWithBox(obj, Trajectory, Box, Start_Pose)


            % Animate robot movement
            obj.LBRiiwa.model.animate(Trajectory);
          
            % Get current end effector pose
            EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
            
            % Update gripper base to be at the end effector pose
            obj.Box_Gripper.setGripperBase(EndEffector_Pose);

            % Ensure that gripper arms remain stationary while moving
            obj.Box_Gripper.stationaryGripper();

            % Calculate the transformation of the box and update its position
            Box_Transformation = EndEffector_Pose * inv(Start_Pose);
            Box.updateBoxPosition(Box_Transformation);
            
            drawnow();
            pause(0);


        end

        function moveWithFilledBox(obj, Trajectory, Box, Candies, Box_Start_Pose, Candy_Start_Poses)


            % Animate robot movement
            obj.LBRiiwa.model.animate(Trajectory);
          
            % Get current end effector pose
            EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
            
            % Update gripper base to be at the end effector pose
            obj.Box_Gripper.setGripperBase(EndEffector_Pose);

            % Ensure that gripper arms remain stationary while moving
            obj.Box_Gripper.stationaryGripper();

            % Calculate the transformation of the box and update its position
            Box_Transformation = EndEffector_Pose * inv(Box_Start_Pose);
            Box.updateBoxPosition(Box_Transformation);

            % Calculate the transformation of the brick and update its position
            for x = 1:size(Candies, 2)
                % Determine the starting and ending indices for inv()
                Start_Index = (x - 1) * 4 + 1;  % 1, 5, 9
                End_Index = Start_Index + 3;    % 4, 8, 12 (inclusive)

                % Create a copy of the Candy_Start_Poses to apply offsets
                Adjusted_Candy_Start_Poses = Candy_Start_Poses(Start_Index:End_Index, :);
            
                % Apply specific offsets based on the candy index
                if x == 1
                    Adjusted_Candy_Start_Poses(3, 4) = Adjusted_Candy_Start_Poses(3, 4) + 0.065;  % Offset for the first candy
                    Adjusted_Candy_Start_Poses(2, 4) = Adjusted_Candy_Start_Poses(2, 4) - 0.022;  % Offset for the first candy
                elseif x == 2
                    Adjusted_Candy_Start_Poses(3, 4) = Adjusted_Candy_Start_Poses(3, 4) + 0.125;  % Offset for the second candy
                    Adjusted_Candy_Start_Poses(2, 4) = Adjusted_Candy_Start_Poses(2, 4) - 0.022;  % Offset for the first candy
                elseif x == 3
                    Adjusted_Candy_Start_Poses(3, 4) = Adjusted_Candy_Start_Poses(3, 4) + 0.185;  % Offset for the third candy
                    Adjusted_Candy_Start_Poses(2, 4) = Adjusted_Candy_Start_Poses(2, 4) - 0.022;  % Offset for the first candy
                end
            
                % Use the adjusted starting poses to calculate the Candy_Transformation
                Candy_Transformation = EndEffector_Pose * inv(Adjusted_Candy_Start_Poses);
            
                % Update the candy position using the transformed matrix
                Candies(x).updateCandyPosition(Candy_Transformation);
            end

            drawnow();
            pause(0);

        end

        function moveWithoutBox(obj, Trajectory)


            % Animate robot movement
            obj.LBRiiwa.model.animate(Trajectory);
          
            % Get current end effector pose
            EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
            
            % Update gripper base to be at the end effector pose
            obj.Box_Gripper.setGripperBase(EndEffector_Pose);

            % Ensure that gripper arms remain stationary while moving
            obj.Box_Gripper.stationaryGripper();
            
            drawnow();
            pause(0);


        end
    end
end
