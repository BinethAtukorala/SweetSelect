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
        end

        % Check whether robot is busy
        function result = isBusy(obj)
            sz = size(obj.Work_Queue);
            result = sz(1, 1) > 0;
        end

        % Execute work queue
        function executeQueue(obj)    

            % For every work item
            while prod(size(obj.Work_Queue)) > 0 

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
                            obj.Box_Gripper.closeGripper();
                        elseif obj.Work_Queue(1).Gripper_Status == "open"
                            obj.Gripper_Status = "open";
                            obj.Box_Gripper.openGripper();
                        end
                    end
                    
                    % Movement
                    if obj.Work_Queue(1).Gripper_Has_Object == 2   % More than two objects moved by gripper (Box and Candies)
                        
                        obj.moveWithFilledBox(...
                            obj.Work_Queue(1).traj(1, :),...
                            obj.Work_Queue(1).Gripper_Object, ...
                            obj.Work_Queue(1).Gripper_Extra_Objects,...
                            obj.Work_Queue(1).Gripper_Object_Start_Poses(1:4, :),...
                            obj.Work_Queue(1).Gripper_Object_Start_Poses(5:size(obj.Work_Queue(1).Gripper_Object_Start_Poses, 1), :)...
                            );
                    elseif obj.Work_Queue(1).Gripper_Has_Object == 1   % Only one object moved by gripper (Box only)

                        obj.moveWithBox( ...
                            obj.Work_Queue(1).traj(1, :), ...
                            obj.Work_Queue(1).Gripper_Object, ...
                            obj.Work_Queue(1).Gripper_Object_Start_Poses(1:4, :) ...
                            );

                    else   % No object moved by gripper
                        obj.moveWithoutBox(obj.Work_Queue(1).traj(1, :))
                    end

                    % Delete trajectory performed
                    obj.Work_Queue(1).traj = obj.Work_Queue(1).traj(2:size(obj.Work_Queue(1).traj, 1), :);
                end

                % Delete work item performed
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


            obj.Work_Queue = [obj.Work_Queue; ...
                                WorkQueueRobotClass("open", Current_To_First, false);
                                WorkQueueRobotClass("open", First_To_Initial, false);
                            ];


            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueRobotClass("close", Initial_To_First, true, Gripper_Object=Box, Gripper_Object_Start_Poses=[Start_Pose]);
                                WorkQueueRobotClass("close", First_To_Midway, true, Gripper_Object=Box, Gripper_Object_Start_Poses=[Start_Pose]);
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
                [80, -111, 180, 18, 180, 3.2, 90];
                [70, -111, 180, 18, 180, 3.2, 90];
            ]* pi / 180;

            % Trajectories for front column of boxes    
            Midway_To_Second = jtraj(Front_Midway_Waypoint, Front_Second_Waypoint, 50);
            Second_To_Final = jtraj(Front_Second_Waypoint, Front_Final_Waypoint(Box_Index,:), 50);
            Final_To_Second = jtraj(Front_Final_Waypoint(Box_Index,:), Front_Second_Waypoint, 50);



            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueRobotClass("close", Midway_To_Second, 2, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueRobotClass("close", Second_To_Final, 2, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueRobotClass("open", Final_To_Second, false);
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

            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueRobotClass("open", Current_To_First, false);
                                WorkQueueRobotClass("open", First_To_Initial, false);
                                WorkQueueRobotClass("close", Initial_To_First, true, Gripper_Object=Box, Gripper_Object_Start_Poses=[Start_Pose]);
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



            obj.Work_Queue = [obj.Work_Queue;
                                WorkQueueRobotClass("close", Midway_To_Second, true, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueRobotClass("close", Second_To_Final, true, Gripper_Object=Box, Gripper_Extra_Objects=Candies, Gripper_Object_Start_Poses=[Box_Start_Pose; Candy_Start_Poses]);
                                WorkQueueRobotClass("open", Final_To_Second, false);
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

        function moveWithTeach(obj,Q)
            % Check for collision
            [Box_Faces, Box_Vertices, ~] = plyread('toolbox.ply', 'tri');
            Box_Position = [-0.3, 1.7, 0.5];% Position of the box
            
            % Get transformed vertices of the box
            Box_Vertices_Transformed = [Box_Vertices, ones(size(Box_Vertices, 1), 1)] * transl(Box_Position)';
            Box_Vertices_Transformed = Box_Vertices_Transformed(:, 1:3);  % Get transformed vertices
            
            % Calculate the distance from the box origin to its furthest vertex
            Box_Origin = Box_Position;
            Distances = sqrt(sum((Box_Vertices_Transformed - Box_Origin).^2, 2));
            Max_Distance = max(Distances);  % Maximum distance from origin to a vertex
            
            % Define sphere properties
            Sphere_Center = Box_Origin;  % Sphere center is the box origin
            Radius = Max_Distance + 0.2;  % Sphere radius is max distance + 0.1

            % Create sphere
            [X, Y, Z] = sphere(20);
            X = X * Radius + Sphere_Center(1);
            Y = Y * Radius + Sphere_Center(2);
            Z = Z * Radius + Sphere_Center(3);
            
            % Plot the sphere as a point cloud or triangle mesh
            Make_Triangle_Mesh = true;  % Keep mesh plot for collision detection only
            
            % Create the sphere mesh but make it invisible
            if Make_Triangle_Mesh
                Tri = delaunay(X, Y, Z);
                % Create the mesh and set properties to make it completely invisible
                SphereTri_h = trimesh(Tri, X, Y, Z, 'FaceColor', 'none', 'EdgeColor', 'none'); % Invisible sphere
            end

            Tr = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos).T;
            EndEffector_To_Center_Dist = sqrt(sum((Sphere_Center - Tr(1:3, 4)').^2));

            if EndEffector_To_Center_Dist <= Radius
                error('Robot movement stopped due to collision.');
                % break;  % Stop the robot movement if a collision is detected
            else

                obj.moveWithoutBox(Q);
            end
        end

        function moveWithGivenPose(obj, Final_X, Final_Y, Final_Z)
            % Check for collision
            [Box_Faces, Box_Vertices, ~] = plyread('toolbox.ply', 'tri');
            Box_Position = [-0.3, 1.6, 0.5]; % Position of the box
            
            

            
            % Get transformed vertices of the box
            Box_Vertices_Transformed = [Box_Vertices, ones(size(Box_Vertices, 1), 1)] * transl(Box_Position)';
            Box_Vertices_Transformed = Box_Vertices_Transformed(:, 1:3); % Get transformed vertices
            
            % Calculate the distance from the box origin to its furthest vertex
            Box_Origin = Box_Position;
            Distances = sqrt(sum((Box_Vertices_Transformed - Box_Origin).^2, 2));
            Max_Distance = max(Distances); % Maximum distance from origin to a vertex
            
            % Define sphere properties
            Sphere_Center = Box_Origin; % Sphere center is the box origin
            Radius = Max_Distance + 0.2; % Sphere radius is max distance + 0.1

            LBRiiwa_EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
            obj.Box_Gripper.setGripperBase(LBRiiwa_EndEffector_Pose);
            
            % Animate robot in joint angles, far from singularity and high manupability
            Initial_Joint_Angles = deg2rad([0, 0, 0, -90, 0, 0, 0]);
            obj.LBRiiwa.model.animate(Initial_Joint_Angles); 
            
            % Define translation of the target pose
            Translation_Target = transl(Final_X, Final_Y, Final_Z);
            
            % Combine rotation and translation to form the target pose
            Target_Pose = Translation_Target * trotx(pi); % The final transformation matrix
            
            % Inverse kinematics to get target joint configuration
            Q_Target = obj.LBRiiwa.model.ikcon(Target_Pose); % Using the rotated target pose
            
            % Initial joint configuration
            Q_Start = obj.LBRiiwa.model.getpos(); % Get current joint angles
            Steps = 50; % Number of steps for the trajectory
            Delta_T = 0.05; % Discrete time step
            
            % Mask for considering x, y, z motion only
            M = [1 1 1 zeros(1, 3)]; 
            
            % Get current pose of LBRiiwa
            LBRiiwa_Pose = obj.LBRiiwa.model.fkine(Q_Start).T;
            
            % Define start and end positions
            X1 = LBRiiwa_Pose(1:3, 4); % Current end-effector position
            X2 = Target_Pose(1:3, 4); % Desired end-effector position (from target pose)
            
            % Create trajectory using RMRC
            X = zeros(3, Steps);
            S = lspb(0, 1, Steps); % Interpolation scalar using trapezoidal velocity profile
            for I = 1:Steps
                X(:, I) = X1 * (1 - S(I)) + S(I) * X2;
            end
            
            % Initialize joint angles matrix
            Q_Matrix = nan(Steps, 7);
            Q_Matrix(1, :) = Q_Start; % Initial joint configuration
            
            % Damped least squares parameter (Lambda)
            Lambda = 0.1; % Damping factor (adjust based on stability requirements)
            
            % Loop for RMRC trajectory execution
            for I = 1:Steps-1
                % Compute velocity (Xdot) for x, y, and z
                Xdot = (X(:, I+1) - X(:, I)) / Delta_T;                       
                
                % Get the full Jacobian for the current joint configuration
                J = obj.LBRiiwa.model.jacob0(Q_Matrix(I, :));
                J_Pos = J(1:3, :); % Consider only the first 3 rows for x, y, z motion
                
                % Damped Least Squares inverse of the Jacobian
                J_Damped = J_Pos' * inv(J_Pos * J_Pos' + Lambda^2 * eye(3)); 
                
                % Compute joint velocities using DLS
                Qdot = J_Damped * Xdot;  
                
                % Update joint angles for the next step
                Q_Matrix(I+1, :) = Q_Matrix(I, :) + Delta_T * Qdot';
                
                % Animate the robot
                obj.LBRiiwa.model.animate(Q_Matrix(I+1, :));
            
                LBRiiwa_EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
                obj.Box_Gripper.setGripperBase(LBRiiwa_EndEffector_Pose);
            
                obj.Box_Gripper.stationaryGripper();
                pause(0.1); % Pause for smooth animation

                Tr = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos).T;
                EndEffector_To_Center_Dist = sqrt(sum((Sphere_Center - Tr(1:3, 4)').^2));

                if EndEffector_To_Center_Dist <= Radius
                    Target_Pose_To_Center_Dist = sqrt(sum((Sphere_Center - Target_Pose(1:3, 4)').^2));
                    % Check if the target pose is inside sphere, then display error message, else create new waypoint
                    if Target_Pose_To_Center_Dist <= Radius
                        error('The final pose is within the collision sphere!');
                    else

                        % Check for collision and move to waypoints
                        % if CheckCollision(LBRiiwa, Sphere_Center, Radius)
                        disp('Collision detected! Creating waypoint above current pose.');
                
                        % Get the current joint configuration
                        Q_Current = obj.LBRiiwa.model.getpos(); % Get current joint angles
                        Current_Pose = obj.LBRiiwa.model.fkine(Q_Current).T; % Get current end-effector pose
                        Waypoint_Above = Current_Pose; % Start with the current pose
                        Waypoint_Above(3, 4) = Current_Pose(3, 4) + 0.1; % Move above current pose
                        
                        % Inverse kinematics to get joint configuration for the waypoint
                        Q_Above = obj.LBRiiwa.model.ikcon(Waypoint_Above); % Using the new waypoint position
                        Q_Above_Traj = jtraj(Q_Current, Q_Above, 20); % Trajectory from current pose to the above waypoint
                
                        % Animate movement to the waypoint above
                        for J = 1:size(Q_Above_Traj, 1)
                            obj.LBRiiwa.model.animate(Q_Above_Traj(J, :));
                
                            LBRiiwa_EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
                            obj.Box_Gripper.setGripperBase(LBRiiwa_EndEffector_Pose);
                            obj.Box_Gripper.stationaryGripper();
                
                            drawnow();
                            pause(0.1);
                        end
                        
                        % Update current joint angles and create a new trajectory from this point to the target pose
                        Q_Current = obj.LBRiiwa.model.getpos(); % Get new joint angles after moving up
                        Current_Pose = obj.LBRiiwa.model.fkine(Q_Current).T; % Get the current pose as an SE3 object
                        X1 = Current_Pose(1:3, 4); % Extract the x, y, z coordinates from the transformation matrix
                
                        % Create a new trajectory from the current position (after the waypoint) to the target pose
                        X2 = Target_Pose(1:3, 4); % Desired end-effector position (from target pose)
                        
                        % Create RMRC trajectory from the current position to the target pose
                        New_Steps = 50; % Number of steps for the new trajectory
                        New_X = zeros(3, New_Steps);
                        S = lspb(0, 1, New_Steps); % Interpolation scalar using trapezoidal velocity profile
                        for K = 1:New_Steps
                            New_X(:, K) = X1 * (1 - S(K)) + S(K) * X2; % Interpolate positions
                        end
                
                        % Initialize new joint angles matrix
                        New_Q_Matrix = nan(New_Steps, 6);
                        New_Q_Matrix(1, :) = Q_Current; % Initial joint configuration
                
                        % Loop for RMRC trajectory execution from waypoint to target pose
                        for K = 1:New_Steps-1
                            % Compute velocity (Xdot) for x, y, and z
                            Xdot = (New_X(:, K+1) - New_X(:, K)) / Delta_T;                       
                            
                            % Get the full Jacobian for the current joint configuration
                            J = obj.LBRiiwa.model.jacob0(New_Q_Matrix(K, :));
                            J_Pos = J(1:3, :); % Consider only the first 3 rows for x, y, z motion
                            
                            % Damped Least Squares inverse of the Jacobian
                            J_Damped = J_Pos' * inv(J_Pos * J_Pos' + Lambda^2 * eye(3)); 
                            
                            % Compute joint velocities using DLS
                            Qdot = J_Damped * Xdot;  
                            
                            % Update joint angles for the next step
                            New_Q_Matrix(K+1, :) = New_Q_Matrix(K, :) + Delta_T * Qdot';
                            
                            % Animate the robot
                            obj.LBRiiwa.model.animate(New_Q_Matrix(K+1, :));
                
                            LBRiiwa_EndEffector_Pose = obj.LBRiiwa.model.fkine(obj.LBRiiwa.model.getpos()).T;
                            obj.Box_Gripper.setGripperBase(LBRiiwa_EndEffector_Pose);
                
                            obj.Box_Gripper.stationaryGripper();
                            pause(0.1); % Pause for smooth animation
                        end
                        
                        % Exit the loop since we have successfully created a new trajectory to the target pose
                        break;
                    end
                end
            end
        end
    end
end
