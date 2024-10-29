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
                            obj.Candy_Gripper.closeGripper();
                        elseif obj.Work_Queue(1).Gripper_Status == "open"
                            obj.Gripper_Status = "open";
                            obj.Candy_Gripper.openGripper();
                        end
                    end
                    
                    % Movement
                    if obj.Work_Queue(1).Gripper_Has_Object
                        obj.moveWithCandy(obj.Work_Queue(1).traj(1, :), obj.Work_Queue(1).Gripper_Object, obj.Work_Queue(1).Gripper_Object_Start_Poses(1:4, :));
                    else
                        obj.moveWithoutCandy(obj.Work_Queue(1).traj(1, :));
                    end

                    % Delete trajectory performed
                    obj.Work_Queue(1).traj = obj.Work_Queue(1).traj(2:size(obj.Work_Queue(1).traj, 1), :);
                end
                % Delete work item performed
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
            
                Current_To_First = jtraj(UR3e_Pose, First_Waypoint, 30);
                First_To_Initial = jtraj(First_Waypoint, Candy_Waypoint, 20);
                Initial_To_First = jtraj(Candy_Waypoint, First_Waypoint, 20);
                First_To_Second = jtraj(First_Waypoint, Second_Waypoint, 30);
                Second_To_Final = jtraj(Second_Waypoint, Final_Waypoint, 20);
                Final_To_Second = jtraj(Final_Waypoint, Second_Waypoint, 20);

                obj.Work_Queue = [obj.Work_Queue;
                                    WorkQueueRobotClass("open", Current_To_First, false);          
                                    WorkQueueRobotClass("open", First_To_Initial, false);     
                                    WorkQueueRobotClass("close", Initial_To_First, true, Gripper_Object=Candy, Gripper_Object_Start_Poses=[Start_Pose]);     
                                    WorkQueueRobotClass("close", First_To_Second, true, Gripper_Object=Candy, Gripper_Object_Start_Poses=[Start_Pose]);    
                                    WorkQueueRobotClass("close", Second_To_Final, true, Gripper_Object=Candy, Gripper_Object_Start_Poses=[Start_Pose]);
                                    WorkQueueRobotClass("open", Final_To_Second, false);
                                    ];

                obj.executeQueue();
       
        end


        function moveWithCandy(obj, q_angles, Candy, Start_Pose)

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

        function moveWithoutCandy(obj, q_angles)



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


        function moveWithTeach(obj,Q)
            % Check for collision
            [Box_Faces, Box_Vertices, ~] = plyread('toolbox.ply', 'tri');
            Box_Position = [0.94, 0.85, 0.8];% Position of the box
            
            % Create the box object
            % Box = PlaceObject("toolbox.ply", Box_Position);
            
            % Get transformed vertices of the box
            Box_Vertices_Transformed = [Box_Vertices, ones(size(Box_Vertices, 1), 1)] * transl(Box_Position)';
            Box_Vertices_Transformed = Box_Vertices_Transformed(:, 1:3);  % Get transformed vertices
            
            % Calculate the distance from the box origin to its furthest vertex
            Box_Origin = Box_Position;
            Distances = sqrt(sum((Box_Vertices_Transformed - Box_Origin).^2, 2));
            Max_Distance = max(Distances);  % Maximum distance from origin to a vertex
            
            % Define sphere properties
            Sphere_Center = Box_Origin;  % Sphere center is the box origin
            Radius = Max_Distance + 0.1;  % Sphere radius is max distance + 0.1

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

            Tr = obj.UR3e.model.fkine(obj.UR3e.model.getpos).T;
            EndEffector_To_Center_Dist = sqrt(sum((Sphere_Center - Tr(1:3, 4)').^2));

            if EndEffector_To_Center_Dist <= Radius
                error('Robot movement stopped due to collision.');
                % break;  % Stop the robot movement if a collision is detected
            else
                obj.moveWithoutCandy(Q);
            end
        end

        function moveWithGivenPose(obj, Final_X, Final_Y, Final_Z)
            % Check for collision
            [Box_Faces, Box_Vertices, ~] = plyread('toolbox.ply', 'tri');
            Box_Position = [0.94, 1.5, 0.8]; % Position of the box
            
            % Get transformed vertices of the box
            Box_Vertices_Transformed = [Box_Vertices, ones(size(Box_Vertices, 1), 1)] * transl(Box_Position)';
            Box_Vertices_Transformed = Box_Vertices_Transformed(:, 1:3); % Get transformed vertices
            
            % Calculate the distance from the box origin to its furthest vertex
            Box_Origin = Box_Position;
            Distances = sqrt(sum((Box_Vertices_Transformed - Box_Origin).^2, 2));
            Max_Distance = max(Distances); % Maximum distance from origin to a vertex
            
            % Define sphere properties
            Sphere_Center = Box_Origin; % Sphere center is the box origin
            Radius = Max_Distance + 0.1; % Sphere radius is max distance + 0.1

            UR3e_EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
            obj.Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);
            
            % Animate robot in joint angles, far from singularity and high manupability
            Initial_Joint_Angles = deg2rad([0, -40, 40, -90, -90, 0]);
            obj.UR3e.model.animate(Initial_Joint_Angles); 
            
            % Define translation of the target pose
            Translation_Target = transl(Final_X, Final_Y, Final_Z);
            disp("VALUE")
            disp(Translation_Target)
            
            % Combine rotation and translation to form the target pose
            Target_Pose = Translation_Target * trotx(pi); % The final transformation matrix
            
            % Inverse kinematics to get target joint configuration
            Q_Target = obj.UR3e.model.ikcon(Target_Pose); % Using the rotated target pose
            
            % Initial joint configuration
            Q_Start = obj.UR3e.model.getpos(); % Get current joint angles
            Steps = 50; % Number of steps for the trajectory
            Delta_T = 0.05; % Discrete time step
            
            % Mask for considering x, y, z motion only
            M = [1 1 1 zeros(1, 3)]; 
            
            % Get current pose of UR3e
            UR3e_Pose = obj.UR3e.model.fkine(Q_Start).T;
            
            % Define start and end positions
            X1 = UR3e_Pose(1:3, 4); % Current end-effector position
            X2 = Target_Pose(1:3, 4) % Desired end-effector position (from target pose)
            
            % Create trajectory using RMRC
            X = zeros(3, Steps);
            S = lspb(0, 1, Steps); % Interpolation scalar using trapezoidal velocity profile
            for I = 1:Steps
                X(:, I) = X1 * (1 - S(I)) + S(I) * X2;
            end
            
            % Initialize joint angles matrix
            Q_Matrix = nan(Steps, 6);
            Q_Matrix(1, :) = Q_Start; % Initial joint configuration
            
            % Damped least squares parameter (Lambda)
            Lambda = 0.1; % Damping factor (adjust based on stability requirements)
            
            % Loop for RMRC trajectory execution
            for I = 1:Steps-1
                % Compute velocity (Xdot) for x, y, and z
                Xdot = (X(:, I+1) - X(:, I)) / Delta_T;                       
                
                % Get the full Jacobian for the current joint configuration
                J = obj.UR3e.model.jacob0(Q_Matrix(I, :));
                J_Pos = J(1:3, :); % Consider only the first 3 rows for x, y, z motion
                
                % Damped Least Squares inverse of the Jacobian
                J_Damped = J_Pos' * inv(J_Pos * J_Pos' + Lambda^2 * eye(3)); 
                
                % Compute joint velocities using DLS
                Qdot = J_Damped * Xdot;  
                
                % Update joint angles for the next step
                Q_Matrix(I+1, :) = Q_Matrix(I, :) + Delta_T * Qdot';
                
                % Animate the robot
                obj.UR3e.model.animate(Q_Matrix(I+1, :));
            
                UR3e_EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
                obj.Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);
            
                obj.Candy_Gripper.stationaryGripper();
                pause(0.1); % Pause for smooth animation

                Tr = obj.UR3e.model.fkine(obj.UR3e.model.getpos).T;
                EndEffector_To_Center_Dist = sqrt(sum((Sphere_Center - Tr(1:3, 4)').^2));

                if EndEffector_To_Center_Dist <= Radius
                    Target_Pose_To_Center_Dist = sqrt(sum((Sphere_Center - Target_Pose(1:3, 4)').^2));
                    % Check if the target pose is inside sphere, then display error message, else create new waypoint
                    if Target_Pose_To_Center_Dist <= Radius
                        error('The final pose is within the collision sphere!');
                    else

                        % Check for collision and move to waypoints
                        % if CheckCollision(UR3e, Sphere_Center, Radius)
                        disp('Collision detected! Creating waypoint above current pose.');
                
                        % Get the current joint configuration
                        Q_Current = obj.UR3e.model.getpos(); % Get current joint angles
                        Current_Pose = obj.UR3e.model.fkine(Q_Current).T; % Get current end-effector pose
                        Waypoint_Above = Current_Pose; % Start with the current pose
                        Waypoint_Above(3, 4) = Current_Pose(3, 4) + 0.1; % Move above current pose
                        
                        % Inverse kinematics to get joint configuration for the waypoint
                        Q_Above = obj.UR3e.model.ikcon(Waypoint_Above); % Using the new waypoint position
                        Q_Above_Traj = jtraj(Q_Current, Q_Above, 20); % Trajectory from current pose to the above waypoint
                
                        % Animate movement to the waypoint above
                        for J = 1:size(Q_Above_Traj, 1)
                            obj.UR3e.model.animate(Q_Above_Traj(J, :));
                
                            UR3e_EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
                            obj.Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);
                            obj.Candy_Gripper.stationaryGripper();
                
                            drawnow();
                            pause(0.1);
                        end
                        
                        % Update current joint angles and create a new trajectory from this point to the target pose
                        Q_Current = obj.UR3e.model.getpos(); % Get new joint angles after moving up
                        Current_Pose = obj.UR3e.model.fkine(Q_Current).T; % Get the current pose as an SE3 object
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
                            J = obj.UR3e.model.jacob0(New_Q_Matrix(K, :));
                            J_Pos = J(1:3, :); % Consider only the first 3 rows for x, y, z motion
                            
                            % Damped Least Squares inverse of the Jacobian
                            J_Damped = J_Pos' * inv(J_Pos * J_Pos' + Lambda^2 * eye(3)); 
                            
                            % Compute joint velocities using DLS
                            Qdot = J_Damped * Xdot;  
                            
                            % Update joint angles for the next step
                            New_Q_Matrix(K+1, :) = New_Q_Matrix(K, :) + Delta_T * Qdot';
                            
                            % Animate the robot
                            obj.UR3e.model.animate(New_Q_Matrix(K+1, :));
                
                            UR3e_EndEffector_Pose = obj.UR3e.model.fkine(obj.UR3e.model.getpos()).T;
                            obj.Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);
                
                            obj.Candy_Gripper.stationaryGripper();
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
