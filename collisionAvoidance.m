%% UR3e
% Maybe make the entire code to a function where it takes the robot as
% paramters, things to change -
 % Robot name everywhere and number of links
clear all;
 addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperCandy')

Candy_Gripper = GripperCandyClass();

surf([-1, -1; 1, 1], ...               % X coordinates
     [-1, 1; -1, 1], ...               % Y coordinates
     [0, 0; 0, 0], ...                 % Z coordinates 
     'CData', imread('pinkfloor.jpg'), ...            
     'FaceColor', 'texturemap'); 
% Load the box object and get its vertices
[box_faces, box_vertices, ~] = plyread('box.ply', 'tri');
box_position = [0, 0.3, 0];  % Position of the box

% Create the box object
box = PlaceObject("box.ply", box_position);

% Get transformed vertices of the box
box_vertices_transformed = [box_vertices, ones(size(box_vertices, 1), 1)] * transl(box_position)';
box_vertices_transformed = box_vertices_transformed(:, 1:3);  % Get transformed vertices

% Calculate the distance from the box origin to its furthest vertex
box_origin = box_position;
distances = sqrt(sum((box_vertices_transformed - box_origin).^2, 2));
max_distance = max(distances);  % Maximum distance from origin to a vertex

% Define sphere properties
sphereCenter = box_origin;  % Sphere center is the box origin
radius = max_distance + 0.1; % Sphere radius is max distance + 0.2

% Initialize the UR3e robot
UR3e = UR3e(transl(0, 0, 0));  % Set the initial base transform

UR3e_EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);

initial_joint_angles = deg2rad([0, -40, 40, -90, -90, 0]);
UR3e.model.animate(initial_joint_angles); 

% Loop to get valid final pose
valid_pose = false;  % Flag to indicate valid pose
while ~valid_pose
    % Get the final pose from user input
    final_x = input('Enter the final X value for the robot pose: ');
    final_y = input('Enter the final Y value for the robot pose: ');
    final_z = input('Enter the final Z value for the robot pose: ');
    
    % Define translation of the target pose
    translation_target = transl(final_x, final_y, final_z);
    
    % Define a 180-degree rotation about the X-axis
    % R_x_180 = trotx(pi);  % Rotation matrix for 180-degree rotation about the X-axis
    R_x_180 = 1; %NO need this
    % Combine rotation and translation to form the target pose
    target_pose = translation_target * trotx(pi);  % The final transformation matrix
    
    % Check if the final pose is within the sphere
    endEffectorToCenterDist = sqrt(sum((sphereCenter - target_pose(1:3, 4)').^2));

    if endEffectorToCenterDist > radius
        valid_pose = true;  % Valid pose found
    else
        disp('The final pose is within the collision sphere! Please enter a new pose.');
    end
end

% Inverse kinematics to get target joint configuration
q_target = UR3e.model.ikcon(target_pose);  % Using the rotated target pose

% Initial joint configuration
q_start = UR3e.model.getpos();  % Get current joint angles
steps = 50;  % Number of steps for the trajectory
deltaT = 0.05;  % Discrete time step

% Mask for considering x, y, z motion only
M = [1 1 1 zeros(1, 3)]; 

% Get current pose of UR3e
UR3e_Pose = UR3e.model.fkine(q_start).T;

% Define start and end positions
x1 = UR3e_Pose(1:3, 4);  % Current end-effector position
x2 = target_pose(1:3, 4);  % Desired end-effector position (from target pose)

% Create trajectory using RMRC
x = zeros(3, steps);
s = lspb(0, 1, steps); % Interpolation scalar using trapezoidal velocity profile
for i = 1:steps
    x(:, i) = x1 * (1 - s(i)) + s(i) * x2;
end

% Initialize joint angles matrix
qMatrix = nan(steps, 6);
qMatrix(1, :) = q_start;  % Initial joint configuration

% Damped least squares parameter (Lambda)
lambda = 0.1; % Damping factor (adjust based on stability requirements)

% Loop for RMRC trajectory execution
for i = 1:steps-1
    % Compute velocity (xdot) for x, y, and z
    xdot = (x(:, i+1) - x(:, i)) / deltaT;                       
    
    % Get the full Jacobian for the current joint configuration
    J = UR3e.model.jacob0(qMatrix(i, :));
    J_pos = J(1:3, :);  % Consider only the first 3 rows for x, y, z motion
    
    % Damped Least Squares inverse of the Jacobian
    J_damped = J_pos' * inv(J_pos * J_pos' + lambda^2 * eye(3)); 
    
    % Compute joint velocities using DLS
    qdot = J_damped * xdot;  
    
    % Update joint angles for the next step
    qMatrix(i+1, :) = qMatrix(i, :) + deltaT * qdot';
    
    % Animate the robot
    UR3e.model.animate(qMatrix(i+1, :));

    UR3e_EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
    Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);

    Candy_Gripper.stationaryGripper();
    pause(0.1); % Pause for smooth animation
    
    % Check for collision and move to waypoints
    if CheckCollision(UR3e, sphereCenter, radius)
        disp('Collision detected! Creating waypoint above current pose.');

        % Get the current joint configuration
        q_current = UR3e.model.getpos();  % Get current joint angles
        current_pose = UR3e.model.fkine(q_current).T; % Get current end-effector pose
        waypoint_above = current_pose;  % Start with the current pose
        waypoint_above(3, 4) = current_pose(3, 4) + 0.1;  % Move above current pose
        
        % Inverse kinematics to get joint configuration for the waypoint
        q_above = UR3e.model.ikcon(waypoint_above);  % Using the new waypoint position
        q_above_traj = jtraj(q_current, q_above, 20);  % Traject0.ory from current pose to the above waypoint

        % Animate movement to the waypoint above
        for j = 1:size(q_above_traj, 1)
            UR3e.model.animate(q_above_traj(j, :));

            UR3e_EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
            Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);
            Candy_Gripper.stationaryGripper();

            drawnow();
            pause(0.1);
        end
        
        % Update current joint angles and create a new trajectory from this point to the target pose
        q_current = UR3e.model.getpos();  % Get new joint angles after moving up
        current_pose = UR3e.model.fkine(q_current).T;  % Get the current pose as an SE3 object
        x1 = current_pose(1:3, 4);  % Extract the x, y, z coordinates from the transformation matrix

        % Create a new trajectory from the current position (after the waypoint) to the target pose
        x2 = target_pose(1:3, 4);  % Desired end-effector position (from target pose)
        
        % Create RMRC trajectory from the current position to the target pose
        new_steps = 50;  % Number of steps for the new trajectory
        new_x = zeros(3, new_steps);
        s = lspb(0, 1, new_steps); % Interpolation scalar using trapezoidal velocity profile
        for k = 1:new_steps
            new_x(:, k) = x1 * (1 - s(k)) + s(k) * x2;  % Interpolate positions
        end

        % Initialize new joint angles matrix
        new_qMatrix = nan(new_steps, 6);
        new_qMatrix(1, :) = q_current;  % Initial joint configuration

        % Loop for RMRC trajectory execution from waypoint to target pose
        for k = 1:new_steps-1
            % Compute velocity (xdot) for x, y, and z
            xdot = (new_x(:, k+1) - new_x(:, k)) / deltaT;                       
            
            % Get the full Jacobian for the current joint configuration
            J = UR3e.model.jacob0(new_qMatrix(k, :));
            J_pos = J(1:3, :);  % Consider only the first 3 rows for x, y, z motion
            
            % Damped Least Squares inverse of the Jacobian
            J_damped = J_pos' * inv(J_pos * J_pos' + lambda^2 * eye(3)); 
            
            % Compute joint velocities using DLS
            qdot = J_damped * xdot;  
            
            % Update joint angles for the next step
            new_qMatrix(k+1, :) = new_qMatrix(k, :) + deltaT * qdot';
            
            % Animate the robot
            UR3e.model.animate(new_qMatrix(k+1, :));

            UR3e_EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
            Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);

            Candy_Gripper.stationaryGripper();
            pause(0.1); % Pause for smooth animation
        end
        
        % Exit the loop since we have successfully created a new trajectory to the target pose
        break;
    end
end

%% LBRiiwa
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/LBRiiwa')
clear all;
% Load the box object and get its vertices
[box_faces, box_vertices, ~] = plyread('box.ply', 'tri');
box_position = [0, 0.4, 0];  % Position of the box

% Create the box object
box = PlaceObject("box.ply", box_position);

% Get transformed vertices of the box
box_vertices_transformed = [box_vertices, ones(size(box_vertices, 1), 1)] * transl(box_position)';
box_vertices_transformed = box_vertices_transformed(:, 1:3);  % Get transformed vertices

% Calculate the distance from the box origin to its furthest vertex
box_origin = box_position;
distances = sqrt(sum((box_vertices_transformed - box_origin).^2, 2));
max_distance = max(distances);  % Maximum distance from origin to a vertex

% Define sphere properties
sphereCenter = box_origin;  % Sphere center is the box origin
radius = max_distance + 0.2; % Sphere radius is max distance + 0.2

% Initialize the LBRiiwa robot
LBRiiwa = LBRiiwa(transl(0, 0, 0));  % Set the initial base transform

initial_joint_angles = deg2rad([0, 0, 0, -90, 0, 0, 0]);
LBRiiwa.model.animate(initial_joint_angles); 

% Loop to get valid final pose
valid_pose = false;  % Flag to indicate valid pose
while ~valid_pose
    % Get the final pose from user input
    final_x = input('Enter the final X value for the robot pose: ');
    final_y = input('Enter the final Y value for the robot pose: ');
    final_z = input('Enter the final Z value for the robot pose: ');
    
    % Define translation of the target pose
    translation_target = transl(final_x, final_y, final_z);
    
    % Define a 180-degree rotation about the X-axis
    % R_x_180 = trotx(pi);  % Rotation matrix for 180-degree rotation about the X-axis
    R_x_180 = 1;
    % Combine rotation and translation to form the target pose
    target_pose = translation_target * R_x_180;  % The final transformation matrix
    
    % Check if the final pose is within the sphere
    endEffectorToCenterDist = sqrt(sum((sphereCenter - target_pose(1:3, 4)').^2));

    if endEffectorToCenterDist > radius
        valid_pose = true;  % Valid pose found
    else
        disp('The final pose is within the collision sphere! Please enter a new pose.');
    end
end

% Inverse kinematics to get target joint configuration
q_target = LBRiiwa.model.ikcon(target_pose);  % Using the rotated target pose

% Initial joint configuration
q_start = LBRiiwa.model.getpos();  % Get current joint angles
steps = 50;  % Number of steps for the trajectory
deltaT = 0.05;  % Discrete time step

% Mask for considering x, y, z motion only
M = [1 1 1 zeros(1, 3)]; 

% Get current pose of LBRiiwa
LBRiiwa_Pose = LBRiiwa.model.fkine(q_start).T;

% Define start and end positions
x1 = LBRiiwa_Pose(1:3, 4);  % Current end-effector position
x2 = target_pose(1:3, 4);  % Desired end-effector position (from target pose)

% Create trajectory using RMRC
x = zeros(3, steps);
s = lspb(0, 1, steps); % Interpolation scalar using trapezoidal velocity profile
for i = 1:steps
    x(:, i) = x1 * (1 - s(i)) + s(i) * x2;
end

% Initialize joint angles matrix
qMatrix = nan(steps, 7);
qMatrix(1, :) = q_start;  % Initial joint configuration

% Damped least squares parameter (Lambda)
lambda = 0.1; % Damping factor (adjust based on stability requirements)

% Loop for RMRC trajectory execution
for i = 1:steps-1
    % Compute velocity (xdot) for x, y, and z
    xdot = (x(:, i+1) - x(:, i)) / deltaT;                       
    
    % Get the full Jacobian for the current joint configuration
    J = LBRiiwa.model.jacob0(qMatrix(i, :));
    J_pos = J(1:3, :);  % Consider only the first 3 rows for x, y, z motion
    
    % Damped Least Squares inverse of the Jacobian
    J_damped = J_pos' * inv(J_pos * J_pos' + lambda^2 * eye(3)); 
    
    % Compute joint velocities using DLS
    qdot = J_damped * xdot;  
    
    % Update joint angles for the next step
    qMatrix(i+1, :) = qMatrix(i, :) + deltaT * qdot';
    
    % Animate the robot
    LBRiiwa.model.animate(qMatrix(i+1, :));
    pause(0.1); % Pause for smooth animation
    
    % Check for collision and move to waypoints
    if CheckCollision(LBRiiwa, sphereCenter, radius)
        disp('Collision detected! Creating waypoint above current pose.');

        % Get the current joint configuration
        q_current = LBRiiwa.model.getpos();  % Get current joint angles
        current_pose = LBRiiwa.model.fkine(q_current).T; % Get current end-effector pose
        waypoint_above = current_pose;  % Start with the current pose
        waypoint_above(3, 4) = current_pose(3, 4) + 0.2;  % Move above current pose
        
        % Inverse kinematics to get joint configuration for the waypoint
        q_above = LBRiiwa.model.ikcon(waypoint_above);  % Using the new waypoint position
        q_above_traj = jtraj(q_current, q_above, 20);  % Trajectory from current pose to the above waypoint

        % Animate movement to the waypoint above
        for j = 1:size(q_above_traj, 1)
            LBRiiwa.model.animate(q_above_traj(j, :));
            drawnow();
            pause(0.1);
        end
        
        % Update current joint angles and create a new trajectory from this point to the target pose
        q_current = LBRiiwa.model.getpos();  % Get new joint angles after moving up
        current_pose = LBRiiwa.model.fkine(q_current).T;  % Get the current pose as an SE3 object
        x1 = current_pose(1:3, 4);  % Extract the x, y, z coordinates from the transformation matrix

        % Create a new trajectory from the current position (after the waypoint) to the target pose
        new_traj = jtraj(q_current, q_target, steps);  % Create a joint-space trajectory from the current pose to the target pose

        % Animate movement along the new joint-space trajectory
        for k = 1:steps
            LBRiiwa.model.animate(new_traj(k, :));
            drawnow();
            pause(0.1);
        end
        
        % Exit the loop since we have successfully created a new trajectory to the target pose
        break;
    end
end

function isCollision = CheckCollision(robot, sphereCenter, radius)
    % Check if the robot end effector is colliding with the sphere
    tr = robot.model.fkine(robot.model.getpos).T;
    endEffectorToCenterDist = sqrt(sum((sphereCenter - tr(1:3, 4)').^2));

    if endEffectorToCenterDist <= radius
        disp('Oh no a collision!');
        isCollision = true;  % Indicate a collision
    else
        disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
        isCollision = false;  % No collision
    end
end
