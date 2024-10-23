%% This code is to stop robot movement near an object, when a joint angle is given. Need to edit it with teach function
% Maybe make the entire code to a function where it takes the robot as
% paramters, things to change -
 % Robot name everywhere and number of links
%% UR3e

clear all;
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/UR3e')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperCandy')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/Environment')

Candy_Gripper = GripperCandyClass();

surf([-1, -1; 1, 1], ...               % X coordinates
     [-1, 1; -1, 1], ...               % Y coordinates
     [0, 0; 0, 0], ...                 % Z coordinates 
     'CData', imread('pinkfloor.jpg'), ...            
     'FaceColor', 'texturemap');     

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
radius = max_distance + 0.2;  % Sphere radius is max distance + 0.1

% Initialize the UR3e robot
UR3e = UR3e(transl(0, 0, 0));  % Set the initial base transform

UR3e_EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);

initial_joint_angles = deg2rad([0, -40, 40, -90, -90, 0]);
UR3e.model.animate(initial_joint_angles); 

% Initial joint configuration
q_start = UR3e.model.getpos();  % Get current joint angles
scale = 0.5;
workspace = [-0.5 1.5 -0.5 1.5 -1 1];                                      
UR3e.model.animate(q_start);                 
hold on;

% Create sphere
[X, Y, Z] = sphere(20);
X = X * radius + sphereCenter(1);
Y = Y * radius + sphereCenter(2);
Z = Z * radius + sphereCenter(3);

% Plot the sphere as a point cloud or triangle mesh
makeTriangleMesh = true;  % Keep mesh plot for collision detection only

% Create the sphere mesh but make it invisible
if makeTriangleMesh
    tri = delaunay(X, Y, Z);
    % Create the mesh and set properties to make it completely invisible
    sphereTri_h = trimesh(tri, X, Y, Z, 'FaceColor', 'none', 'EdgeColor', 'none'); % Invisible sphere
end

drawnow();
view(3);
axis equal;

% Define target pose (you can modify these angles based on your requirements)
q_target = [-pi, 0, 0, 0, 0, 0];  % Example target joint configuration

% Generate trajectory using jtraj
t = 50;  % Time vector
q_traj = jtraj(q_start, q_target, t);  % Generate joint trajectory

% Move Robot along the trajectory
for i = 1:size(q_traj, 1)
    UR3e.model.animate(q_traj(i, :));  % Animate the UR3e robot

    UR3e_EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
    Candy_Gripper.setGripperBase(UR3e_EndEffector_Pose);

    Candy_Gripper.stationaryGripper();

    drawnow();
    pause(0.1);

    % Check for collision
    if CheckCollision(UR3e, sphereCenter, radius)
        disp('Robot movement stopped due to collision.');
        break;  % Stop the robot movement if a collision is detected
    end
end
%% LBRiiwa
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/LBRiiwa')

clear all;
% Load the box object and get its vertices
[box_faces, box_vertices, ~] = plyread('box.ply', 'tri');
box_position = [-0.4, 0.4, 0.2];  % Position of the box

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
radius = max_distance + 0.2;  % Sphere radius is max distance + 0.1

% Initialize the LBRiiwa robot
LBRiiwa = LBRiiwa(transl(0, 0, 0));  % Set the initial base transform

initial_joint_angles = deg2rad([0, 0, 0, -90, 0, 0, 0]);
LBRiiwa.model.animate(initial_joint_angles); 

% Initial joint configuration
q_start = LBRiiwa.model.getpos();  % Get current joint angles
scale = 0.5;
workspace = [-0.5 1.5 -0.5 1.5 -1 1];                                      
LBRiiwa.model.animate(q_start);                 
hold on;

% Create sphere
[X, Y, Z] = sphere(20);
X = X * radius + sphereCenter(1);
Y = Y * radius + sphereCenter(2);
Z = Z * radius + sphereCenter(3);

% Plot the sphere as a point cloud or triangle mesh
makeTriangleMesh = true;  % Keep mesh plot for collision detection only

% Create the sphere mesh but make it invisible
if makeTriangleMesh
    tri = delaunay(X, Y, Z);
    % Create the mesh and set properties to make it completely invisible
    sphereTri_h = trimesh(tri, X, Y, Z, 'FaceColor', 'none', 'EdgeColor', 'none'); % Invisible sphere
end

drawnow();
view(3);
axis equal;

% Define target pose (you can modify these angles based on your requirements)
q_target = deg2rad([-50, -90, 0, 0, 0, 0, 0]);  % Example target joint configuration

% Generate trajectory using jtraj
t = 50;  % Time vector
q_traj = jtraj(q_start, q_target, t);  % Generate joint trajectory

% Move Robot along the trajectory
for i = 1:size(q_traj, 1)
    LBRiiwa.model.animate(q_traj(i, :));  % Animate the LBRiiwa robot
    drawnow();
    pause(0.1);

    % Check for collision
    if CheckCollision(LBRiiwa, sphereCenter, radius)
        disp('Robot movement stopped due to collision.');
        break;  % Stop the robot movement if a collision is detected
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


