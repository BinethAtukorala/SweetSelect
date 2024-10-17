%% UR3e - RMRC using Damping Least Squares
clear all
UR3e = UR3e(transl(0, 0, 0));

% Initial joint angles
initial_joint_angles = deg2rad([0, -40, 40, -90, -90, 0]);
UR3e.model.animate(initial_joint_angles); 

% Get current pose of UR3e
UR3e_Pose = UR3e.model.fkine(UR3e.model.getpos()).T
steps = 50;

% Mask for considering x, y, z motion only
M = [1 1 1 zeros(1,3)];   

% Get initial end-effector position
T1_x = UR3e_Pose(1,4);
T1_y = UR3e_Pose(2,4);
T1_z = UR3e_Pose(3,4);

% Input the desired final position in x, y, and z
disp('Enter the ending pose as x, y, and z values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');
T2_z = input('T2_z = ');

% Define start and end positions
x1 = [T1_x T1_y T1_z]';
x2 = [T2_x T2_y T2_z]';
deltaT = 0.05;  % Discrete time step

% Create trajectory between initial and final positions
x = zeros(3, steps);
s = lspb(0, 1, steps); % Interpolation scalar using trapezoidal velocity profile
for i = 1:steps
    x(:, i) = x1 * (1 - s(i)) + s(i) * x2;
end

% Initialize joint angles matrix
qMatrix = nan(steps, 6);
qMatrix(1, :) = UR3e.model.ikcon(UR3e_Pose, initial_joint_angles); % Compute initial joint configuration

% Damped least squares parameter (Lambda)
lambda = 0.1; % Damping factor (adjust based on stability requirements)

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
    pause(0.1); % Pause for smooth animation
end

%% LBRiiwa - RMRC using Damping Least Squares
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/LBRiiwa')
clear all
LBRiiwa = LBRiiwa(transl(0, 0, 0));

% Initial joint angles
initial_joint_angles = deg2rad([0, 0, 0, -90, 0, 0, 0]);
LBRiiwa.model.animate(initial_joint_angles); 

% Get current pose of the robot
LBRiiwa_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T;
steps = 50;

% Mask for considering x, y, z motion only
M = [1 1 1 zeros(1,3)];   

% Get initial end-effector position
T1_x = LBRiiwa_Pose(1,4);
T1_y = LBRiiwa_Pose(2,4);
T1_z = LBRiiwa_Pose(3,4);

% Input the desired final position in x, y, and z
disp('Enter the ending pose as x, y, and z values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');
T2_z = input('T2_z = ');

% Define start and end positions
x1 = [T1_x T1_y T1_z]';
x2 = [T2_x T2_y T2_z]';
deltaT = 0.05;  % Discrete time step

% Create trajectory between initial and final positions
x = zeros(3, steps);
s = lspb(0, 1, steps); % Interpolation scalar using trapezoidal velocity profile
for i = 1:steps
    x(:, i) = x1 * (1 - s(i)) + s(i) * x2;
end

% Initialize joint angles matrix
qMatrix = nan(steps, 7);
qMatrix(1, :) = LBRiiwa.model.ikcon(LBRiiwa_Pose, initial_joint_angles); % Compute initial joint configuration

% Damped least squares parameter (Lambda)
lambda = 0.1; % Damping factor (adjust based on stability requirements)

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
end

%% UR3e - RMRC using pseudo-inverse
clear all
UR3e = UR3e(transl(0, 0, 0));

initial_joint_angles = deg2rad([0, -40, 40, -90, -90, 0]);
UR3e.model.animate(initial_joint_angles); 

UR3e_Pose = UR3e.model.fkine(UR3e.model.getpos()).T
steps = 50;

M = [1 1 1 zeros(1,3)];   

T1_x = UR3e_Pose(1,4);
T1_y = UR3e_Pose(2,4);
T1_z = UR3e_Pose(3,4); % Include z-axis

disp('Enter the ending pose as x, y, and z values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');
T2_z = input('T2_z = '); % Take z-axis input

x1 = [T1_x T1_y T1_z]'
x2 = [T2_x T2_y T2_z]'
deltaT = 0.05;                                     

x = zeros(3,steps);
s = lspb(0,1,steps);                                
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;        
end

qMatrix = nan(steps,6);
 
qMatrix(1, :) = UR3e.model.ikcon(UR3e_Pose, [0 0 0 0 0 0]);               

for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;  % Compute velocity for x, y, and z                       
    J = UR3e.model.jacob0(qMatrix(i,:)); % Get the full Jacobian            
    J = J(1:3,:);  % Consider only the first 3 rows for x, y, and z translational motion                         
    qdot = pinv(J)*xdot;  % Use the pseudo-inverse to compute joint velocities                   
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';  % Update joint angles
    UR3e.model.animate(qMatrix(i+1,:));
    pause(0.1)
end

%% LBRiiwa - RMRC using pseudo-inverse
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/LBRiiwa')
clear all
LBRiiwa = LBRiiwa(transl(0, 0, 0));

initial_joint_angles = deg2rad([0, 0, 0, -90, 0, 0, 0]);

LBRiiwa.model.animate(initial_joint_angles); 

LBRiiwa_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T;
steps = 50;

M = [1 1 1 zeros(1,3)];   

T1_x = LBRiiwa_Pose(1,4);
T1_y = LBRiiwa_Pose(2,4);
T1_z = LBRiiwa_Pose(3,4); % Include z-axis

disp('Enter the ending pose as x, y, and z values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');
T2_z = input('T2_z = '); % Take z-axis input

x1 = [T1_x T1_y T1_z]';
x2 = [T2_x T2_y T2_z]';
deltaT = 0.05;                                     

x = zeros(3,steps);
s = lspb(0,1,steps);                                
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;        
end

qMatrix = nan(steps,7);
 
qMatrix(1, :) = LBRiiwa.model.ikcon(LBRiiwa_Pose, [0 0 0 0 0 0 0]);               

for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;  % Compute velocity for x, y, and z                       
    J = LBRiiwa.model.jacob0(qMatrix(i,:)); % Get the full Jacobian            
    J = J(1:3,:);  % Consider only the first 3 rows for x, y, and z translational motion                         
    qdot = pinv(J)*xdot;  % Use the pseudo-inverse to compute joint velocities                   
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';  % Update joint angles
    LBRiiwa.model.animate(qMatrix(i+1,:));
    pause(0.1)
end
