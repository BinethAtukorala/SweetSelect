%% RMRC with x and y axis
clear all
UR3e = UR3e(transl(0, 0, 0));
UR3e_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
steps = 50;

M = [1 1 zeros(1,4)];   

T1_x = UR3e_Pose(1,4);
T1_y = UR3e_Pose(2,4);

disp('Enter the ending pose as x and y values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');


x1 = [T1_x T1_y]';
x2 = [T2_x T2_y]';
deltaT = 0.05;                                     


x = zeros(2,steps);
s = lspb(0,1,steps);                                
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;        
end


qMatrix = nan(steps,6);

qMatrix(1,:) = UR3e.model.ikine(UR3e_Pose, 'q0', [0 0 0 0 0 0], 'mask', M);               


for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                       
    J = UR3e.model.jacob0(qMatrix(i,:));            
    J = J(1:2,:);                           
    qdot = pinv(J)*xdot;                      
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                  
end


UR3e.model.plot(qMatrix,'trail','r-');

%% RMRC with x,y and z axis
clear all
UR3e = UR3e(transl(0, 0, 0));
UR3e_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
steps = 50;

% Modify the mask to consider x, y, and z
M = [1 1 1 zeros(1,3)];   

T1_x = UR3e_Pose(1,4);
T1_y = UR3e_Pose(2,4);
T1_z = UR3e_Pose(3,4); % Include z-axis

disp('Enter the ending pose as x, y, and z values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');
T2_z = input('T2_z = '); % Take z-axis input

% Now include z-axis in trajectory points
x1 = [T1_x T1_y T1_z]';
x2 = [T2_x T2_y T2_z]';
deltaT = 0.05;                                     

% Update trajectory generation to handle 3D (x, y, z)
x = zeros(3,steps);
s = lspb(0,1,steps);                                
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;        
end

qMatrix = nan(steps,6); % Initialize for 6 joint angles

% Compute the initial joint configuration using ikine
% qMatrix(1,:) = UR3e.model.ikine(UR3e_Pose, 'q0', [0 0 0 0 0 0], 'mask', M);   
qMatrix(1, :) = UR3e.model.ikcon(UR3e_Pose, [0 0 0 0 0 0]);               


for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;  % Compute velocity for x, y, and z                       
    J = UR3e.model.jacob0(qMatrix(i,:)); % Get the full Jacobian            
    J = J(1:3,:);  % Consider only the first 3 rows for x, y, and z translational motion                         
    qdot = pinv(J)*xdot;  % Use the pseudo-inverse to compute joint velocities                   
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';  % Update joint angles                 
end

% % Plot the robot's movement
% UR3e.model.plot(qMatrix,'trail','r-');

% % Plot without arrows, links, or joints
% UR3e.model.plot(qMatrix, 'noarrow', 'nojoints', 'trail', 'r-');  % Plot robot with options to hide arrows and joints

% Plot robot while hiding links and joints, only displaying the PLY models
UR3e.model.plot(qMatrix, 'noarrow', 'nojoints', 'nobase', 'noshadow', 'trail', 'r-');


%% Joint interpolation with x,y and z axis

% Clear workspace and set figure properties
clear all
close all
clc
set(0, 'DefaultFigureWindowStyle', 'docked');

% Initialize UR3e robot
UR3e = UR3e(transl(0, 0, 0));

% Get the current pose of the robot
UR3e_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
steps = 50;

% Modify the mask to consider x, y, and z
M = [1 1 1 zeros(1,3)];   

T1_x = UR3e_Pose(1, 4);
T1_y = UR3e_Pose(2, 4);
T1_z = UR3e_Pose(3, 4); % Include z-axis

disp('Enter the ending pose as x, y, and z values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');
T2_z = input('T2_z = '); % Take z-axis input

% Define the initial and final poses in homogeneous transformation matrix
T1 = UR3e_Pose;
T2 = transl(T2_x, T2_y, T2_z); % Create the transformation for the end pose

% Compute the initial and final joint configurations using ikcon
q1 = UR3e.model.ikcon(T1, [0 0 0 0 0 0]); % Initial joint angles
q2 = UR3e.model.ikcon(T2, [0 0 0 0 0 0]); % Final joint angles

% Plot the robot's movement for initial configuration
UR3e.model.plot(q1, 'trail', 'r-');
pause(3);

% Generate joint trajectory
qMatrix = jtraj(q1, q2, steps);

% Plot the robot's movement along the trajectory
UR3e.model.plot(qMatrix, 'noarrow', 'nojoints', 'nobase', 'noshadow', 'trail', 'r-');





