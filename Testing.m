hold on;
clear all
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/Environment')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/LBRiiwa')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/UR3')

% Setting the limits for the x, y, and z axes
xlim([-2, 2]);                                              
ylim([-2, 2]);
zlim([0, 2.5]);

% Retaining the current plot when adding new elements
hold on;                                                 

% Adding a .ply file as the flat background surface (Code 
% taken from Canvas in Lab Assignment 1 submission page) 
surf([-2, -2; 2, 2], ...               % X coordinates
     [-2, 2; -2, 2], ...               % Y coordinates
     [0.0, 0.0; 0.0, 0.0], ...                 % Z coordinates 
     'CData', imread('pinktiles.jpg'), ...            
     'FaceColor', 'texturemap');      

tableRobot = PlaceObject('tableRobot.ply', [0, 1, 0]); 
tableServe = PlaceObject('tableServe.ply', [0, 0.25, 0]); 
% 
% candyShelf1 = PlaceObject('candyShelf.ply', [1, 1.75, 0]);
% candyShelf2 = PlaceObject('candyShelf.ply', [-1, 1.75, 0]);
candyBox1 = PlaceObject('candyBox.ply', [1, 0.75, 0.5]);
candyBox2 = PlaceObject('candyBox.ply', [1, 1, 0.5]);
candyBox3 = PlaceObject('candyBox.ply', [1, 1.25, 0.5]);
% creditCardReader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
% monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);

SweetPositions1 = [                                         
    0.95, 0.7, 0.5;
    1.1, 0.7, 0.5;
    0.95, 0.8, 0.5;
    1.1, 0.8, 0.5;
    1.025, 0.75, 0.5;
];
SweetPositions2 = [                                         
    0.95, 0.95, 0.5;
    1.1, 0.95, 0.5;
    0.95, 1.05, 0.5;
    1.1, 1.05, 0.5;
    1.025, 1, 0.5;
];
SweetPositions3 = [                                         
    0.95, 1.2, 0.5;
    1.1, 1.2, 0.5;
    0.95, 1.3, 0.5;
    1.1, 1.3, 0.5;
    1.025, 1.25, 0.5
];

SweetPositions1(:,1) = SweetPositions1(:,1) + 0;
SweetPositions2(:,1) = SweetPositions2(:,1) + 0;
SweetPositions3(:,1) = SweetPositions3(:,1) + 0;

NumSteps = size(SweetPositions1, 1);
TableBlue = zeros(NumSteps, 6);                               
for i = 1:NumSteps                                             
    TableBlue(i) = PlaceObject('candyballraspberry.ply', SweetPositions1(i, :)); 
    TableBlue(i) = PlaceObject('candyballblueberry.ply', SweetPositions2(i, :)); 
    TableBlue(i) = PlaceObject('candyballgreenapple.ply', SweetPositions3(i, :)); 
end 


% Initialising the robot and the gripper and creating a robot
% at the top of the table
% UR3 = UR3(transl(0.7, 1, 0.5));
LBRiiwa = LBRiiwa(transl(-0.3, 1, 0.5));

%% Candy stuff to be continued

% Midway_Pose_Joint_Angles = [170, -80, 170, 18, 170, -29, 20] * pi / 180;
% LBRiiwa.model.animate(Midway_Pose_Joint_Angles);
% drawnow();
% 
% 
% Candy_Start_Pose = [eye(3), SweetPositions1(2,:)'; 0, 0, 0, 1] * trotx(pi);
% % Candy_Start_Pose = [eye(3), [1, 1, 1]'; 0, 0, 0, 1]  %* trotx(pi);
% Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1 %Add sweet height here
% 
% Candy_First_Waypoint_Joint_Angles = [11, -80, 170, 18, 170, -29] * pi / 180;
% Candy_Midway_Pose_Joint_Angles = [170, -80, 170, 18, 170, -29] * pi / 180;
% Candy_Second_Waypoint_Joint_Angles = [92, -80, 170, 18, 170, -29] * pi / 180;
% 
% UR3_Pose = UR3.model.getpos();
% Candy_Initial_Joint_Angles = UR3.model.ikcon(Candy_Start_Pose)
% disp(rad2deg(Candy_Initial_Joint_Angles))
% 
% % Calculate the trajectories
% Candy_Current_To_Initial = jtraj(UR3_Pose, Candy_Initial_Joint_Angles, 100);
% Candy_Initial_To_First = jtraj(Candy_Initial_Joint_Angles, Candy_First_Waypoint_Joint_Angles, 100);
% Candy_First_To_Midway = jtraj(Candy_First_Waypoint_Joint_Angles, Candy_Midway_Pose_Joint_Angles, 100);
% Candy_Midway_To_Second = jtraj(Candy_Midway_Pose_Joint_Angles, Candy_Second_Waypoint_Joint_Angles, 100);
% 
% 
% for i = 1:size(Candy_Current_To_Initial, 1)
%     % Animate robot movement
%     UR3.model.animate(Candy_Current_To_Initial(i,:));
%     drawnow();
%     pause(0);
% end
% 
% for i = 1:size(Candy_Initial_To_First, 1)
%     % Animate robot movement
%     UR3.model.animate(Candy_Initial_To_First(i,:));
%     drawnow();
%     pause(0);
% end
% 
% for i = 1:size(Candy_First_To_Midway, 1)
%     % Animate robot movement
%     UR3.model.animate(Candy_First_To_Midway(i,:));
%     drawnow();
%     pause(0);
% end
% 
% for i = 1:size(Candy_Midway_To_Second, 1)
%     % Animate robot movement
%     UR3.model.animate(Candy_Midway_To_Second(i,:));
%     drawnow();
%     pause(0);
% end
%% box movement

addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperBox')

Right_Finger = GripperBoxRight(transl([0, 0, 0]));
Left_Finger = GripperBoxLeft(transl([0, 0, 0]));

% Box1 = PlaceObject('box.ply', [-1.1, 0.85, 0.5]);
% Box2 = PlaceObject('box.ply', [-1.2, 1, 0.5]);
% Box3 = PlaceObject('box.ply', [-1.1, 1.15, 0.5]);

Box1 = PlaceObject('box.ply', [-1.1, 0.9, 0.5]);
Box2 = PlaceObject('box.ply', [-1.1, 0.9, 0.6]);
Box3 = PlaceObject('box.ply', [-1.1, 1.1, 0.5]);
Box4 = PlaceObject('box.ply', [-1.1, 1.1, 0.6]);

% Define the initial, midway, and final box positions
% Initial_Box_Poses = [
%     -1.1, 0.9, 0.6;
%     -1.1, 0.9, 0.5;
%     -1.1, 1.1, 0.6;
%     -1.1, 1.1, 0.5;
% ];

Initial_Box_Poses = [
    -1.1, 0.9, 0.6;
    -1.1, 0.9, 0.5;
];

two_Initial_Box_Poses = [
    -1.1, 1.1, 0.6;
    -1.1, 1.1, 0.5;
];

% Make 2 different set of waypoints for the 2 columns

% Define other waypoint joint angles
% First_Waypoint_Joint_Angles = [11, -80, 170, 18, 170, -29, 20] * pi / 180;
First_Waypoint_Joint_Angles = [0, -80, 180, 18, 180, -28, 0] * pi / 180;
Midway_Pose_Joint_Angles = [180, -80, 180, 18, 180, -28, 0] * pi / 180;
Second_Waypoint_Joint_Angles = [92, -80, 180, 18, 180, -28, 0] * pi / 180;
Final_Pose_Joint_Angles = [92, -118, 180, 18, 180, 12, 0] * pi / 180;

two_First_Waypoint_Joint_Angles = [0, -80, -180, 18, -180, -28, 0] * pi / 180;
two_Midway_Pose_Joint_Angles = [180, -80, -180, 18, -180, -28, 0] * pi / 180;
two_Second_Waypoint_Joint_Angles = [92, -80, -180, 18, -180, -28, 0] * pi / 180;
two_Final_Pose_Joint_Angles = [92, -118, -180, 18, -180, 12, 0] * pi / 180;

% Get the current robot pose
LBRiiwa_Pose = LBRiiwa.model.getpos();

% Loop through each Initial_Box_Pose
for idx = 1:size(Initial_Box_Poses, 1)
    % Define the start pose for each box
    Start_Pose = [eye(3), Initial_Box_Poses(idx,:)'; 0, 0, 0, 1] * troty(-pi/2);
    Start_Pose(1,4) = Start_Pose(1,4) + 0.2;
    Start_Pose(3,4) = Start_Pose(3,4) + 0.1;


    % Get the inverse kinematics solution for the current box
    Initial_Joint_Angles = LBRiiwa.model.ikcon(Start_Pose)
    Initial_Joint_Angles(1,7) = deg2rad(0)
    disp(rad2deg(Initial_Joint_Angles))

    % Calculate the trajectories for the robot
    Current_To_Initial = jtraj(LBRiiwa_Pose, Initial_Joint_Angles, 100);
    Initial_To_First = jtraj(Initial_Joint_Angles, First_Waypoint_Joint_Angles, 100);
    First_To_Midway = jtraj(First_Waypoint_Joint_Angles, Midway_Pose_Joint_Angles, 100);
    Midway_To_Second = jtraj(Midway_Pose_Joint_Angles, Second_Waypoint_Joint_Angles, 100);
    Second_To_Final = jtraj(Second_Waypoint_Joint_Angles, Final_Pose_Joint_Angles, 100);
    Final_To_Second = jtraj(Final_Pose_Joint_Angles, Second_Waypoint_Joint_Angles, 100);
    Second_To_First = jtraj(Second_Waypoint_Joint_Angles, First_Waypoint_Joint_Angles, 100);

    % Animate robot movement through each trajectory
    for i = 1:size(Current_To_Initial, 1)
        LBRiiwa.model.animate(Current_To_Initial(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    for i = 1:size(Initial_To_First, 1)
        LBRiiwa.model.animate(Initial_To_First(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    for i = 1:size(First_To_Midway, 1)
        LBRiiwa.model.animate(First_To_Midway(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)
    end

    for i = 1:size(Midway_To_Second, 1)
        LBRiiwa.model.animate(Midway_To_Second(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    for i = 1:size(Second_To_Final, 1)
        LBRiiwa.model.animate(Second_To_Final(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    % for i = 1:size(Final_To_Second, 1)
    %     LBRiiwa.model.animate(Final_To_Second(i,:));
    %     drawnow();
    %     pause(0);
    % 
    %     stationary(LBRiiwa, Right_Finger, Left_Finger)
    % 
    % end
    % 
    % 
    % for i = 1:size(Second_To_First, 1)
    %     LBRiiwa.model.animate(Second_To_First(i,:));
    %     drawnow();
    %     pause(0);
    % 
    %     stationary(LBRiiwa, Right_Finger, Left_Finger)
    % 
    % end

    % Update the robot's current pose for the next iteration
    LBRiiwa_Pose = Final_Pose_Joint_Angles; % Update to final pose
end

%% 2nd column

% Loop through each Initial_Box_Pose
for idx = 1:size(two_Initial_Box_Poses, 1)
    % Define the start pose for each box
    Start_Pose = [eye(3), two_Initial_Box_Poses(idx,:)'; 0, 0, 0, 1] * troty(-pi/2);
    Start_Pose(1,4) = Start_Pose(1,4) + 0.2;
    Start_Pose(3,4) = Start_Pose(3,4) + 0.1;


    % Get the inverse kinematics solution for the current box
    Initial_Joint_Angles = LBRiiwa.model.ikcon(Start_Pose)
    Initial_Joint_Angles(1,7) = deg2rad(0)
    disp(rad2deg(Initial_Joint_Angles))

    % Calculate the trajectories for the robot
    Current_To_Initial = jtraj(LBRiiwa_Pose, Initial_Joint_Angles, 100);
    Initial_To_First = jtraj(Initial_Joint_Angles, two_First_Waypoint_Joint_Angles, 100);
    First_To_Midway = jtraj(two_First_Waypoint_Joint_Angles, two_Midway_Pose_Joint_Angles, 100);
    Midway_To_Second = jtraj(two_Midway_Pose_Joint_Angles, two_Second_Waypoint_Joint_Angles, 100);
    Second_To_Final = jtraj(two_Second_Waypoint_Joint_Angles, two_Final_Pose_Joint_Angles, 100);
    Final_To_Second = jtraj(two_Final_Pose_Joint_Angles, two_Second_Waypoint_Joint_Angles, 100);
    Second_To_First = jtraj(two_Second_Waypoint_Joint_Angles, two_First_Waypoint_Joint_Angles, 100);

    % Animate robot movement through each trajectory
    for i = 1:size(Current_To_Initial, 1)
        LBRiiwa.model.animate(Current_To_Initial(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    for i = 1:size(Initial_To_First, 1)
        LBRiiwa.model.animate(Initial_To_First(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)
    end

    for i = 1:size(First_To_Midway, 1)
        LBRiiwa.model.animate(First_To_Midway(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)
    end

    for i = 1:size(Midway_To_Second, 1)
        LBRiiwa.model.animate(Midway_To_Second(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    for i = 1:size(Second_To_Final, 1)
        LBRiiwa.model.animate(Second_To_Final(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end

    for i = 1:size(Final_To_Second, 1)
        LBRiiwa.model.animate(Final_To_Second(i,:));
        drawnow();
        pause(0);

        stationary(LBRiiwa, Right_Finger, Left_Finger)

    end


    % for i = 1:size(Second_To_First, 1)
    %     LBRiiwa.model.animate(Second_To_First(i,:));
    %     drawnow();
    %     pause(0);
    % 
    %     stationary(LBRiiwa, Right_Finger, Left_Finger)
    % 
    % end

    % Update the robot's current pose for the next iteration
    LBRiiwa_Pose = Final_Pose_Joint_Angles; % Update to final pose
end


function stationary(LBRiiwa, Right_Finger, Left_Finger)
    % Get the end-effector pose
    EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T;

    % Set right gripper base to end-effector pose and get current right gripper pose
    % Then generate trajectories to animate the right finger to its current position (no change)
    Right_Finger.model.base = EndEffector_Pose;
    Right_Pose = Right_Finger.model.getpos();
    Right_Trajectory = jtraj(Right_Pose, Right_Pose, 5);

    % Set left gripper base to end-effector pose and get current left gripper pose
    % Then generate trajectories to animate the left finger to its current position (no change)
    Left_Finger.model.base = EndEffector_Pose;
    Left_Pose = Left_Finger.model.getpos();
    Left_Trajectory = jtraj(Left_Pose, Left_Pose, 5);
    
    % Perform the animations for the gripper fingers, animating the first row is enough since all rows contain the same values
    Right_Finger.model.animate(Right_Trajectory(1,:));
    Left_Finger.model.animate(Left_Trajectory(1,:));
end