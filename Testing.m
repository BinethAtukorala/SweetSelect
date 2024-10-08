hold on;
clear all
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/Environment')
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/LBRiiwa')
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/UR3e')
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/GripperBox')
addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/GripperCandy')

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


% candyBox1 = PlaceObject('candyBox.ply', [1, 0.75, 0.5]);
% candyBox2 = PlaceObject('candyBox.ply', [1, 1, 0.5]);
% candyBox3 = PlaceObject('candyBox.ply', [1, 1.25, 0.5]);


% creditCardReader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
% monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);


%% Candy movement


Raspberry = [ 
    0.5, 1.6, 0.8;
    0.56, 1.6, 0.8;
    0.62, 1.6, 0.8;
    0.68, 1.6, 0.8;
];

Blueberry = [                                         
    0.74, 1.6, 0.8;
    0.80, 1.6, 0.8;
    0.74, 1.54, 0.8;
    0.80, 1.54, 0.8;
];
Greenapple = [                                         
    0.5, 1.54, 0.8;
    0.56, 1.54, 0.8;
    0.62, 1.54, 0.8;
    0.68, 1.54, 0.8;
    % 0.74, 1.54, 0.8;
    % 0.80, 1.55, 0.8;
    % 0.86, 1.55, 0.8;
    % 0.85, 1.55, 0.8;
];


NumBlue = size(Blueberry, 1);
NumRed = size(Raspberry, 1);
NumGreen = size(Greenapple, 1);

TableBlue = zeros(NumBlue, 6);
TableRed = zeros(NumRed, 6);
TableGreen = zeros(NumGreen, 6);
for i = 1:NumBlue                                             
    TableBlue(i) = PlaceObject('candyballblueberry.ply', Blueberry(i, :)); 
end 

for i = 1:NumGreen                                             
    TableGreen(i) = PlaceObject('candyballgreenapple.ply', Greenapple(i, :)); 
end 

for i = 1:NumRed                                       
    TableRed(i) = PlaceObject('candyballraspberry.ply', Raspberry(i, :)); 
end 

% Box1 = PlaceObject('box.ply', [-1.15, 0.9, 0.6]);
% Box2 = PlaceObject('box.ply', [-1.15, 0.9, 0.64]);
% Box3 = PlaceObject('box.ply', [-1.15, 1.1, 0.6]);
% Box4 = PlaceObject('box.ply', [-1.15, 1.1, 0.64]);

% Box1 = PlaceObject('box.ply', [0.54, 1, 0.93]);



% Box1 = PlaceObject('box.ply', [-1.15, 0.9, 0.6]);
% rotate(Box1, [0, 0, 1], 90);

% % Place the object at the desired position
% Box1 = PlaceObject('box.ply', [0.54, 1, 0.93]);
% 
% % Get the object's current position
% pos = [0.54, 1, 0.93];
% 
% % Create an hgtransform object to apply transformations
% h = hgtransform;
% 
% % Move the box under the transformation object
% set(Box1, 'Parent', h);
% 
% % Create the transformation matrix: 
% % First, translate to the origin, then apply the rotation, then translate back
% T1 = makehgtform('translate', -pos);  % Translate to origin
% R = makehgtform('zrotate', pi/2);       % Rotate 180 degrees around Z-axis
% T2 = makehgtform('translate', pos);   % Translate back to original position
% 
% % Combine the transformations (translation, rotation, translation back)
% set(h, 'Matrix', T2 * R * T1);


candy = PlaceObject('candyballraspberry.ply', [-1.08, 0.9, 0.68]);
candy = PlaceObject('candyballraspberry.ply', [-1.15, 0.9, 0.68]);
candy = PlaceObject('candyballraspberry.ply', [-1.22, 0.9, 0.68]);

% addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperCandy')

% UR3e = UR3e(transl(0.8, 1, 0.6));

% UR3e = UR3e(transl(0.75, 1.3, 0.8));
LBRiiwa = LBRiiwa(transl(-0.3, 1, 0.5));

UR3e = UR3e(transl(0.94, 1.25, 0.8));

% Box_Right_Finger = GripperBoxRight(transl([0, 0, 0]));
% Box_Left_Finger = GripperBoxLeft(transl([0, 0, 0]));
% Candy_Right_Finger = GripperCandyRight(transl([0, 0, 0]));
% Candy_Left_Finger = GripperCandyLeft(transl([0, 0, 0]));
% 
% Midway_Pose_Joint_Angles = [180, -80, 180, 18, 180, -28, 90] * pi / 180;
% LBRiiwa.model.animate(Midway_Pose_Joint_Angles);
% 
% EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T
% Box_Right_Finger.model.base = EndEffector_Pose;
% Box_Left_Finger.model.base = EndEffector_Pose;
% 
% Box_Right_Finger.model.animate([0, pi/16])
% Box_Left_Finger.model.animate([0, pi/16])
% drawnow();

%% 
% 
% 
% 
% Robot initialization
% UR3e = UR3e(transl(0.7, 1, 0.5));
% UR3e = UR3e(transl(0.8, 1, 0.6));
% LBRiiwa = LBRiiwa(transl(-0.3, 1, 0.5));

Candy_Right_Finger = GripperCandyRight(transl([0, 0, 0]));
Candy_Left_Finger = GripperCandyLeft(transl([0, 0, 0]));

OpenGripper(UR3e, Candy_Right_Finger, Candy_Left_Finger, [0, 8*pi/25], [0, -8*pi/25]);

Midway_Pose_Joint_Angles = [180, -80, 180, 18, 180, -28, 90] * pi / 180;
LBRiiwa.model.animate(Midway_Pose_Joint_Angles);
drawnow();

% Box1 = PlaceObject('box.ply', [0.54, 1, 0.93]);
Box1 = PlaceObject('box.ply', [0.54, 1, 0.93]);

candy = PlaceObject('candyballraspberry.ply', [0.48, 1, 0.95]);
candy = PlaceObject('candyballraspberry.ply', [0.54, 1, 0.95]);
candy = PlaceObject('candyballraspberry.ply', [0.60, 1, 0.95]);

% Define the sets of candy positions
NumSteps = size(Raspberry, 1);


Candy_Final_Poses = [                                         
0.48, 1, 0.95;
0.54, 1, 0.95;
0.60, 1, 0.95;
];

% Candy_Final_Poses(:,3) = Candy_Final_Poses(:,3) + 0.2;

Candy_Final_Poses(:,3) = Candy_Final_Poses(:,3) + 0.08;


for i = 1:NumRed
    % Calculate start pose for each candy
    Candy_Start_Pose = [eye(3), Raspberry(i, :)'; 0, 0, 0, 1] * trotx(pi);
    Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.08;  % Adjust height


    % Determine which final candy pose to use, repeating if i > 3
    index = mod(i-1, size(Candy_Final_Poses, 1)) + 1
    Candy_Final_Pose = Candy_Final_Poses(index, :);
    Candy_Final = [eye(3), Candy_Final_Pose'; 0, 0, 0, 1]* trotx(pi);
    Candy_Final_Waypoint = UR3e.model.ikcon(Candy_Final)
    disp(rad2deg(Candy_Final_Waypoint))

    Candy_First_Waypoint = [-68, -11, -34, -45, -90, 36] * pi / 180;
    Candy_Second_Waypoint = [16, -26, -50, -14, -90, 106] * pi / 180;

    % Get current robot pose
    UR3e_Pose = UR3e.model.getpos();

    % Calculate inverse kinematics to get initial joint angles
    Candy_Initial_Joint_Angles = UR3e.model.ikcon(Candy_Start_Pose);

    % Display joint angles
    disp('Candy Initial Joint Angles:');
    disp(rad2deg(Candy_Initial_Joint_Angles));

    Candy_Current_To_First = jtraj(UR3e_Pose, Candy_First_Waypoint, 30);
    Candy_First_To_Initial = jtraj(Candy_First_Waypoint, Candy_Initial_Joint_Angles, 30);
    Candy_Initial_To_First = jtraj(Candy_Initial_Joint_Angles, Candy_First_Waypoint, 30);
    Candy_First_To_Second = jtraj(Candy_First_Waypoint, Candy_Second_Waypoint, 30);
    Candy_Second_To_Final = jtraj( Candy_Second_Waypoint, Candy_Final_Waypoint, 30);
    Candy_Final_To_Second = jtraj(Candy_Final_Waypoint,  Candy_Second_Waypoint, 30);

    % Animate the robot for each part of the movement


    for j = 1:size(Candy_Current_To_First, 1)
        UR3e.model.animate(Candy_Current_To_First(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_First_To_Initial, 1)
        UR3e.model.animate(Candy_First_To_Initial(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    CloseGripper(UR3e, Candy_Right_Finger, Candy_Left_Finger, [0, deg2rad(35)], [0, -deg2rad(35)]);

    for j = 1:size(Candy_Initial_To_First, 1)
        UR3e.model.animate(Candy_Initial_To_First(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_First_To_Second, 1)
        UR3e.model.animate(Candy_First_To_Second(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Second_To_Final, 1)
        UR3e.model.animate(Candy_Second_To_Final(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    OpenGripper(UR3e, Candy_Right_Finger, Candy_Left_Finger, [0, 8*pi/25], [0, -8*pi/25]);

    for j = 1:size(Candy_Final_To_Second, 1)
        UR3e.model.animate(Candy_Final_To_Second(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T

end

% idx = NumRed;

for i = 1:NumBlue
    % Calculate start pose for each candy
    Candy_Start_Pose = [eye(3), Blueberry(i, :)'; 0, 0, 0, 1] * trotx(pi);
    Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height

    % Determine which final candy pose to use, repeating if i > 3
    index = mod(i-1, size(Candy_Final_Poses, 1)) + 1
    Candy_Final_Pose = Candy_Final_Poses(index, :);
    Candy_Final = [eye(3), Candy_Final_Pose'; 0, 0, 0, 1]* trotx(pi);
    Candy_Final_Waypoint = UR3e.model.ikcon(Candy_Final)
    disp(rad2deg(Candy_Final_Waypoint))

    Candy_First_Waypoint = [-68, -11, -34, -45, -90, 36] * pi / 180;
    Candy_Second_Waypoint = [16, -26, -50, -14, -90, 106] * pi / 180;

    % Get current robot pose
    UR3e_Pose = UR3e.model.getpos();

    % Calculate inverse kinematics to get initial joint angles
    Candy_Initial_Joint_Angles = UR3e.model.ikcon(Candy_Start_Pose);

    % Display joint angles
    disp('Candy Initial Joint Angles:');
    disp(rad2deg(Candy_Initial_Joint_Angles));

    Candy_Current_To_First = jtraj(UR3e_Pose, Candy_First_Waypoint, 30);
    Candy_First_To_Initial = jtraj(Candy_First_Waypoint, Candy_Initial_Joint_Angles, 30);
    Candy_Initial_To_First = jtraj(Candy_Initial_Joint_Angles, Candy_First_Waypoint, 30);
    Candy_First_To_Second = jtraj(Candy_First_Waypoint, Candy_Second_Waypoint, 30);
    Candy_Second_To_Final = jtraj( Candy_Second_Waypoint, Candy_Final_Waypoint, 30);
    Candy_Final_To_Second = jtraj(Candy_Final_Waypoint,  Candy_Second_Waypoint, 30);

    % Animate the robot for each part of the movement
    for j = 1:size(Candy_Current_To_First, 1)
        UR3e.model.animate(Candy_Current_To_First(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_First_To_Initial, 1)
        UR3e.model.animate(Candy_First_To_Initial(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Initial_To_First, 1)
        UR3e.model.animate(Candy_Initial_To_First(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_First_To_Second, 1)
        UR3e.model.animate(Candy_First_To_Second(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Second_To_Final, 1)
        UR3e.model.animate(Candy_Second_To_Final(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Final_To_Second, 1)
        UR3e.model.animate(Candy_Final_To_Second(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T
end

% idx = NumRed + NumBlue;

for i = 1:NumGreen
    % Calculate start pose for each candy
    Candy_Start_Pose = [eye(3), Greenapple(i, :)'; 0, 0, 0, 1] * trotx(pi);
    Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height

    % Determine which final candy pose to use, repeating if i > 3
    index = mod(i-1, size(Candy_Final_Poses, 1)) + 1
    Candy_Final_Pose = Candy_Final_Poses(index, :);
    Candy_Final = [eye(3), Candy_Final_Pose'; 0, 0, 0, 1]* trotx(pi);
    Candy_Final_Waypoint = UR3e.model.ikcon(Candy_Final)
    disp(rad2deg(Candy_Final_Waypoint))

    Candy_First_Waypoint = [-68, -11, -34, -45, -90, 36] * pi / 180;
    Candy_Second_Waypoint = [16, -26, -50, -14, -90, 106] * pi / 180;

    % Get current robot pose
    UR3e_Pose = UR3e.model.getpos();

    % Calculate inverse kinematics to get initial joint angles
    Candy_Initial_Joint_Angles = UR3e.model.ikcon(Candy_Start_Pose);

    % Display joint angles
    disp('Candy Initial Joint Angles:');
    disp(rad2deg(Candy_Initial_Joint_Angles));

    Candy_Current_To_First = jtraj(UR3e_Pose, Candy_First_Waypoint, 30);
    Candy_First_To_Initial = jtraj(Candy_First_Waypoint, Candy_Initial_Joint_Angles, 30);
    Candy_Initial_To_First = jtraj(Candy_Initial_Joint_Angles, Candy_First_Waypoint, 30);
    Candy_First_To_Second = jtraj(Candy_First_Waypoint, Candy_Second_Waypoint, 30);
    Candy_Second_To_Final = jtraj( Candy_Second_Waypoint, Candy_Final_Waypoint, 30);
    Candy_Final_To_Second = jtraj(Candy_Final_Waypoint,  Candy_Second_Waypoint, 30);

    % Animate the robot for each part of the movement
    for j = 1:size(Candy_Current_To_First, 1)
        UR3e.model.animate(Candy_Current_To_First(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_First_To_Initial, 1)
        UR3e.model.animate(Candy_First_To_Initial(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Initial_To_First, 1)
        UR3e.model.animate(Candy_Initial_To_First(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_First_To_Second, 1)
        UR3e.model.animate(Candy_First_To_Second(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Second_To_Final, 1)
        UR3e.model.animate(Candy_Second_To_Final(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Final_To_Second, 1)
        UR3e.model.animate(Candy_Final_To_Second(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3e, Candy_Right_Finger, Candy_Left_Finger)
    end

    EndEffector_Pose = UR3e.model.fkine(UR3e.model.getpos()).T

end


%% box movement

% % addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperBox')
% % 
% Right_Finger = GripperBoxRight(transl([0, 0, 0]));
% Left_Finger = GripperBoxLeft(transl([0, 0, 0]));
% 
% OpenGripper(LBRiiwa, Right_Finger, Left_Finger, [0, -pi/4], [0, pi/4]);
% 
% % Box1 = PlaceObject('box.ply', [-1.1, 0.85, 0.5]);
% % Box2 = PlaceObject('box.ply', [-1.2, 1, 0.5]);
% % Box3 = PlaceObject('box.ply', [-1.1, 1.15, 0.5]);
% % 
% % Box1 = PlaceObject('box.ply', [-1.15, 0.9, 0.5]);
% % Box2 = PlaceObject('box.ply', [-1.15, 0.9, 0.6]);
% % Box3 = PlaceObject('box.ply', [-1.15, 1.1, 0.5]);
% % Box4 = PlaceObject('box.ply', [-1.15, 1.1, 0.6]);
% 
% Box1 = PlaceObject('box.ply', [-1.15, 0.9, 0.6]);
% Box2 = PlaceObject('box.ply', [-1.15, 0.9, 0.66]);
% Box3 = PlaceObject('box.ply', [-1.15, 1.1, 0.6]);
% Box4 = PlaceObject('box.ply', [-1.15, 1.1, 0.66]);
% 
% % candy = PlaceObject('candyballraspberry.ply', [-1.2, 0.9, 0.5])
% 
% % Define the initial, midway, and final box positions
% % Initial_Box_Poses = [
% %     -1.1, 0.9, 0.6;
% %     -1.1, 0.9, 0.5;
% %     -1.1, 1.1, 0.6;
% %     -1.1, 1.1, 0.5;
% % ];
% 
% Initial_Box_Poses = [
%     -1.15, 0.9, 0.66;
%     -1.15, 0.9, 0.6;
% ];
% 
% two_Initial_Box_Poses = [
%     -1.15, 1.1, 0.66;
%     -1.15, 1.1, 0.6;
% ];
% 
% 
% % rotz(pi) followed by rotx(-pi/2).
% % Final_Box_Pose = [
% %     -0.2, 0.35, 0.6;
% %     -0.23, 1.1, 0.6;
% % ];
% % 
% % 
% % Final_Box_Pose2 = [
% %     -0.26, 1.1, 0.6;
% %     -0.29, 1.1, 0.6;
% % ];
% 
% 
% 
% % Make 2 different set of waypoints for the 2 columns
% 
% % Define other waypoint joint angles
% % First_Waypoint_Joint_Angles = [11, -80, 170, 18, 170, -29, 20] * pi / 180;
% First_Waypoint_Joint_Angles = [0, -80, 180, 18, 180, -28, 90] * pi / 180;
% Midway_Pose_Joint_Angles = [180, -80, 180, 18, 180, -28, 90] * pi / 180;
% % Second_Waypoint_Joint_Angles = [92, -80, 180, 18, 180, -28, 90] * pi / 180;
% Second_Waypoint_Joint_Angles = [90, -80, 180, 18, 180, -28, 90] * pi / 180;
% % Final_Pose_Joint_Angles = [92, -118, 180, 18, 180, 12, 90] * pi / 180;
% % Final_Pose_Joint_Angles = [90, -92.8, 180, 32.4, 180, -30, 90] * pi / 180;
% 
% Final_Pose_Joint_Angles = [
% [110, -111, 180, 18, 180, 3.2, 90];
% [100,  -111, 180, 18, 180, 3.2, 90];
% ]* pi / 180;
% 
% % two_First_Waypoint_Joint_Angles = [-7.5, -80, -180, 15.6, -173, -25.6, 83] * pi / 180;
% % two_Midway_Pose_Joint_Angles = [180, -80, -180, 18, -180, -28, 90] * pi / 180;
% % two_Second_Waypoint_Joint_Angles = [92, -80, -180, 18, -180, -28, 90] * pi / 180;
% % two_Final_Pose_Joint_Angles = [92, -118, -180, 18, -180, 12, 90] * pi / 180;
% 
% two_First_Waypoint_Joint_Angles = [-30, -80, -180, 15.6, -173, -25.6, 83] * pi / 180;
% two_Midway_Pose_Joint_Angles = [180, -80, -180, 18, -180, -28, 90] * pi / 180;
% % two_Second_Waypoint_Joint_Angles = [92, -80, -180, 18, -180, -28, 90] * pi / 180;
% two_Second_Waypoint_Joint_Angles = [90, -80, -180, 18, -180, -28, 90] * pi / 180;
% % two_Final_Pose_Joint_Angles = [92, -92.8, 180, 32.4, 180, -25.6, 90] * pi / 180;
% 
% two_Final_Pose_Joint_Angles = [
% [80,  -111, -180, 18, -180, 3.2, 90];
% [70,  -111, -180, 18, -180, 3.2, 90];
% ]* pi / 180;
% 
% % rotz(pi/2) 
% % Midway_Pose = [0.35, 1, 0.95];
% % Midway_Pose = [eye(3), Midway_Pose'; 0, 0, 0, 1] * troty(pi/2) * trotz(-pi/2);
% % Midway_Angles = LBRiiwa.model.ikcon(Midway_Pose);
% % Midway1 = jtraj(First_Waypoint_Joint_Angles, Midway_Angles, 50);
% % Midway2 = jtraj(Midway_Angles, Second_Waypoint_Joint_Angles, 50);
% 
% % Get the current robot pose
% LBRiiwa_Pose = LBRiiwa.model.getpos();
% 
% 
% % Loop through each Initial_Box_Pose
% for idx = 1:size(Initial_Box_Poses, 1)
%     % Define the start pose for each box
%     Start_Pose = [eye(3), Initial_Box_Poses(idx,:)'; 0, 0, 0, 1] * troty(-pi/2);
%     Start_Pose(1,4) = Start_Pose(1,4) + 0.19;
%     Start_Pose(3,4) = Start_Pose(3,4) + 0.04;
% 
%     % Final_Pose = [eye(3), Final_Box_Pose(idx,:)'; 0, 0, 0, 1] * trotz(pi) * trotx(-pi/2)
%     % Final_Pose_Joint_Angles = LBRiiwa.model.ikcon(Final_Pose);
% 
% 
% 
%     % Get the inverse kinematics solution for the current box
%     Initial_Joint_Angles = LBRiiwa.model.ikcon(Start_Pose)
%     Initial_Joint_Angles(1,7) = Initial_Joint_Angles(1,7) + deg2rad(90)
%     % First_Waypoint_Joint_Angles(1,7) = Initial_Joint_Angles(1,7);
%     % Midway_Pose_Joint_Angles(1,7) = Initial_Joint_Angles(1,7);
%     % Second_Waypoint_Joint_Angles(1,7) = Initial_Joint_Angles(1,7);
%     % Final_Pose_Joint_Angles(1,7) = Initial_Joint_Angles(1,7);
% 
%     Initial_Joint_Angles(1,7)
%     disp(rad2deg(Initial_Joint_Angles))
% 
%     % Calculate the trajectories for the robot
%     Current_To_First = jtraj(LBRiiwa_Pose, First_Waypoint_Joint_Angles, 50);
%     First_To_Initial = jtraj(First_Waypoint_Joint_Angles, Initial_Joint_Angles, 50);
%     Initial_To_First = jtraj(Initial_Joint_Angles, First_Waypoint_Joint_Angles, 50);
%     First_To_Midway = jtraj(First_Waypoint_Joint_Angles, Midway_Pose_Joint_Angles, 50);
%     Midway_To_Second = jtraj(Midway_Pose_Joint_Angles, Second_Waypoint_Joint_Angles, 50);
%     Second_To_Final = jtraj(Second_Waypoint_Joint_Angles, Final_Pose_Joint_Angles(idx,:), 50);
%     Final_To_Second = jtraj(Final_Pose_Joint_Angles(idx,:), Second_Waypoint_Joint_Angles, 50);
%     % Second_To_First = jtraj(Second_Waypoint_Joint_Angles, First_Waypoint_Joint_Angles, 100);
% 
%     % Animate robot movement through each trajectory
% 
%     for i = 1:size(Current_To_First, 1)
%         LBRiiwa.model.animate(Current_To_First(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     for i = 1:size(First_To_Initial, 1)
%         LBRiiwa.model.animate(First_To_Initial(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T
% 
% 
%     CloseGripper(LBRiiwa, Right_Finger, Left_Finger, [0, -pi/16], [0, pi/16]);
% 
%     for i = 1:size(Initial_To_First, 1)
%         LBRiiwa.model.animate(Initial_To_First(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     for i = 1:size(First_To_Midway, 1)
%         LBRiiwa.model.animate(First_To_Midway(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     % for i = 1:size(Midway1, 1)
%     %     LBRiiwa.model.animate(Midway1(i,:));
%     %     drawnow();
%     %     pause(0);
%     %     stationary(LBRiiwa, Right_Finger, Left_Finger)
%     % end
% 
%     EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T
% 
%     for i = 1:size(Midway_To_Second, 1)
%         LBRiiwa.model.animate(Midway_To_Second(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     % for i = 1:size(Midway2, 1)
%     %     LBRiiwa.model.animate(Midway2(i,:));
%     %     drawnow();
%     %     pause(0);
%     %     stationary(LBRiiwa, Right_Finger, Left_Finger)
%     % end
% 
%     for i = 1:size(Second_To_Final, 1)
%         LBRiiwa.model.animate(Second_To_Final(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     OpenGripper(LBRiiwa, Right_Finger, Left_Finger, [0, -pi/4], [0, pi/4]);
% 
%     EndEffector_Pose = LBRiiwa.model.fkine(LBRiiwa.model.getpos()).T
% 
%     for i = 1:size(Final_To_Second, 1)
%         LBRiiwa.model.animate(Final_To_Second(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     % Update the robot's current pose for the next iteration
%     LBRiiwa_Pose = LBRiiwa.model.getpos();  % Update to final pose
% end
% 
% %% 2nd column
% 
% % Loop through each Initial_Box_Pose
% for idx = 1:size(two_Initial_Box_Poses, 1)
%     % Define the start pose for each box
%     Start_Pose = [eye(3), two_Initial_Box_Poses(idx,:)'; 0, 0, 0, 1] * troty(-pi/2);
%     Start_Pose(1,4) = Start_Pose(1,4) + 0.19;
%     Start_Pose(3,4) = Start_Pose(3,4) + 0.04;
% 
%     % Get the inverse kinematics solution for the current box
%     Initial_Joint_Angles = LBRiiwa.model.ikcon(Start_Pose)
%     Initial_Joint_Angles(1,7) = Initial_Joint_Angles(1,7) + deg2rad(90)
%     disp(rad2deg(Initial_Joint_Angles))
% 
%     % Calculate the trajectories for the robot
%     Current_To_First = jtraj(LBRiiwa_Pose, two_First_Waypoint_Joint_Angles, 100);
%     First_To_Initial = jtraj(two_First_Waypoint_Joint_Angles, Initial_Joint_Angles, 100);
%     Initial_To_First = jtraj(Initial_Joint_Angles, two_First_Waypoint_Joint_Angles, 100);
%     First_To_Midway = jtraj(two_First_Waypoint_Joint_Angles, two_Midway_Pose_Joint_Angles, 100);
%     Midway_To_Second = jtraj(two_Midway_Pose_Joint_Angles, two_Second_Waypoint_Joint_Angles, 100);
%     Second_To_Final = jtraj(two_Second_Waypoint_Joint_Angles, two_Final_Pose_Joint_Angles(idx,:), 100);
%     Final_To_Second = jtraj(two_Final_Pose_Joint_Angles(idx,:), two_Second_Waypoint_Joint_Angles, 100);
%     % Second_To_First = jtraj(Second_Waypoint_Joint_Angles, First_Waypoint_Joint_Angles, 100);
% 
%     % Animate robot movement through each trajectory
% 
%     for i = 1:size(Current_To_First, 1)
%         LBRiiwa.model.animate(Current_To_First(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
% 
%     for i = 1:size(First_To_Initial, 1)
%         LBRiiwa.model.animate(First_To_Initial(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     CloseGripper(LBRiiwa, Right_Finger, Left_Finger, [0, -pi/16], [0, pi/16]);
% 
%     for i = 1:size(Initial_To_First, 1)
%         LBRiiwa.model.animate(Initial_To_First(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     for i = 1:size(First_To_Midway, 1)
%         LBRiiwa.model.animate(First_To_Midway(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     for i = 1:size(Midway_To_Second, 1)
%         LBRiiwa.model.animate(Midway_To_Second(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     for i = 1:size(Second_To_Final, 1)
%         LBRiiwa.model.animate(Second_To_Final(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     OpenGripper(LBRiiwa, Right_Finger, Left_Finger, [0, -pi/4], [0, pi/4]);
% 
%     for i = 1:size(Final_To_Second, 1)
%         LBRiiwa.model.animate(Final_To_Second(i,:));
%         drawnow();
%         pause(0);
%         stationary(LBRiiwa, Right_Finger, Left_Finger)
%     end
% 
%     % Update the robot's current pose for the next iteration
%     LBRiiwa_Pose = LBRiiwa.model.getpos();  % Update to final pose
% end


function stationary(Robot, Right_Finger, Left_Finger)
    % Get the end-effector pose
    EndEffector_Pose = Robot.model.fkine(Robot.model.getpos()).T;

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

function OpenGripper(Robot, Right_Finger, Left_Finger, Right_FinalJoints, Left_FinalJoints)

    % Get the end-effector pose
    EndEffector_Pose = Robot.model.fkine(Robot.model.getpos()).T; 


    % Set right gripper base to end-effector pose and get current right gripper pose, then generate trajectory for opening
    Right_Finger.model.base = EndEffector_Pose;
    Right_Pose = Right_Finger.model.getpos(); 
    Right_Trajectory = jtraj(Right_Pose, Right_FinalJoints, 50);

    % Set left gripper base to end-effector pose and get current left gripper pose, then generate trajectory for opening
    Left_Finger.model.base = EndEffector_Pose; 
    Left_Pose = Left_Finger.model.getpos();
    Left_Trajectory = jtraj(Left_Pose, Left_FinalJoints, 50); 

    % Loop through the trajectories to animate the gripper fingers opening
    i = 1;
    while i <= size(Right_Trajectory, 1)
        Right_Finger.model.animate(Right_Trajectory(i, :));
        Left_Finger.model.animate(Left_Trajectory(i, :));
        pause(0);
        i = i + 1;
    end
end

function CloseGripper(Robot, Right_Finger, Left_Finger, Right_FinalJoints, Left_FinalJoints)

    % Get the end-effector pose
    EndEffector_Pose = Robot.model.fkine(Robot.model.getpos()).T;

    % Set right gripper base to end-effector pose and get current right gripper pose, then generate trajectory for closing
    Right_Finger.model.base = EndEffector_Pose;
    Right_Pose = Right_Finger.model.getpos();
    Right_Trajectory = jtraj(Right_Pose, Right_FinalJoints, 50);

    % Set left gripper base to end-effector pose and get current left gripper pose, then generate trajectory for closing
    Left_Finger.model.base = EndEffector_Pose;
    Left_Pose = Left_Finger.model.getpos();
    Left_Trajectory = jtraj(Left_Pose, Left_FinalJoints, 50);

    % Loop through the trajectories to animate the gripper fingers closing
    i = 1;
    while i <= size(Right_Trajectory, 1)
        Right_Finger.model.animate(Right_Trajectory(i, :));
        Left_Finger.model.animate(Left_Trajectory(i, :));
        pause(0);
        i = i + 1;
    end
end