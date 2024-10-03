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


% candyBox1 = PlaceObject('candyBox.ply', [1, 0.75, 0.5]);
% candyBox2 = PlaceObject('candyBox.ply', [1, 1, 0.5]);
% candyBox3 = PlaceObject('candyBox.ply', [1, 1.25, 0.5]);


% creditCardReader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
% monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);




% Raspberry = [                                         
%     0.95, 0.8, 0.5;
%     0.95, 0.85, 0.5;
%     0.95, 0.9, 0.5;
% ];
% Blueberry = [                                         
%     0.95, 0.95, 0.5;
%     0.95, 1, 0.5;
%     0.95, 1.05, 0.5;
% ];
% Greenapple = [                                         
%     0.95, 1.1, 0.5;
%     0.95, 1.15, 0.5;
%     0.95, 1.2, 0.5;
% ];

%% Candy movement


Raspberry = [ 
    1.21, 0.7, 0.6;
    1.21, 0.75, 0.6;
    % 0.95, 0.8, 0.5;
    1.21, 0.85, 0.6;
];
% Raspberry = [ 
%     1.05, 1.1, 0.5;
%     1.05, 1.15, 0.5;
%     1.05, 1.2, 0.5;
%     1.05, 1.25, 0.5;
% ];
Blueberry = [                                         
    1.15, 1, 0.6;
    1.15, 1.05, 0.6;
    1.15, 1.1, 0.6;
    1.15, 1.15, 0.6;
];
Greenapple = [                                         
    1.21, 0.9, 0.6;
    1.21, 0.95, 0.6;
    1.21, 1, 0.6;
    1.21, 1.05, 0.6;
];

dummyred = [ 
    1.21, 0.8, 0.6;
];
dummyblue = [                                         
    1.15, 0.7, 0.6;
    1.15, 0.75, 0.6;
    1.15, 0.8, 0.6;
    1.15, 0.85, 0.6;
    1.15, 0.9, 0.6;
    1.15, 0.95, 0.6;
];
dummygreen = [                                         
    1.21, 1.1, 0.6;
    1.21, 1.15, 0.6;
];

NumSteps = size(dummyred, 1);
dumred = zeros(NumSteps, 6);
for i = 1:NumSteps                                      
    dumred(i) = PlaceObject('candyballraspberry.ply', dummyred(i, :)); 
end 

NumSteps = size(dummyblue, 1);
dumblue = zeros(NumSteps, 6);
for i = 1:NumSteps                                      
    dumblue(i) = PlaceObject('candyballblueberry.ply', dummyblue(i, :)); 
end 

NumSteps = size(dummygreen, 1);
dumgreen = zeros(NumSteps, 6);
for i = 1:NumSteps                                       
    dumgreen(i) = PlaceObject('candyballgreenapple.ply', dummygreen(i, :)); 
end 

% 0.05 
% Raspberry(:,1) = Raspberry(:,1) + 0.2;
% Blueberry(:,1) = Blueberry(:,1) + 0.2;
% Greenapple(:,1) = Greenapple(:,1) + 0.2;
% 
% Raspberry(:,3) = Raspberry(:,3) + 0.1;
% Blueberry(:,3) = Blueberry(:,3) + 0.1;
% Greenapple(:,3) = Greenapple(:,3) + 0.1;


NumSteps = size(Blueberry, 1);
TableBlue = zeros(NumSteps, 6);                               
for i = 1:NumSteps                                             
    % TableBlue(i) = PlaceObject('candyballraspberry.ply', Raspberry(i, :)); 
    TableBlue(i) = PlaceObject('candyballblueberry.ply', Blueberry(i, :)); 
    TableBlue(i) = PlaceObject('candyballgreenapple.ply', Greenapple(i, :)); 
end 

for i = 1:3                                       
    TableBlue(i) = PlaceObject('candyballraspberry.ply', Raspberry(i, :)); 
end 

addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperCandy')

% Robot initialization
% UR3 = UR3(transl(0.7, 1, 0.5));
UR3 = UR3(transl(0.8, 1, 0.6));
LBRiiwa = LBRiiwa(transl(-0.3, 1, 0.5));

Candy_Right_Finger = GripperCandyRight(transl([0, 0, 0]));
Candy_Left_Finger = GripperCandyLeft(transl([0, 0, 0]));

Midway_Pose_Joint_Angles = [180, -80, 180, 18, 180, -28, 90] * pi / 180;
LBRiiwa.model.animate(Midway_Pose_Joint_Angles);
drawnow();


% Define the sets of candy positions
NumSteps = size(Raspberry, 1);


for i = 1:NumSteps
    % Calculate start pose for each candy
    Candy_Start_Pose = [eye(3), Raspberry(i, :)'; 0, 0, 0, 1] * trotx(pi);
    Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height

    Blue_Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
    Candy_Midway_Waypoint = [170, -360, -21.6, -70, 90, 37.5] * pi / 180;
    Candy_Final_Waypoint = [-161, -24, -360, -65.6, 90, -71] * pi / 180;
    
    % Get current robot pose
    UR3_Pose = UR3.model.getpos();
    
    % Calculate inverse kinematics to get initial joint angles
    Candy_Initial_Joint_Angles = UR3.model.ikcon(Candy_Start_Pose);
    
    % Display joint angles
    disp('Candy Initial Joint Angles:');
    disp(rad2deg(Candy_Initial_Joint_Angles));

    Candy_Current_To_Midway = jtraj(UR3_Pose, Blue_Midway_Waypoint, 30);
    Candy_Midway_To_Initial = jtraj(Blue_Midway_Waypoint, Candy_Initial_Joint_Angles, 30);
    Candy_Initial_To_Midway = jtraj(Candy_Initial_Joint_Angles, Blue_Midway_Waypoint, 30);
    Candy_Midway_To_Final = jtraj(Blue_Midway_Waypoint, Candy_Final_Waypoint, 30);
    
    % Animate the robot for each part of the movement

    for j = 1:size( Candy_Current_To_Midway, 1)
        UR3.model.animate( Candy_Current_To_Midway(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Midway_To_Initial, 1)
        UR3.model.animate(Candy_Midway_To_Initial(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

   for j = 1:size(Candy_Initial_To_Midway, 1)
        UR3.model.animate(Candy_Initial_To_Midway(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
   end


    for j = 1:size(Candy_Midway_To_Final, 1)
        UR3.model.animate(Candy_Midway_To_Final(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

end

for i = 1:NumSteps
    % Calculate start pose for each candy
    Candy_Start_Pose = [eye(3), Blueberry(i, :)'; 0, 0, 0, 1] * trotx(pi);
    Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height

    Blue_Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
    Candy_Midway_Waypoint = [170, -360, -21.6, -70, 90, 37.5] * pi / 180;
    Candy_Final_Waypoint = [-161, -24, -360, -65.6, 90, -71] * pi / 180;
    
    % Get current robot pose
    UR3_Pose = UR3.model.getpos();
    
    % Calculate inverse kinematics to get initial joint angles
    Candy_Initial_Joint_Angles = UR3.model.ikcon(Candy_Start_Pose);
    
    % Display joint angles
    disp('Candy Initial Joint Angles:');
    disp(rad2deg(Candy_Initial_Joint_Angles));

    Candy_Current_To_Midway = jtraj(UR3_Pose, Blue_Midway_Waypoint, 30);
    Candy_Midway_To_Initial = jtraj(Blue_Midway_Waypoint, Candy_Initial_Joint_Angles, 30);
    Candy_Initial_To_Midway = jtraj(Candy_Initial_Joint_Angles, Blue_Midway_Waypoint, 30);
    Candy_Midway_To_Final = jtraj(Blue_Midway_Waypoint, Candy_Final_Waypoint, 30);
    
    % Animate the robot for each part of the movement

    for j = 1:size( Candy_Current_To_Midway, 1)
        UR3.model.animate( Candy_Current_To_Midway(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Midway_To_Initial, 1)
        UR3.model.animate(Candy_Midway_To_Initial(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

   for j = 1:size(Candy_Initial_To_Midway, 1)
        UR3.model.animate(Candy_Initial_To_Midway(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
   end


    for j = 1:size(Candy_Midway_To_Final, 1)
        UR3.model.animate(Candy_Midway_To_Final(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

end

for i = 1:NumSteps
    % Calculate start pose for each candy
    Candy_Start_Pose = [eye(3), Greenapple(i, :)'; 0, 0, 0, 1] * trotx(pi);
    Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height

    Blue_Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
    Candy_Midway_Waypoint = [170, -360, -21.6, -70, 90, 37.5] * pi / 180;
    Candy_Final_Waypoint = [-161, -24, -360, -65.6, 90, -71] * pi / 180;
    
    % Get current robot pose
    UR3_Pose = UR3.model.getpos();
    
    % Calculate inverse kinematics to get initial joint angles
    Candy_Initial_Joint_Angles = UR3.model.ikcon(Candy_Start_Pose);
    
    % Display joint angles
    disp('Candy Initial Joint Angles:');
    disp(rad2deg(Candy_Initial_Joint_Angles));

    Candy_Current_To_Midway = jtraj(UR3_Pose, Blue_Midway_Waypoint, 30);
    Candy_Midway_To_Initial = jtraj(Blue_Midway_Waypoint, Candy_Initial_Joint_Angles, 30);
    Candy_Initial_To_Midway = jtraj(Candy_Initial_Joint_Angles, Blue_Midway_Waypoint, 30);
    Candy_Midway_To_Final = jtraj(Blue_Midway_Waypoint, Candy_Final_Waypoint, 30);
    
    % Animate the robot for each part of the movement

    for j = 1:size( Candy_Current_To_Midway, 1)
        UR3.model.animate( Candy_Current_To_Midway(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

    for j = 1:size(Candy_Midway_To_Initial, 1)
        UR3.model.animate(Candy_Midway_To_Initial(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

   for j = 1:size(Candy_Initial_To_Midway, 1)
        UR3.model.animate(Candy_Initial_To_Midway(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
   end

    for j = 1:size(Candy_Midway_To_Final, 1)
        UR3.model.animate(Candy_Midway_To_Final(j,:));
        drawnow();
        pause(0.01);
        stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
    end

end

% for i = 1:NumSteps
%     % Calculate start pose for each candy
%     Candy_Start_Pose = [eye(3), Raspberry(i, :)'; 0, 0, 0, 1] * trotx(pi);
%     Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.1;  % Adjust height
% 
%     Blue_Midway_Waypoint = [26.2, -76.4, -281, -96.1, 90, -71] * pi / 180;
%     Candy_Midway_Waypoint = [170, -360, -21.6, -70, 90, 37.5] * pi / 180;
%     Candy_Final_Waypoint = [221, -360, -21.6, -70, 90, 37.5] * pi / 180;
% 
%     % Get current robot pose
%     UR3_Pose = UR3.model.getpos();
% 
%     % Calculate inverse kinematics to get initial joint angles
%     Candy_Initial_Joint_Angles = UR3.model.ikcon(Candy_Start_Pose);
% 
%     % Display joint angles
%     disp('Candy Initial Joint Angles:');
%     disp(rad2deg(Candy_Initial_Joint_Angles));
% 
% 
%     Candy_Current_To_Midway = jtraj(UR3_Pose, Candy_Midway_Waypoint, 30);
%     Candy_Midway_To_Initial = jtraj(Candy_Midway_Waypoint, Candy_Initial_Joint_Angles, 30);
%     Candy_Initial_To_Midway = jtraj(Candy_Initial_Joint_Angles, Candy_Midway_Waypoint, 30);
%     Candy_Midway_To_Final = jtraj(Candy_Midway_Waypoint, Candy_Final_Waypoint, 30);
% 
%     % Animate the robot for each part of the movement
%     for j = 1:size(Candy_Current_To_Midway, 1)
%         UR3.model.animate(Candy_Current_To_Midway(j,:));
%         drawnow();
%         pause(0.01);
%         stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
%     end
% 
%     for j = 1:size(Candy_Midway_To_Initial, 1)
%         UR3.model.animate(Candy_Midway_To_Initial(j,:));
%         drawnow();
%         pause(0.01);
%         stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
%     end
% 
%    for j = 1:size(Candy_Initial_To_Midway, 1)
%         UR3.model.animate(Candy_Initial_To_Midway(j,:));
%         drawnow();
%         pause(0.01);
%         stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
%    end
% 
%     for j = 1:size(Candy_Midway_To_Final, 1)
%         UR3.model.animate(Candy_Midway_To_Final(j,:));
%         drawnow();
%         pause(0.01);
%         stationary(UR3, Candy_Right_Finger, Candy_Left_Finger)
%     end
% 
% end


%% box movement

% addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperBox')
% 
% Right_Finger = GripperBoxRight(transl([0, 0, 0]));
% Left_Finger = GripperBoxLeft(transl([0, 0, 0]));
% 
% OpenGripper(LBRiiwa, Right_Finger, Left_Finger, [0, -pi/4], [0, pi/4]);
% 
% % Box1 = PlaceObject('box.ply', [-1.1, 0.85, 0.5]);
% % Box2 = PlaceObject('box.ply', [-1.2, 1, 0.5]);
% % Box3 = PlaceObject('box.ply', [-1.1, 1.15, 0.5]);
% 
% Box1 = PlaceObject('box.ply', [-1.15, 0.9, 0.5]);
% Box2 = PlaceObject('box.ply', [-1.15, 0.9, 0.6]);
% Box3 = PlaceObject('box.ply', [-1.15, 1.1, 0.5]);
% Box4 = PlaceObject('box.ply', [-1.15, 1.1, 0.6]);
% 
% candy = PlaceObject('candyballraspberry.ply', [-1.2, 0.9, 0.5])
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
%     -1.15, 0.9, 0.6;
%     -1.15, 0.9, 0.5;
% ];
% 
% two_Initial_Box_Poses = [
%     -1.15, 1.1, 0.6;
%     -1.15, 1.1, 0.5;
% ];
% 
% % Make 2 different set of waypoints for the 2 columns
% 
% % Define other waypoint joint angles
% % First_Waypoint_Joint_Angles = [11, -80, 170, 18, 170, -29, 20] * pi / 180;
% First_Waypoint_Joint_Angles = [0, -80, 180, 18, 180, -28, 90] * pi / 180;
% Midway_Pose_Joint_Angles = [180, -80, 180, 18, 180, -28, 90] * pi / 180;
% Second_Waypoint_Joint_Angles = [92, -80, 180, 18, 180, -28, 90] * pi / 180;
% Final_Pose_Joint_Angles = [92, -118, 180, 18, 180, 12, 90] * pi / 180;
% 
% % two_First_Waypoint_Joint_Angles = [-7.5, -80, -180, 15.6, -173, -25.6, 83] * pi / 180;
% % two_Midway_Pose_Joint_Angles = [180, -80, -180, 18, -180, -28, 90] * pi / 180;
% % two_Second_Waypoint_Joint_Angles = [92, -80, -180, 18, -180, -28, 90] * pi / 180;
% % two_Final_Pose_Joint_Angles = [92, -118, -180, 18, -180, 12, 90] * pi / 180;
% 
% two_First_Waypoint_Joint_Angles = [-30, -80, -180, 15.6, -173, -25.6, 83] * pi / 180;
% two_Midway_Pose_Joint_Angles = [180, -80, -180, 18, -180, -28, 90] * pi / 180;
% two_Second_Waypoint_Joint_Angles = [92, -80, -180, 18, -180, -28, 90] * pi / 180;
% two_Final_Pose_Joint_Angles = [92, -118, -180, 18, -180, 12, 90] * pi / 180;
% 
% % Get the current robot pose
% LBRiiwa_Pose = LBRiiwa.model.getpos();
% 
% % Loop through each Initial_Box_Pose
% for idx = 1:size(Initial_Box_Poses, 1)
%     % Define the start pose for each box
%     Start_Pose = [eye(3), Initial_Box_Poses(idx,:)'; 0, 0, 0, 1] * troty(-pi/2);
%     Start_Pose(1,4) = Start_Pose(1,4) + 0.19;
%     Start_Pose(3,4) = Start_Pose(3,4) + 0.1;
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
%     Current_To_First = jtraj(LBRiiwa_Pose, First_Waypoint_Joint_Angles, 100);
%     First_To_Initial = jtraj(First_Waypoint_Joint_Angles, Initial_Joint_Angles, 100);
%     Initial_To_First = jtraj(Initial_Joint_Angles, First_Waypoint_Joint_Angles, 100);
%     First_To_Midway = jtraj(First_Waypoint_Joint_Angles, Midway_Pose_Joint_Angles, 100);
%     Midway_To_Second = jtraj(Midway_Pose_Joint_Angles, Second_Waypoint_Joint_Angles, 100);
%     Second_To_Final = jtraj(Second_Waypoint_Joint_Angles, Final_Pose_Joint_Angles, 100);
%     Final_To_Second = jtraj(Final_Pose_Joint_Angles, Second_Waypoint_Joint_Angles, 100);
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
% 
% %% 2nd column
% 
% % Loop through each Initial_Box_Pose
% for idx = 1:size(two_Initial_Box_Poses, 1)
%     % Define the start pose for each box
%     Start_Pose = [eye(3), two_Initial_Box_Poses(idx,:)'; 0, 0, 0, 1] * troty(-pi/2);
%     Start_Pose(1,4) = Start_Pose(1,4) + 0.19;
%     Start_Pose(3,4) = Start_Pose(3,4) + 0.1;
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
%     Second_To_Final = jtraj(two_Second_Waypoint_Joint_Angles, two_Final_Pose_Joint_Angles, 100);
%     Final_To_Second = jtraj(two_Final_Pose_Joint_Angles, two_Second_Waypoint_Joint_Angles, 100);
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
% 
% 
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