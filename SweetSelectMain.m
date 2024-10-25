clear all;
hold on
xlim([-2, 2]);                                              
ylim([-2, 2.5]);
zlim([0, 2]);

addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/Environment')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/LBRiiwa')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/UR3e')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperBox')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperCandy')

% addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/Environment')
% addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/LBRiiwa')
% addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/UR3e')
% addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/GripperBox')
% addpath('/Users/binadasudusinghe/Documents/MATLAB/LABASSESSMENT2/SweetSelect/GripperCandy')



% Create an instance of the EnvironmentClass, representing the environment
Environment = EnvironmentClass();

% Create an instance of the GripperBox class, representing the box gripper for handling boxes
Box_Gripper = GripperBoxClass();

% Create an instance of the GripperBox class, representing the candy gripper for handling candies
Candy_Gripper = GripperCandyClass();

% Create an instance of the LBRiiwaClass, representing the LBRiiwa model
LBRiiwa = LBRiiwaClass(Box_Gripper);

% Create an instance of the UR3Class, representing the UR3 model
UR3e = UR3eClass(Candy_Gripper);

% Keep grippers open initially
Box_Gripper.openGripper();
Candy_Gripper.openGripper();

%% Candy
% Raspberry = [ 
%     0.5, 1.6, 0.8;
%     0.56, 1.6, 0.8;
%     0.62, 1.6, 0.8;
%     0.68, 1.6, 0.8;
% ];
% 
% Blueberry = [                                         
%     0.74, 1.6, 0.8;
%     0.80, 1.6, 0.8;
%     0.74, 1.54, 0.8;
%     0.80, 1.54, 0.8;
% ];
% Greenapple = [                                         
%     0.5, 1.54, 0.8;
%     0.56, 1.54, 0.8;
%     0.62, 1.54, 0.8;
%     0.68, 1.54, 0.8;
% ];

% Raspberry = [ 
%     0.5, 1.6, 0.8;
%     0.56, 1.6, 0.8;
%     0.62, 1.6, 0.8;
%     0.68, 1.6, 0.8;
% ];
% 
% Blueberry = [                                         
%     0.74, 1.6, 0.8;
%     0.80, 1.6, 0.8;
%     0.74, 1.54, 0.8;
%     0.80, 1.54, 0.8;
% ];
% Greenapple = [                                         
%     0.5, 1.54, 0.8;
%     0.56, 1.54, 0.8;
%     0.62, 1.54, 0.8;
%     0.68, 1.54, 0.8;
% ];

% Raspberry_Jar = PlaceObject('candyJar.ply', [0.5, 1.6, 0.8]);
% Greenapple_Jar = PlaceObject('candyJar.ply', [0.65, 1.6, 0.8;]);
% Blueberry_Jar = PlaceObject('candyJar.ply', [0.8, 1.6, 0.8;]);

Raspberry_Pose = [0.5, 1.6, 0.9]; 
Blueberry_Pose = [0.66, 1.6, 0.9];  
Greenapple_Pose = [0.82, 1.6, 0.9];   

% Edit candy count here
Raspberry_Count = 2;
Blueberry_Count = 2;
Greenapple_Count = 1;
    
% Candy_Initial_Poses = [
%     Raspberry(1:Raspberry_Count, :);
%     Blueberry(1:Blueberry_Count, :);
%     Greenapple(1:Greenapple_Count, :)
% ];

Candy_Initial_Poses = [
    repmat(Raspberry_Pose, Raspberry_Count, 1);  
    repmat(Greenapple_Pose, Blueberry_Count, 1);  
    repmat(Blueberry_Pose, Greenapple_Count, 1)   
];

% Initialize an empty array for the Candy objects
Candies = [];

% Loop through each candy pose and assign the correct flavor
for i = 1:size(Candy_Initial_Poses, 1)
    if i <= Raspberry_Count
        % For Raspberry candies
        Candies = [Candies, CandyClass(Candy_Initial_Poses(i, :), "Raspberry")];
    elseif i <= Raspberry_Count + Blueberry_Count
        % For Blueberry candies
        Candies = [Candies, CandyClass(Candy_Initial_Poses(i, :), "Blueberry")];
    else
        % For Greenapple candies
        Candies = [Candies, CandyClass(Candy_Initial_Poses(i, :), "Greenapple")];
    end
end
% 
% % Place remaining unselected candies using PlaceObject
% % Unselected Raspberry candies
% for i = Raspberry_Count+1:size(Raspberry, 1)
%     PlaceObject('candyBallRaspberry.ply', Raspberry(i, :));
% end
% 
% % Unselected Blueberry candies
% for i = Blueberry_Count+1:size(Blueberry, 1)
%     PlaceObject('candyBallBlueberry.ply', Blueberry(i, :));
% end
% 
% % Unselected Greenapple candies
% for i = Greenapple_Count+1:size(Greenapple, 1)
%     PlaceObject('candyBallGreenApple.ply', Greenapple(i, :));
% end

Candy_Final_Poses = [                                         
    0.48, 1, 0.95;
    0.54, 1, 0.95;
    0.60, 1, 0.95;
];

Candy_Final_Poses(:,3) = Candy_Final_Poses(:,3) + 0.08;

%% Boxes
Front_Box_Poses = [
    -1.15, 0.9, 0.8;
    -1.15, 0.9, 0.76;
    -1.15, 0.9, 0.72;
    -1.15, 0.9, 0.68;
    -1.15, 0.9, 0.64;
    -1.15, 0.9, 0.6;
];

Back_Box_Poses = [
    -1.15, 1.1, 0.8;
    -1.15, 1.1, 0.76;
    -1.15, 1.1, 0.72;
    -1.15, 1.1, 0.68;
    -1.15, 1.1, 0.64;
    -1.15, 1.1, 0.6;
];

% Calculate how many boxes are needed
Num_Boxes = ceil((Raspberry_Count + Blueberry_Count + Greenapple_Count)/ 3);

Boxes = [];
Box_Initial_Poses = [];

% If 1 or 2 boxes are needed, select from the front
if Num_Boxes <= 6
    for i = 1:Num_Boxes
        Boxes = [Boxes, BoxClass(Front_Box_Poses(i, :))];
        Box_Initial_Poses = [Box_Initial_Poses; Front_Box_Poses(i, :)];
    end
% If more than 2 boxes are needed, take 2 from the front and the rest from the back
elseif Num_Boxes > 6
    % If more than 2 boxes are needed, take 2 from the front and the rest from the back
    for a = 1:6
        Boxes = [Boxes, BoxClass(Front_Box_Poses(a, :))];
        Box_Initial_Poses = [Box_Initial_Poses; Front_Box_Poses(a, :)];
    end

    for b = 1:(Num_Boxes - 6)
        Boxes = [Boxes, BoxClass(Back_Box_Poses(b, :))];
        Box_Initial_Poses = [Box_Initial_Poses; Back_Box_Poses(b, :)];
    end
end

% If all boxes are selected, skip the unselected box placement
if Num_Boxes < size(Front_Box_Poses, 1)
    for i = Num_Boxes + 1:size(Front_Box_Poses, 1)
        PlaceObject('box.ply', Front_Box_Poses(i, :));
    end

    for i = 1:size(Back_Box_Poses, 1)
        PlaceObject('box.ply', Back_Box_Poses(i, :));
    end
end

if Num_Boxes >= 6 && (Num_Boxes - 6) < size(Back_Box_Poses, 1)
    for i = Num_Boxes - 6 + 1:size(Back_Box_Poses, 1)
        PlaceObject('box.ply', Back_Box_Poses(i, :));
    end
end

%% Movement

% Get the total number of candies
Num_Candies = size(Candy_Initial_Poses, 1);

% Iterate through the boxes
for Box_Index = 1:Num_Boxes
    % Determine the candy indices for this box
    Start_Index = (Box_Index - 1) * 3 + 1;
    End_Index = min(Box_Index * 3, Num_Candies);

    if Box_Index <= 2

        [Box_Start_Pose] = LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Box_Initial_Poses(Box_Index,:), Boxes(Box_Index));
        Candy_Start_Poses = [];

        for x = Start_Index:End_Index
            [Candy_Start_Pose] = UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x), x);
            Candy_Start_Poses = [Candy_Start_Poses; Candy_Start_Pose];
        end

        LBRiiwa.moveFrontFromMidway(LBRiiwa, Box_Gripper, Boxes(Box_Index), Box_Index, Candies(Start_Index:End_Index), Box_Start_Pose, Candy_Start_Poses);

    else

        [Box_Start_Pose] = LBRiiwa.moveBackToMidway(LBRiiwa, Box_Gripper, Box_Initial_Poses(Box_Index,:), Boxes(Box_Index));
        Candy_Start_Poses = [];

        for x = Start_Index:End_Index
            [Candy_Start_Pose] = UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x), x);
            Candy_Start_Poses = [Candy_Start_Poses; Candy_Start_Pose];
        end
        LBRiiwa.moveBackFromMidway(LBRiiwa, Box_Gripper, Boxes(Box_Index), Box_Index, Candies(Start_Index:End_Index), Box_Start_Pose, Candy_Start_Poses);

    end
end


