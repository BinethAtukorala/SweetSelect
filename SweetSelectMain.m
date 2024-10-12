hold on
xlim([-2, 2]);                                              
ylim([-2, 2]);
zlim([0, 2.5]);

addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/Environment')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/LBRiiwa')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/UR3e')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperBox')
addpath('/Users/bihansudusinghe/Documents/MATLAB/Assignment 2/SweetSelect/GripperCandy')

% Create an instance of the EnvironmentClass, representing the environment
Environment = EnvironmentClass();

% Create an instance of the LBRiiwaClass, representing the LBRiiwa model
LBRiiwa = LBRiiwaClass();

% Create an instance of the UR3Class, representing the UR3 model
UR3e = UR3eClass();

% Create an instance of the GripperBox class, representing the box gripper for handling boxes
Box_Gripper = GripperBoxClass();

% Create an instance of the GripperBox class, representing the candy gripper for handling candies
Candy_Gripper = GripperCandyClass();

%% Candy
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
];

% Edit candy count here
Raspberry_Count = 4;
Blueberry_Count = 4;
Greenapple_Count = 4;
    
Candy_Initial_Poses = [
    Raspberry(1:Raspberry_Count, :);
    Blueberry(1:Blueberry_Count, :);
    Greenapple(1:Greenapple_Count, :)
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

Candy_Final_Poses = [                                         
    0.48, 1, 0.95;
    0.54, 1, 0.95;
    0.60, 1, 0.95;
];

Candy_Final_Poses(:,3) = Candy_Final_Poses(:,3) + 0.1;

%% Boxes
Front_Box_Poses = [
    -1.15, 0.9, 0.66;
    -1.15, 0.9, 0.6;
];

Back_Box_Poses = [
    -1.15, 1.1, 0.66;
    -1.15, 1.1, 0.6;
];

% Calculate how many boxes are needed
Num_Boxes = ceil((Raspberry_Count + Blueberry_Count + Greenapple_Count)/ 3)

Boxes = [];
Box_Initial_Poses = [];

% If 1 or 2 boxes are needed, select from the front
if Num_Boxes <= 2
    for i = 1:size(Num_Boxes, 1)
        Boxes = [Boxes, BoxClass(Front_Box_Poses(i, :))];
        Box_Initial_Poses = [Box_Initial_Poses; Front_Box_Poses(i, :)]
    end
% If more than 2 boxes are needed, take 2 from the front and the rest from the back
elseif Num_Boxes > 2
    % If more than 2 boxes are needed, take 2 from the front and the rest from the back
    for a = 1:2
        Boxes = [Boxes, BoxClass(Front_Box_Poses(a, :))];
        Box_Initial_Poses = [Box_Initial_Poses; Front_Box_Poses(a, :)]
    end

    for b = 1:(size(Num_Boxes) - 2)
        Boxes = [Boxes, BoxClass(Back_Box_Poses(b, :))];
        Box_Initial_Poses = [Box_Initial_Poses; Back_Box_Poses(b, :)]
    end
end

%% Movement

Num_Candies = size(Candy_Initial_Poses, 1); % Get the total number of candies

% Loop through all candies
for x = 1:Num_Candies
    % Determine the box index based on the current candy index
    Box_Index = ceil(x / 3); % This will give 1 for x=1,2,3; 2 for x=4,5,6; etc.
    
    % Move the LBRiiwa and UR3e based on the current box index and candy position
    LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Box_Initial_Poses(Box_Index,:), Boxes(Box_Index));
    UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x,:));
    LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Box_Initial_Poses(Box_Index,:), Boxes(Box_Index));
end

% if size(Candy_Initial_Poses) <= 3
%     for z = 1:size(Candy_Initial_Poses)
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Selected_Boxes(1));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(z,:), Candy_Final_Poses, Candies(z,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(1))
%     end
% 
% elseif size(Candy_Initial_Poses) > 3 && size(Candy_Initial_Poses) <= 6
%     for y = 1:3
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Selected_Boxes(1));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(y,:), Candy_Final_Poses, Candies(y,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(1))
%     end
% 
%     for z = 3:size(Candy_Initial_Poses)
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(2));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(z,:), Candy_Final_Poses, Candies(z,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(2))
%     end
% 
% elseif size(Candy_Initial_Poses) > 6 && size(Candy_Initial_Poses) <= 9
%     for x = 1:3
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(1));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(1))
%     end
% 
%     for y = 3:6
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(2));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(y,:), Candy_Final_Poses, Candies(y,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(2))
%     end
% 
%     for z = 6:size(Candy_Initial_Poses)
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(3));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(z,:), Candy_Final_Poses, Candies(z,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(3))
%     end
% 
% elseif size(Candy_Initial_Poses) > 9 && size(Candy_Initial_Poses) <= 12
%     for w = 1:3
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(1));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(w,:), Candy_Final_Poses, Candies(w,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(1))
%     end
% 
%     for x = 3:6
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(2));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(2))
%     end
% 
%     for x = 6:9
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(3));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(y,:), Candy_Final_Poses, Candies(y,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(3))
%     end
% 
%     for z = 6:size(Candy_Initial_Poses)
%         LBRiiwa.moveFrontToMidway(LBRiiwa, Box_Gripper, Initial_Box_Pose, Selected_Boxes(4));
%         UR3e.moveUR3e(UR3e, Candy_Gripper, Candy_Initial_Poses(z,:), Candy_Final_Poses, Candies(z,:))
%         LBRiiwa.moveFrontFromMidway(obj, LBRiiwa, Box_Gripper, Selected_Boxes(4))
%     end
% end

% Keep grippers open initially
Box_Gripper.openGripper();
Candy_Gripper.openGripper();

