hold on
xlim([-2, 2]);                                              
ylim([-2, 2]);
zlim([0, 2.5]);

% Create an instance of the EnvironmentClass, representing the environment
Environment = EnvironmentClass();

% Create an instance of the LBRiiwaClass, representing the LBRiiwa model
LBRiiwa = LBRiiwaClass();

% Create an instance of the UR3Class, representing the UR3 model
UR3 = UR3Class();

% Create an instance of the GripperBox class, representing the box gripper for handling boxes
Box_Gripper = GripperBoxClass();

% Create an instance of the GripperBox class, representing the candy gripper for handling candies
Candy_Gripper = GripperCandyClass();

% Place boxes
Front_Box_Poses = [
    -1.15, 0.9, 0.6;
    -1.15, 0.9, 0.5;
];

Back_Box_Poses = [
    -1.15, 1.1, 0.6;
    -1.15, 1.1, 0.5;
];

Front_Boxes = [];

for i = 1:size(Front_Box_Poses, 1)
    Front_Boxes = [Front_Boxes, BoxClass(Front_Box_Poses(i, :))];
end

Back_Boxes = [];

for i = 1:size(Front_Box_Poses, 1)
    Back_Boxes = [Back_Boxes, BoxClass(Back_Box_Poses(i, :))];
end


% Place candies
CandyClass.placeCandy();

% Keep grippers open initially
Box_Gripper.openGripper();
Candy_Gripper.openGripper();

% Move the boxes and candy
LBRiiwaClass.moveBox();
UR3Class.moveCandy();
