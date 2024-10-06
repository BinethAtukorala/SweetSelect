hold on
xlim([-2, 2]);                                              
ylim([-2, 2]);
zlim([0, 2.5]);

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

% Place boxes
Front_Box_Poses = [
    -1.15, 0.9, 0.66;
    -1.15, 0.9, 0.6;
];

Back_Box_Poses = [
    -1.15, 1.1, 0.66;
    -1.15, 1.1, 0.6;
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

Raspberry_Candy = [];

for i = 1:size(Raspberry, 1)
    Raspberry_Candy = [Raspberry_Candy, CandyClass(Raspberry_Candy(i, :),"Raspberry")];
end

Blueberry_Candy = [];

for i = 1:size(Raspberry, 1)
    Blueberry_Candy = [Blueberry_Candy, CandyClass(Blueberry_Candy(i, :),"Blueberry")];
end

Greenapple_Candy = [];

for i = 1:size(Raspberry, 1)
    Greenapple_Candy = [Greenapple_Candy, CandyClass(Greenapple_Candy(i, :),"Greenapple")];
end

Candy_Final_Poses = [                                         
0.48, 1, 0.95;
0.54, 1, 0.95;
0.60, 1, 0.95;
];

Candy_Final_Poses(:,3) = Candy_Final_Poses(:,3) + 0.1;

% Keep grippers open initially
Box_Gripper.openGripper();
Candy_Gripper.openGripper();

% Move the boxes and candy
LBRiiwaClass.moveBox();
UR3Class.moveCandy();
