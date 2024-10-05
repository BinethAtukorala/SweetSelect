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
    1.21, 0.7, 0.6;
    1.21, 0.75, 0.6;
    1.21, 0.85, 0.6;
];

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

% Place dummy candy
Dummy_Raspberry = [ 
    1.21, 0.8, 0.6;
];

Dummy_Blueberry = [                                         
    1.15, 0.7, 0.6;
    1.15, 0.75, 0.6;
    1.15, 0.8, 0.6;
    1.15, 0.85, 0.6;
    1.15, 0.9, 0.6;
    1.15, 0.95, 0.6;
];
Dummy_Greenapple = [                                         
    1.21, 1.1, 0.6;
    1.21, 1.15, 0.6;
];

for i = 1:size(Dummy_Raspberry, 1)
    PlaceObject('candyballraspberry.ply', Dummy_Raspberry(i, :));
end

for i = 1:size(Dummy_Blueberry, 1)
    PlaceObject('candyballblueberry.ply', Dummy_Blueberry(i, :));
end

for i = 1:size(Dummy_Greenapple, 1)
    PlaceObject('candyballgreenapple.ply', Dummy_Greenapple(i, :));
end


% Keep grippers open initially
Box_Gripper.openGripper();
Candy_Gripper.openGripper();

% Move the boxes and candy
LBRiiwaClass.moveBox();
UR3Class.moveCandy();
