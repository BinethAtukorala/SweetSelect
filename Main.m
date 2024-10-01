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
BoxClass.placeBox();

% Place candies
CandyClass.placeCandy();

% Keep grippers open initially
Box_Gripper.openGripper();
Candy_Gripper.openGripper();

% Move the boxes and candy
LBRiiwaClass.moveBox();
UR3Class.moveCandy();
