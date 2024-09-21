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
     'CData', imread('concrete.jpg'), ...            
     'FaceColor', 'texturemap');      



%  Defining the barrier positions
TableBluePositions = [                                         
    -2.5, -0.5, 0;
    -1.5, -0.5, 0;
    -0.5, -0.5, 0;
     0.5, -0.5, 0;
     1.5, -0.5, 0;
     2.5, -0.5, 0;
];

% Initialising an array with zeros and looping through each 
% barrier position to place the barriers at their specified 
% positions and rotating them as needed.
NumSteps = size(TableBluePositions, 1);
TableBlue = zeros(NumSteps, 6);                               
for i = 1:NumSteps                                             
    TableBlue(i) = PlaceObject('tableBlue1x1x0.5m.ply', TableBluePositions(i, :));  
end 

TableBrown1 = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0, 0.6, 0]); 
Box1 = PlaceObject('opencrate.ply', [-0.8, 0.2, 0.5]);
Box2 = PlaceObject('opencrate.ply', [-0.8, 0.4, 0.5]);
Box3 = PlaceObject('opencrate.ply', [-0.8, 0.6, 0.5]);


ChocolateBar1 = PlaceObject('ChocolateBar.ply', [0.5, 0.5, 0.5]);
ChocolateBar2 = PlaceObject('ChocolateBar.ply', [0.55, 0.5, 0.5]);
ChocolateBar3 = PlaceObject('ChocolateBar.ply', [0.6, 0.5, 0.5]);

ChocolateBar1 = PlaceObject('milka.ply', [0.75, 0.5, 0.5]);
ChocolateBar1 = PlaceObject('ferroro.ply', [0.9, 0.5, 0.5]);
ChocolateBar1 = PlaceObject('hershey.ply', [0.9, 0.6, 0.5]);





% Initialising the robot and the gripper and creating a robot
% at the top of the table
Robot1 = UR3e(transl(0.7, 1, 0.5));
Robot2 = UR3e(transl(0, 0.5, 0.5));

% TableBlue6 = PlaceObject('opencrate.ply', [-0.8, 0.2, 0.5]);

% 
% 
% % TableBlue6 = PlaceObject('opencrate2.ply', [0, 0, 0]);
% 
% 


% 
% %  Defining the barrier positions
% BarrierPositions = [                                         
%     -0.69,  0.9,  0;
%      0.69,  0.9,  0;
%      0,    -1.4,  0;
%     -0.69, -0.9,  0;
%      0.69, -0.9,  0;
%      0,    -1.4,  0
% ];
% 
% % Initialising an array with zeros and looping through each 
% % barrier position to place the barriers at their specified 
% % positions and rotating them as needed.
% NumSteps = size(BarrierPositions, 1);
% Barriers = zeros(NumSteps, 7);                               
% for i = 1:NumSteps                                             
%     Barriers(i) = PlaceObject('barrier1.5x0.2x1m.ply', BarrierPositions(i, :));  
%     if i == 3                                                 
%         rotate(Barriers(i), [0, 0, 1], 90);                      
%     elseif i == 6
%         rotate(Barriers(i), [0, 0, 1], -90);                    
%     end 
% end 
% 
% % Defining the table top's position to place the robot on top
% % of the table
% TableTopPosition = [0, 0, 0.5];                           
% 
% % Initialising the robot and the gripper and creating a robot
% % at the top of the table
% Robot = LinearUR3e(transl(TableTopPosition));             
% GripperLeft = LeftGripper(transl([0, 0, 0]));                                 
% GripperRight = RightGripper(transl([0, 0, 0]));                
% 
% % Placing objects in the simulation environemnt
% TableBrown = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0, 0, 0]);                % To hold the robot  
% 
% TableRound = PlaceObject('tableRound0.3x0.3x0.3m.ply', [-0.625, 1.475, 0]);       % To hold the emergency stop button
% 
% EmergencyStop = PlaceObject('emergencyStopButton.ply', [-0.75, 1.5, 0.3]);        % To press in case of emergency and is located near the human as a saefty feature  
% 
% Human = PlaceObject('personFemaleBusiness.ply', [1.25, -1.5, 0]);                 % To observe the robot in case anything goes wrong as a saefty feature 
% rotate(Human, [0, 0, 1], 180);                                                     
% 
% Bookshelf = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [1.5, 2.8, 0]);     % To hold the fire extinguisher
% rotate(Bookshelf, [0, 0, 1], 90);                                              
% 
% FireExtinguisher = PlaceObject('fireExtinguisher.ply', [2.8, -1.5, 0.5]);         % To use in case of a fire and is located near the human as a saefty feature 
% rotate(FireExtinguisher, [0, 0, 1], 180);                                 
