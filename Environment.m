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




TableBluePositions = [                                         
    -2.5, -0.5, 0;
    -1.5, -0.5, 0;
    -0.5, -0.5, 0;
     0.5, -0.5, 0;
     1.5, -0.5, 0;
     2.5, -0.5, 0;
];

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

