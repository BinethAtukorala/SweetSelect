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

TableRobot = PlaceObject('robottable2.ply', [0, 0.6, 0]); 
TableServe = PlaceObject('servingtable2.ply', [0, 0, 1]); 
Box1 = PlaceObject('opencrate2.ply', [-0.8, 0.2, 0.5]);
Box2 = PlaceObject('opencrate2.ply', [-0.8, 0.4, 0.5]);
Box3 = PlaceObject('opencrate2.ply', [-0.8, 0.6, 0.5]);


SweetBox1 = PlaceObject('candyshelf2.ply', [1, 0.75, 0.5]);
SweetBox1 = PlaceObject('candybox2.ply', [1, 0.75, 0.5]);
SweetBox2 = PlaceObject('candybox2.ply', [1, 1, 0.5]);
SweetBox3 = PlaceObject('candybox2.ply', [1, 1.25, 0.5]);
SweetBox3 = PlaceObject('creditcardreader.ply', [1.2, 1.25, 0.5]);
SweetBox3 = PlaceObject('tv2.ply', [0.8, 1.25, 0.5]);



SweetPositions1 = [                                         
    0.95, 0.7, 0.5;
    1.1, 0.7, 0.5;
    0.95, 0.8, 0.5;
    1.1, 0.8, 0.5;
    1.025, 0.75, 0.5;
];
SweetPositions2 = [                                         
    0.95, 0.95, 0.5;
    1.1, 0.95, 0.5;
    0.95, 1.05, 0.5;
    1.1, 1.05, 0.5;
    1.025, 1, 0.5;
];
SweetPositions3 = [                                         
    0.95, 1.2, 0.5;
    1.1, 1.2, 0.5;
    0.95, 1.3, 0.5;
    1.1, 1.3, 0.5;
    1.025, 1.25, 0.5
];


NumSteps = size(SweetPositions1, 1);
TableBlue = zeros(NumSteps, 6);                               
for i = 1:NumSteps                                             
    TableBlue(i) = PlaceObject('candyballraspberry.ply', SweetPositions1(i, :)); 
    TableBlue(i) = PlaceObject('candyballblueberry.ply', SweetPositions2(i, :)); 
    TableBlue(i) = PlaceObject('candyballgreenapple.ply', SweetPositions3(i, :)); 
end 





% Initialising the robot and the gripper and creating a robot
% at the top of the table
Robot1 = UR3e(transl(0.7, 1, 0.5));
Robot2 = UR3e(transl(0, 0.5, 0.5));

