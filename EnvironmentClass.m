classdef EnvironmentClass
    methods
        % Constructor for the class to set up a 3D environment for the 
        % simulation
        function obj = EnvironmentClass()
            % Retaining the current plot when adding new elements
            hold on;                                                 
            
            % Adding a .ply file as the flat background surface (Code 
            % taken from Canvas in Lab Assignment 1 submission page) 
            surf([-2, -2; 2, 2], ...               % X coordinates
                 [-2, 2.5; -2, 2.5], ...               % Y coordinates
                 [0.0, 0.0; 0.0, 0.0], ...                 % Z coordinates 
                 'CData', imread('pinktiles.jpg'), ...            
                 'FaceColor', 'texturemap');      
            
            Table_Robot = PlaceObject('tableRobot.ply', [0, 1.2, 0]); 
            Table_Serve = PlaceObject('tableServe.ply', [0, 0.25, 0]);
            Candy_Shelf_1 = PlaceObject('candyShelf.ply', [1, 2.25, 0]);
            Candy_Shelf_2 = PlaceObject('candyShelf.ply', [-1, 2.25, 0]);

            Mesh_Barrier = PlaceObject('meshbarrier.ply', [0, 0, 0.9]);
            Barrier1 = PlaceObject('barrier.ply', [-1.5, -0.1, 0]);
            Barrier2 = PlaceObject('barrier.ply', [-0.5, -0.1, 0]);
            Barrier3 = PlaceObject('barrier.ply', [0.5, -0.1, 0]);
            Barrier4 = PlaceObject('barrier.ply', [1.5, -0.1, 0]);

            Stage_Robot = PlaceObject('stageRobot.ply', [0.74, 1.25, 0.5]);
            Stage_Box = PlaceObject('stageBox.ply', [-1.15, 1, 0.5]);
            Stage_Candy = PlaceObject('stageCandy.ply', [-0.2, 0.2, 0.5]);

            Credit_Card_Reader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
            Monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);

            Customer = PlaceObject('personMaleCasual.ply', [-1, 1.8, 0]);
            rotate(Customer, [0, 0, 1], 90);
            Staff = PlaceObject('personFemaleBusiness.ply', [1.5, -0.8, 0]);
            
            TableRound = PlaceObject('tableRound0.3x0.3x0.3m.ply', [1.1, -0.8, 0]); 
            EmergencyStop = PlaceObject('emergencyStopButton.ply', [0.975, -0.775, 0.3]); 

            Raspberry_Jar = PlaceObject('raspberryJar.ply', [0.5, 1.6, 0.8]);
            Blueberry_Jar = PlaceObject('blueberryJar.ply', [0.8, 1.6, 0.8]);
            Greenapple_Jar = PlaceObject('greenappleJar.ply', [0.65, 1.6, 0.8]);

        end
    end
end