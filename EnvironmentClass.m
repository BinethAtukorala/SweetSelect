classdef EnvironmentClass
    methods
        % Constructor for the class to set up a 3D environment for the 
        % simulation
        function obj = EnvironmentClass()
            addpath('C:\Data\SweetSelect\Environment')
            % Retaining the current plot when adding new elements
            hold on;                                                 
            
            % Adding a .ply file as the flat background surface (Code 
            % taken from Canvas in Lab Assignment 1 submission page) 
            surf([-2, -2; 2, 2], ...               % X coordinates
                 [-2, 2.5; -2, 2.5], ...               % Y coordinates
                 [0.0, 0.0; 0.0, 0.0], ...                 % Z coordinates 
                 'CData', imread('pinktiles.jpg'), ...            
                 'FaceColor', 'texturemap');      
            
            % tableRobot = PlaceObject('tableRobot.ply', [0, 1.2, 0]); 
            % tableServe = PlaceObject('tableServe.ply', [0, 0.25, 0]);
            % candyShelf1 = PlaceObject('candyShelf.ply', [1, 2.25, 0]);
            % candyShelf2 = PlaceObject('candyShelf.ply', [-1, 2.25, 0]);
            % candytray = PlaceObject('tray.ply', [0.65, 1.575, 0.8]);
            % boxRobot = PlaceObject('boxRobot.ply', [0.74, 1.25, 0.5]);
            % boxBox = PlaceObject('boxBox.ply', [-1.15, 1, 0.5]);
            % boxCandy = PlaceObject('boxCandy.ply', [-0.2, 0.2, 0.5]);
            % creditCardReader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
            % monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);
            % 
            % Table_Robot = PlaceObject('tableRobot.ply', [0, 1.2, 0]); 
            % Table_Serve = PlaceObject('tableServe.ply', [0, 0.25, 0]);
            % Candy_Shelf_1 = PlaceObject('candyShelf.ply', [1, 2.25, 0]);
            % Candy_Shelf_2 = PlaceObject('candyShelf.ply', [-1, 2.25, 0]);
            % % candytray = PlaceObject('tray.ply', [0.65, 1.575, 0.8]);
            % Stage_Robot = PlaceObject('stageRobot.ply', [0.74, 1.25, 0.5]);
            % Stage_Box = PlaceObject('stageBox.ply', [-1.15, 1, 0.5]);
            % Stage_Candy = PlaceObject('stageCandy.ply', [-0.2, 0.2, 0.5]);
            % Credit_Card_Reader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
            % Monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);
            % 
            % Raspberry_Jar = PlaceObject('raspberryJar.ply', [0.5, 1.6, 0.8]);
            % Blueberry_Jar = PlaceObject('blueberryJar.ply', [0.8, 1.6, 0.8]);
            % Greenapple_Jar = PlaceObject('greenappleJar.ply', [0.65, 1.6, 0.8]);

            Table_Robot = PlaceObject('tableRobot.ply', [0, 1.2, 0]); 
            Table_Serve = PlaceObject('tableServe.ply', [0, 0.25, 0]);
            Candy_Shelf_1 = PlaceObject('candyShelf.ply', [1, 2.25, 0]);
            Candy_Shelf_2 = PlaceObject('candyShelf.ply', [-1, 2.25, 0]);
            Mesh_Barrier = PlaceObject('meshbarrier.ply', [0, 0.2, 0.9]);
            Barrier1 = PlaceObject('barrier.ply', [-1.5, -0.1, 0]);
            Barrier2 = PlaceObject('barrier.ply', [-0.5, -0.1, 0]);
            Barrier3 = PlaceObject('barrier.ply', [0.5, -0.1, 0]);
            Barrier4 = PlaceObject('barrier.ply', [1.5, -0.1, 0]);
            Stage_Robot = PlaceObject('stageRobot.ply', [0.74, 1.25, 0.5]);
            Stage_Box = PlaceObject('stageBox.ply', [-1.15, 1, 0.5]);
            Stage_Candy = PlaceObject('stageCandy.ply', [-0.2, 0.2, 0.5]);
            Credit_Card_Reader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
            Monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);

            Raspberry_Jar = PlaceObject('raspberryJar.ply', [0.5, 1.6, 0.8]);
            Blueberry_Jar = PlaceObject('blueberryJar.ply', [0.8, 1.6, 0.8]);
            Greenapple_Jar = PlaceObject('greenappleJar.ply', [0.65, 1.6, 0.8]);

            Item_Box_LBRiiwa = PlaceObject("toolbox.ply", [-0.3, 1.6, 0.5]) %OR [-0.4, 0.4, 0.2] if LBRiiwa is in 0,0,0;
            Item_Box_UR3e = PlaceObject("toolbox.ply", [0.94, 0.85, 0.8]) %OR [0.94, 0.75, 0.8] for collision detection, Test this;
        end
    end
end