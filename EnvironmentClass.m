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
            
            tableRobot = PlaceObject('tableRobot.ply', [0, 1.2, 0]); 
            tableServe = PlaceObject('tableServe.ply', [0, 0.25, 0]);
            candyShelf1 = PlaceObject('candyShelf.ply', [1, 2.25, 0]);
            candyShelf2 = PlaceObject('candyShelf.ply', [-1, 2.25, 0]);
            % candytray = PlaceObject('tray.ply', [0.65, 1.575, 0.8]);
            boxRobot = PlaceObject('boxRobot.ply', [0.74, 1.25, 0.5]);
            boxBox = PlaceObject('boxBox.ply', [-1.15, 1, 0.5]);
            boxCandy = PlaceObject('boxCandy.ply', [-0.2, 0.2, 0.5]);
            creditCardReader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
            monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);

        end
    end
end