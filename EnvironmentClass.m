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
                 [-2, 2; -2, 2], ...               % Y coordinates
                 [0.0, 0.0; 0.0, 0.0], ...                 % Z coordinates 
                 'CData', imread('pinktiles.jpg'), ...            
                 'FaceColor', 'texturemap');      
            
            tableRobot = PlaceObject('tableRobot.ply', [0, 1, 0]); 
            tableServe = PlaceObject('tableServe.ply', [0, 0.25, 0]); 
            
            % candyShelf1 = PlaceObject('candyShelf.ply', [1, 1.75, 0]);
            % candyShelf2 = PlaceObject('candyShelf.ply', [-1, 1.75, 0]);
            % candyBox1 = PlaceObject('candyBox.ply', [1, 0.75, 0.5]);
            % candyBox2 = PlaceObject('candyBox.ply', [1, 1, 0.5]);
            % candyBox3 = PlaceObject('candyBox.ply', [1, 1.25, 0.5]);
            creditCardReader = PlaceObject('creditCardReader.ply', [-1.2, 0.1, 0.5]);
            monitor = PlaceObject('monitor.ply', [-1.5, 0.1, 0.5]);

        end
    end
end