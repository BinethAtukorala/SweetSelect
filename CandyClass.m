classdef CandyClass
    properties
        Brick_Object  % Handle to the brick's graphical object
        Vertices  % Original vertices of the brick (used for position updates)
    end
    
    methods
        function placeCandy()
            Raspberry_Candy_Initial_Poses = [                                         
            0.95, 0.7, 0.5;
            1.1, 0.7, 0.5;
            0.95, 0.8, 0.5;
            1.1, 0.8, 0.5;
            1.025, 0.75, 0.5;
            ];

            Blueberry_Candy_Initial_Poses = [                                         
                0.95, 0.95, 0.5;
                1.1, 0.95, 0.5;
                0.95, 1.05, 0.5;
                1.1, 1.05, 0.5;
                1.025, 1, 0.5;
            ];

            Green_Apple_Candy_Initial_Poses = [                                         
                0.95, 1.2, 0.5;
                1.1, 1.2, 0.5;
                0.95, 1.3, 0.5;
                1.1, 1.3, 0.5;
                1.025, 1.25, 0.5
            ];

            Raspberry_Candy = [];
            Raspberry_Candy_Index = 1;
            
            while index <= size(Raspberry_Candy_Initial_Poses, 1)
                % Place each box at its initial position
                Placed_Raspberry_Candy = PlaceObject('candyballraspberry.ply', Raspberry_Candy_Initial_Poses(index, :));
                Raspberry_Candy = [Raspberry_Candy, Placed_Raspberry_Candy];
                Vertices = get(Placed_Raspberry_Candy, 'Vertices') % need to add this to get vertices of candy
                Raspberry_Candy_Index = Raspberry_Candy_Index + 1;
            end

            Blueberry_Candy = [];
            Blueberry_Candy_Index = 1;
            
            while index <= size(Raspberry_Candy_Initial_Poses, 1)
                % Place each box at its initial position
                Placed_Blueberry_Candy = PlaceObject('candyballblueberry.ply', Blueberry_Candy_Initial_Poses(index, :));
                Blueberry_Candy = [Blueberry_Candy, Placed_Blueberry_Candy];
                Vertices = get(Placed_Blueberry_Candy, 'Vertices') % need to add this to get vertices of candy
                Blueberry_Candy_Index = Blueberry_Candy_Index + 1;
            end

            Green_Apple_Candy = [];
            Green_Apple_Candy_Index = 1;
            
            while index <= size(Raspberry_Candy_Initial_Poses, 1)
                % Place each box at its initial position
                Placed_Green_Apple_Candy = PlaceObject('candyballgreenapple.ply', Green_Apple_Candy_Initial_Poses(index, :));
                Green_Apple_Candy = [Green_Apple_Candy, Placed_Green_Apple_Candy];
                Vertices = get(Placed_Green_Apple_Candy, 'Vertices') % need to add this to get vertices of candy
                Green_Apple_Candy_Index = Green_Apple_Candy_Index + 1;
            end
        end
        
        function updateCandyPosition(obj, Transformation, Vertices)
            New_Vertices = (Transformation(1:3, 1:3) * Vertices' + Transformation(1:3, 4))';
            set(Placed_Candy, 'Vertices', New_Vertices);
            % Gotta edit this so that it checks what the moving candy is, and then changes that candys vertices
            % For example the "Placed_Candy" above should be either Placed_Green_Apple_Candy, etc
        end
    end
end
