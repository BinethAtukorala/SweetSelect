classdef BoxClass
    properties
        Brick_Object  % Handle to the brick's graphical object
        Vertices  % Original vertices of the brick (used for position updates)
    end
    
    methods
        function placeBox()
            Initial_Box_Poses = [
                -1.2, 0.75, 0.5;
                -1.2, 1, 0.5;
                -1.2, 1.25, 0.5;
            ];

            Boxes = [];
            index = 1;
            
            while index <= size(Initial_Box_Poses, 1)
                % Place each box at its initial position
                Placed_Boxes = PlaceObject('box.ply', Initial_Box_Poses(index, :));
                Boxes = [Boxes, Placed_Boxes];
                Vertices = get(Placed_Boxes, 'Vertices') % need to add this to get vertices of box
                index = index + 1;
            end
        end
        
        function updateBoxPosition(obj, Transformation, Vertices)
            % Applies the transformation to the original vertices to compute new positions
            % The transformation consists of both rotation (1:3, 1:3) and translation (1:3, 4)
            New_Vertices = (Transformation(1:3, 1:3) * Vertices' + Transformation(1:3, 4))';
            set(Placed_Boxes, 'Vertices', New_Vertices);
        end
    end
end
