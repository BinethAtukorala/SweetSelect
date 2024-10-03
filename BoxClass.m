classdef BoxClass
    properties
        Box_Object  % Handle to the brick's graphical object
        Vertices  % Original vertices of the brick (used for position updates)
    end
    
    methods
        function obj = BoxClass(Pose)
            obj.Box_Object = PlaceObject('box.ply', Pose);

            % Stores the vertices of the brick for future transformations
            obj.Vertices = get(obj.Box_Object, 'Vertices');
        end
        
        function updateBoxPosition(obj, Transformation, Vertices)
            % Applies the transformation to the original vertices to compute new positions
            % The transformation consists of both rotation (1:3, 1:3) and translation (1:3, 4)
            New_Vertices = (Transformation(1:3, 1:3) * Vertices' + Transformation(1:3, 4))';
            set(obj.Box_Object, 'Vertices', New_Vertices);
        end
    end
end
