classdef CandyClass
    properties
        Candy_Object  % Handle to the brick's graphical object
        Vertices  % Original vertices of the brick (used for position updates)
    end
    
    methods
        function obj = CandyClass(Pose, Flavour)

            if Flavour == "Raspberry"
                obj.Candy_Object = PlaceObject('candyballraspberry.ply', Pose);
            elseif Flavour == "Blueberry"
                obj.Candy_Object = PlaceObject('candyballblueberry.ply', Pose);
            elseif Flavour == "Greenapple"
                obj.Candy_Object = PlaceObject('candyballgreenapple.ply', Pose);
            end

            % Stores the vertices of the brick for future transformations
            obj.Vertices = get(obj.Candy_Object, 'Vertices');
        end
        
        function updateCandyPosition(obj, Transformation)
            New_Vertices = (Transformation(1:3, 1:3) * obj.Vertices' + Transformation(1:3, 4))';
            set(obj.Candy_Object, 'Vertices', New_Vertices);
            % New_Vertices = (Transformation(1:3, 1:3) * obj.Vertices' + Transformation(1:3, 4))';
            % set(obj.Candy_Object, 'Vertices', New_Vertices);
            % Gotta edit this so that it checks what the moving candy is, and then changes that candys vertices
            % For example the "Placed_Candy" above should be either Placed_Green_Apple_Candy, etc
        end
    end
end
