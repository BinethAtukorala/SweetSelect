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
        
        function updateBoxPosition(obj, Transformation)

            % % Define the rotation matrix for -90 degrees around the x-axis
            % Rotation_X_Neg90 = [1 0 0;
            %                     0 0 -1;
            %                     0 1 0];
            % 
            % % Apply the -90 degree rotation to the transformation matrix
            % New_Transformation = Transformation;
            % New_Transformation(1:3, 1:3) = Rotation_X_Neg90 * Transformation(1:3, 1:3);

                % Define the rotation matrix for 90 degrees around the box's x-axis
            Rotation_X_90 = [1  0  0;
                             0  0 -1;
                             0  1  0];
        
            % Compute the centroid of the box (mean of all vertices)
            centroid = mean(obj.Vertices, 1);
            
            % Translate vertices to the origin (relative to centroid)
            Translated_Vertices = obj.Vertices - centroid;
            
            % Apply the 90-degree x-axis rotation to the translated vertices
            Rotated_Vertices = (Rotation_X_90 * Translated_Vertices')';
            
            % Translate the vertices back to their original position
            Rotated_Vertices = Rotated_Vertices + centroid;

                % Now apply the overall transformation, including translation from the input transformation
            New_Vertices = (Transformation(1:3, 1:3) * Rotated_Vertices' + Transformation(1:3, 4))';
        
            % Update the graphical object's vertices
            set(obj.Box_Object, 'Vertices', New_Vertices);

            % Applies the transformation to the original vertices to compute new positions
            % The transformation consists of both rotation (1:3, 1:3) and translation (1:3, 4)
            % New_Vertices = (Transformation(1:3, 1:3) * obj.Vertices' + Transformation(1:3, 4))';
            % set(obj.Box_Object, 'Vertices', New_Vertices);
        end
    end
end
