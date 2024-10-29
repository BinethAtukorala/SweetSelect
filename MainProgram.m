classdef MainProgram < handle

    properties
        r % Property to store the robot object model
        Raspberry_Pose % Poses for raspberry
        Blueberry_Pose % Poses for blueberry
        Greenapple_Pose % Poses for green apple
        Environment % Environment data
        Box_Gripper % Gripper used to grab the box
        Candy_Gripper % Gripper used to grab candy
        LBRiiwa
        UR3e
        Candies % Candy class objects
        Boxes % Box class objects

        Front_Box_Poses % Poses of the boxes at the front
        Back_Box_Poses % Poses of the boxes at the back
        Candy_Initial_Poses % Initial poses of the candy
        Candy_Final_Poses % Final poses of the candies
        Box_Initial_Poses % Initial poses of the boxes

        E_Stop % Track whether emergency stop is engaged
        Run_Status % Running status of the robots
        Resume_Box_Index % The box being held when resuming
        Resume_Candy_Index % The candy being held when resuming
        Resume_Work_Code % Code of the current operation being run when resuming

        Obstacles   % Track obstacles in the environment that might come in contact 
                    % with the light curtain (Using spheres around the objects)
        Light_Curtain_Pos
    end

    methods
        %% Constructor to initialize the robot model
        function obj = MainProgram(Light_Curtain_Pos)

            hold on
            xlim([-2, 2]);                                              
            ylim([-1.5, 2.5]);
            zlim([0, 2.5]);
            
            addpath('C:\Data\SweetSelect\Environment')
            addpath('C:\Data\SweetSelect\LBRiiwa')
            addpath('C:\Data\SweetSelect\UR3e')
            addpath('C:\Data\SweetSelect\GripperBox')
            addpath('C:\Data\SweetSelect\GripperCandy')
            
            % Create an instance of the EnvironmentClass, representing the environment
            obj.Environment = EnvironmentClass();

            obj.Resume_Box_Index = 0;
            obj.Resume_Candy_Index = 0;
            obj.Resume_Work_Code = 0;

            obj.Light_Curtain_Pos = Light_Curtain_Pos;
            obj.Obstacles = [];

            % Set E_Stop disengaged and Run_Status as running
            obj.E_Stop = false;
            obj.Run_Status = true;
            
            % Create an instance of the GripperBox class, representing the box gripper for handling boxes
            obj.Box_Gripper = GripperBoxClass();
            
            % Create an instance of the GripperBox class, representing the candy gripper for handling candies
            obj.Candy_Gripper = GripperCandyClass();
            
            % Create an instance of the LBRiiwaClass, representing the LBRiiwa model
            obj.LBRiiwa = LBRiiwaClass(obj.Box_Gripper);
            
            % Create an instance of the UR3Class, representing the UR3 model
            obj.UR3e = UR3eClass(obj.Candy_Gripper);
            
            % Keep grippers open initially
            obj.Box_Gripper.openGripper();
            obj.Candy_Gripper.openGripper();
            
            %% Candy
            obj.Raspberry_Pose = [0.5, 1.6, 0.9]; 
            obj.Blueberry_Pose = [0.66, 1.6, 0.9];  
            obj.Greenapple_Pose = [0.82, 1.6, 0.9];   

            %% Boxes
            obj.Front_Box_Poses = [
                -1.15, 0.9, 0.8;
                -1.15, 0.9, 0.76;
                -1.15, 0.9, 0.72;
                -1.15, 0.9, 0.68;
                -1.15, 0.9, 0.64;
                -1.15, 0.9, 0.6;
            ];
            
            obj.Back_Box_Poses = [
                -1.15, 1.1, 0.8;
                -1.15, 1.1, 0.76;
                -1.15, 1.1, 0.72;
                -1.15, 1.1, 0.68;
                -1.15, 1.1, 0.64;
                -1.15, 1.1, 0.6;
            ];
            
            threshold = 0.5; 
            % Obtain the limits of the axes
            %zp = get(gca,'Zlim');
            zp = [0.5 2.5];
            xp = get(gca,'Xlim');
            % Use the axes x and Y limits to find the co-ordinates for the patch
            x1 = [ xp(1) xp(2) xp(2) xp(1)];
            z1 = [ zp(1) zp(1) zp(2) zp(2)];y1 = ones(1,numel(z1))* threshold; 
            v = patch(x1,y1,z1, 'g');
            set(v,'facealpha',0.2);
            set(v,'edgealpha',0.2);
            set(gcf,'renderer','opengl') ;

        end

        function putCandiesInBox(obj, Raspberry_Count, Greenapple_Count, Blueberry_Count)
            % Calculate how many boxes are needed
            Num_Boxes = ceil((Raspberry_Count + Blueberry_Count + Greenapple_Count)/ 3);
            
            if obj.Resume_Work_Code == 0
                obj.Candy_Initial_Poses = [
                    repmat(obj.Raspberry_Pose, Raspberry_Count, 1);  
                    repmat(obj.Greenapple_Pose, Greenapple_Count, 1);  
                    repmat(obj.Blueberry_Pose, Blueberry_Count, 1)   
                ];
    
                % Initialize an empty array for the Candy objects
                obj.Candies = [];
                % Loop through each candy pose and assign the correct flavor
                for i = 1:size(obj.Candy_Initial_Poses, 1)
                    if i <= Raspberry_Count
                        % For Raspberry candies
                        obj.Candies = [obj.Candies, CandyClass(obj.Candy_Initial_Poses(i, :), "Raspberry")];
                    elseif i <= Raspberry_Count + Blueberry_Count
                        % For Blueberry candies
                        obj.Candies = [obj.Candies, CandyClass(obj.Candy_Initial_Poses(i, :), "Blueberry")];
                    else
                        % For Greenapple candies
                        obj.Candies = [obj.Candies, CandyClass(obj.Candy_Initial_Poses(i, :), "Greenapple")];
                    end
                end

                obj.Candy_Final_Poses = [                                         
                    0.48, 1, 0.95;
                    0.54, 1, 0.95;
                    0.60, 1, 0.95;
                ];
                
                obj.Candy_Final_Poses(:,3) = obj.Candy_Final_Poses(:,3) + 0.08;
    
                
                
                obj.Boxes = [];
                obj.Box_Initial_Poses = [];
                
                % If less than or equal to 6 boxes are needed, select from the front
                if Num_Boxes <= 6
                    for i = 1:Num_Boxes
                        obj.Boxes = [obj.Boxes, BoxClass(obj.Front_Box_Poses(i, :))];
                        obj.Box_Initial_Poses = [obj.Box_Initial_Poses; obj.Front_Box_Poses(i, :)];
                    end
                % If more than 2 boxes are needed, take 2 from the front and the rest from the back
                elseif Num_Boxes > 6
                    % If more than 2 boxes are needed, take 2 from the front and the rest from the back
                    for a = 1:6
                        obj.Boxes = [obj.Boxes, BoxClass(obj.Front_Box_Poses(a, :))];
                        obj.Box_Initial_Poses = [obj.Box_Initial_Poses; obj.Front_Box_Poses(a, :)];
                    end
                
                    for b = 1:(Num_Boxes - 6)
                        obj.Boxes = [obj.Boxes, BoxClass(obj.Back_Box_Poses(b, :))];
                        obj.Box_Initial_Poses = [obj.Box_Initial_Poses; obj.Back_Box_Poses(b, :)];
                    end
                end
                
                % If all boxes are selected, skip the unselected box placement
                if Num_Boxes < size(obj.Front_Box_Poses, 1)
                    for i = Num_Boxes + 1:size(obj.Front_Box_Poses, 1)
                        PlaceObject('box.ply', obj.Front_Box_Poses(i, :));
                    end
                
                    for i = 1:size(obj.Back_Box_Poses, 1)
                        PlaceObject('box.ply', obj.Back_Box_Poses(i, :));
                    end
                end
                
                if Num_Boxes >= 6 && (Num_Boxes - 6) < size(obj.Back_Box_Poses, 1)
                    for i = Num_Boxes - 6 + 1:size(obj.Back_Box_Poses, 1)
                        PlaceObject('box.ply', obj.Back_Box_Poses(i, :));
                    end
                end
            else
                disp("Resuming")
            end
               

            %% Movement

            % Get the total number of candies
            Num_Candies = size(obj.Candy_Initial_Poses, 1);

            Start_Box_Index = 1;
            if obj.Resume_Work_Code
                Start_Box_Index = obj.Resume_Box_Index;
            end
            
            % Iterate through the boxes
            for Box_Index = Start_Box_Index:Num_Boxes
                obj.Resume_Box_Index = Box_Index;
                % Determine the candy indices for this box
                Start_Index = (Box_Index - 1) * 3 + 1;
                if obj.Resume_Work_Code
                    Start_Index = obj.Resume_Box_Index;
                end
                End_Index = min(Box_Index * 3, Num_Candies);
            
                % If taking boxes from the front
                if Box_Index <= 6
                    % 1st Movement (Grab boxes and bring near UR3e) ===========================
                    if obj.Resume_Work_Code
                        % Make it so that the end effector grips the box from the side
                        Box_Start_Pose = [eye(3), obj.Box_Initial_Poses(Box_Index,:)'; 0, 0, 0, 1] * troty(-pi/2);
                        Box_Start_Pose(1,4) = Box_Start_Pose(1,4) + 0.18;
                        Box_Start_Pose(3,4) = Box_Start_Pose(3,4) + 0.05;
                        if obj.Resume_Work_Code == 1
                            obj.LBRiiwa.executeQueue();
                        end
                    else
                        obj.Resume_Work_Code = 1;
                        [Box_Start_Pose] = obj.LBRiiwa.moveFrontToMidway( obj.Box_Initial_Poses(Box_Index,:), obj.Boxes(Box_Index));
                    end

                    if obj.Run_Status == false
                        return
                    end
                    % ===========================
                    
                    Candy_Start_Poses = [];
                    
                    % 2nd Movemement (Place candies on the box) ===========================
                    for x = Start_Index:End_Index
                        obj.Resume_Candy_Index = x;
                        if obj.Resume_Work_Code >= 2
                            % Calculate start pose for each candy
                            Candy_Start_Pose = [eye(3), obj.Candy_Initial_Poses(x,:)'; 0, 0, 0, 1] * trotx(pi);
                            Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.09;  % Adjust height

                            if obj.Resume_Work_Code == 2
                                obj.UR3e.executeQueue();
                            end
                        else
                            obj.Resume_Work_Code = 2;
                            [Candy_Start_Pose] = obj.UR3e.moveUR3e( obj.Candy_Initial_Poses(x,:), obj.Candy_Final_Poses, obj.Candies(x), x);
                        end
                        if obj.Run_Status == false
                            return
                        end

                        Candy_Start_Poses = [Candy_Start_Poses; Candy_Start_Pose];
                    end
                    % ===========================

                    % 3rd Movement (Present box to customer) ===========================
                    if obj.Resume_Work_Code == 3
                        obj.LBRiiwa.executeQueue();
                    else
                        obj.Resume_Work_Code = 3;
                        obj.LBRiiwa.moveFrontFromMidway( obj.Boxes(Box_Index), Box_Index, obj.Candies(Start_Index:End_Index), Box_Start_Pose, Candy_Start_Poses);
                    end
                    if obj.Run_Status == false
                        return
                    end
                    % ===========================
                
                % If taking boxes from the back
                else
                    % 1st Movement (Grab boxes and bring near UR3e) ===========================
                    if obj.Resume_Work_Code
                        % Make it so that the end effector grips the box from the side
                        Box_Start_Pose = [eye(3), obj.Box_Initial_Poses(Box_Index,:)'; 0, 0, 0, 1] * troty(-pi/2);
                        Box_Start_Pose(1,4) = Box_Start_Pose(1,4) + 0.18;
                        Box_Start_Pose(3,4) = Box_Start_Pose(3,4) + 0.05;
                        if obj.Resume_Work_Code == 1
                            obj.LBRiiwa.executeQueue();
                        end
                    else
                        obj.Resume_Work_Code = 1;
                        [Box_Start_Pose] = obj.LBRiiwa.moveBackToMidway( obj.Box_Initial_Poses(Box_Index,:), obj.Boxes(Box_Index));
                    end

                    if obj.Run_Status == false
                        return 
                    end
                    % ===========================

                    Candy_Start_Poses = [];
            
                    % 2nd Movemement (Place candies on the box) ===========================
                    for x = Start_Index:End_Index
                        if obj.Resume_Work_Code >= 2
                            % Calculate start pose for each candy
                            Candy_Start_Pose = [eye(3), obj.Candy_Initial_Poses(x,:)'; 0, 0, 0, 1] * trotx(pi);
                            Candy_Start_Pose(3,4) = Candy_Start_Pose(3,4) + 0.09;  % Adjust height

                            if obj.Resume_Work_Code == 2
                                obj.UR3e.executeQueue();
                            end
                        else
                            obj.Resume_Work_Code = 2;
                            [Candy_Start_Pose] = obj.UR3e.moveUR3e( obj.Candy_Initial_Poses(x,:), obj.Candy_Final_Poses, obj.Candies(x), x);
                        end

                        if obj.Run_Status == false
                            return
                        end
                        Candy_Start_Poses = [Candy_Start_Poses; Candy_Start_Pose];
                    end
                    % ===========================

                    % 3rd Movement (Present box to customer) ===========================
                    if obj.Resume_Work_Code
                        obj.LBRiiwa.executeQueue();
                    else
                        obj.Resume_Work_Code = 3;
                        obj.LBRiiwa.moveBackFromMidway( obj.Boxes(Box_Index), Box_Index, obj.Candies(Start_Index:End_Index), Box_Start_Pose, Candy_Start_Poses);
                    end
                    if obj.Run_Status == false
                        return
                    end
                    % ===========================
            
                end

                % Finish movememnt by going to reset position
                obj.Resume_Work_Code = 0;
                obj.Resume_Box_Index = 0;
                obj.Resume_Candy_Index = 0;
            end
            
        end
        
        % Get joint angles for the robot specified
        function result = getQValues(obj, robot)
            if robot == "LBRiiwa"
                result = obj.LBRiiwa.LBRiiwa.model.getpos();
            end
            if robot == "UR3e"
                result = obj.UR3e.UR3e.model.getpos();
            end
        end

        % Get joint angle limits for the robot specified
        function result = getQLims(obj, robot)
            if robot == "LBRiiwa"
                result = obj.LBRiiwa.LBRiiwa.model.qlim;
            end
            if robot == "UR3e"
                result = obj.UR3e.UR3e.model.qlim;
            end
        end

        % Set joint angles of the robot specified
        function setQValues(obj, Q, Robot)
            while Robot == "LBRiiwa"
                obj.LBRiiwa.moveWithTeach(Q);
            end
            while Robot == "UR3e"
                obj.UR3e.moveWithTeach(Q);
            end
        end

        % Get transformation and RPY data of the robot specified
        function [tr, rpy] = getTrAndRPY(obj, q, robot)
            tr = [0 0 0];
            rpy = [0 0 0];
            if robot == "LBRiiwa"
                tr = obj.LBRiiwa.LBRiiwa.model.fkine(q);
                rpy = tr2rpy(tr);
                tr = tr.t;
            end
            if robot == "UR3e"
                tr = obj.UR3e.UR3e.model.fkine(q);
                rpy = tr2rpy(tr);
                tr = tr.t;
            end
            
        end

        % When eStop button is pressed
        function e_stop_status = pressEStop(obj)
            % disengage estop if currently engaged
            if obj.E_Stop
                obj.E_Stop = false;
                obj.UR3e.disengageEStop();
                obj.LBRiiwa.disengageEStop();
            % engage estop if not engaged
            else
                obj.E_Stop = true;
                obj.Run_Status = false;

                obj.UR3e.engageEStop();
                obj.LBRiiwa.engageEStop();
            end
            e_stop_status = obj.E_Stop;
            
        end
    
        % Engage EStop
        function e_stop_status = engageEStop(obj)

            obj.E_Stop = true;
            obj.Run_Status = false;

            obj.UR3e.engageEStop();
            obj.LBRiiwa.engageEStop();

            e_stop_status = obj.E_Stop;
            
        end

        % resume opertation of the robots
        function run_status = resumeOperation(obj)
            if obj.E_Stop == false
                obj.Run_Status = true;
                obj.UR3e.resumeOperation();
                obj.LBRiiwa.resumeOperation();
            end
            run_status = obj.Run_Status;
        end

        % ----------- Light curtain -----------

        % Add obstacle to be tracked by the light curtain
        function index = addObstacle(obj, Center_Point, Radius)
            obj.Obstacles = [obj.Obstacles; Center_Point Radius]
            index = size(obj.Obstacles, 1);

            obj.checkLightCurtainCollision();
        end

        % Update obstacle center point
        function updateObstacle(obj, index, Center_Point)
            obstacle_data = [Center_Point obj.Obstacles(index, 3)];
            obj.Obstacles(1, :) = obstacle_data;

            obj.checkLightCurtainCollision();
        end

        % Check collision
        function isCollision = checkLightCurtainCollision(obj)
            isCollision = false;
            for i = 1:size(obj.Obstacles, 1)
                if abs(obj.Light_Curtain_Pos - obj.Obstacles(i, 2)) <= obj.Obstacles(i, 4)
                    isCollision = true;
                end
            end
        end

    end

end