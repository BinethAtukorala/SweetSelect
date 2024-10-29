classdef MainProgram < handle
    properties
        r % Property to store the robot object model
        Raspberry_Pose % Poses for raspberry
        Blueberry_Pose % Poses for blueberry
        Greenapple_Pose % Poses for green apple
        Environment
        Box_Gripper
        Candy_Gripper
        LBRiiwa
        UR3e
        Front_Box_Poses
        Back_Box_Poses
        E_Stop % Track whether emergency stop is engaged
        Run_Status % Running status of the robots
    end

    methods
        %% Constructor to initialize the robot model
        function obj = MainProgram()
            hold on
            xlim([-2, 2]);                                              
            ylim([-2, 2]);
            zlim([0, 2.5]);
            
            addpath('C:\Data\SweetSelect\Environment')
            addpath('C:\Data\SweetSelect\LBRiiwa')
            addpath('C:\Data\SweetSelect\UR3e')
            addpath('C:\Data\SweetSelect\GripperBox')
            addpath('C:\Data\SweetSelect\GripperCandy')
            
            % Create an instance of the EnvironmentClass, representing the environment
            obj.Environment = EnvironmentClass();

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
            
            

        end

        function putCandiesInBox(obj, Raspberry_Count, Blueberry_Count, Greenapple_Count)
            Candy_Initial_Poses = [
                repmat(obj.Raspberry_Pose, Raspberry_Count, 1);  
                repmat(obj.Greenapple_Pose, Blueberry_Count, 1);  
                repmat(obj.Blueberry_Pose, Greenapple_Count, 1)   
            ];

            % Initialize an empty array for the Candy objects
            Candies = [];
            % Loop through each candy pose and assign the correct flavor
            for i = 1:size(Candy_Initial_Poses, 1)
                if i <= Raspberry_Count
                    % For Raspberry candies
                    Candies = [Candies, CandyClass(Candy_Initial_Poses(i, :), "Raspberry")];
                elseif i <= Raspberry_Count + Blueberry_Count
                    % For Blueberry candies
                    Candies = [Candies, CandyClass(Candy_Initial_Poses(i, :), "Blueberry")];
                else
                    % For Greenapple candies
                    Candies = [Candies, CandyClass(Candy_Initial_Poses(i, :), "Greenapple")];
                end
            end


            
            Candy_Final_Poses = [                                         
                0.48, 1, 0.95;
                0.54, 1, 0.95;
                0.60, 1, 0.95;
            ];
            
            Candy_Final_Poses(:,3) = Candy_Final_Poses(:,3) + 0.08;

            % Calculate how many boxes are needed
            Num_Boxes = ceil((Raspberry_Count + Blueberry_Count + Greenapple_Count)/ 3);
            
            Boxes = [];
            Box_Initial_Poses = [];
            
            % If less than or equal to 6 boxes are needed, select from the front
            if Num_Boxes <= 6
                for i = 1:Num_Boxes
                    Boxes = [Boxes, BoxClass(obj.Front_Box_Poses(i, :))];
                    Box_Initial_Poses = [Box_Initial_Poses; obj.Front_Box_Poses(i, :)];
                end
            % If more than 2 boxes are needed, take 2 from the front and the rest from the back
            elseif Num_Boxes > 6
                % If more than 2 boxes are needed, take 2 from the front and the rest from the back
                for a = 1:6
                    Boxes = [Boxes, BoxClass(obj.Front_Box_Poses(a, :))];
                    Box_Initial_Poses = [Box_Initial_Poses; obj.Front_Box_Poses(a, :)];
                end
            
                for b = 1:(Num_Boxes - 6)
                    Boxes = [Boxes, BoxClass(obj.Back_Box_Poses(b, :))];
                    Box_Initial_Poses = [Box_Initial_Poses; obj.Back_Box_Poses(b, :)];
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

            %% Movement

            % Get the total number of candies
            Num_Candies = size(Candy_Initial_Poses, 1);
            
            % Iterate through the boxes
            for Box_Index = 1:Num_Boxes
                % Determine the candy indices for this box
                Start_Index = (Box_Index - 1) * 3 + 1;
                End_Index = min(Box_Index * 3, Num_Candies);
            
                if Box_Index <= 2
            
                    [Box_Start_Pose] = obj.LBRiiwa.moveFrontToMidway( Box_Initial_Poses(Box_Index,:), Boxes(Box_Index));
                    Candy_Start_Poses = [];
            
                    for x = Start_Index:End_Index
                        [Candy_Start_Pose] = obj.UR3e.moveUR3e( Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x), x);
                        Candy_Start_Poses = [Candy_Start_Poses; Candy_Start_Pose];
                    end
            
                    obj.LBRiiwa.moveFrontFromMidway( Boxes(Box_Index), Box_Index, Candies(Start_Index:End_Index), Box_Start_Pose, Candy_Start_Poses);
            
                else
            
                    [Box_Start_Pose] = obj.LBRiiwa.moveBackToMidway( Box_Initial_Poses(Box_Index,:), Boxes(Box_Index));
                    Candy_Start_Poses = [];
            
                    for x = Start_Index:End_Index
                        [Candy_Start_Pose] = obj.UR3e.moveUR3e( Candy_Initial_Poses(x,:), Candy_Final_Poses, Candies(x), x);
                        Candy_Start_Poses = [Candy_Start_Poses; Candy_Start_Pose];
                    end
                    obj.LBRiiwa.moveBackFromMidway( Boxes(Box_Index), Box_Index, Candies(Start_Index:End_Index), Box_Start_Pose, Candy_Start_Poses);
            
                end
            end
        end

        function result = getQValues(obj, robot)
            if robot == "LBRiiwa"
                result = obj.LBRiiwa.LBRiiwa.model.getpos();
            end
            if robot == "UR3e"
                result = obj.UR3e.UR3e.model.getpos();
            end
        end

        function result = getQLims(obj, robot)
            if robot == "LBRiiwa"
                result = obj.LBRiiwa.LBRiiwa.model.qlim;
            end
            if robot == "UR3e"
                result = obj.UR3e.UR3e.model.qlim;
            end
        end

        function setQValues(obj, q, robot)
            if robot == "LBRiiwa"
                obj.LBRiiwa.moveWithoutBox(q);
            end
            if robot == "UR3e"
                obj.UR3e.moveWithoutCandy(q);
            end
        end

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

        function e_stop_status = pressEStop(obj)
            if obj.E_Stop
                obj.E_Stop = false;
                obj.UR3e.disengageEStop();
                obj.LBRiiwa.disengageEStop();
            else
                obj.E_Stop = true;
                obj.Run_Status = false;

                obj.UR3e.engageEStop();
                obj.LBRiiwa.engageEStop();
            end
            e_stop_status = obj.E_Stop;
            
        end

        function run_status = resumeOperation(obj)
            if obj.E_Stop == false
                obj.Run_Status = true;
            end
            run_status = obj.Run_Status;
        end

    end

end
