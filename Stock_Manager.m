%% -------- EXPLANATION --------
%
% This is the class that will be responsible to control and manage them
% main workflow and the number of cans in shelfs and their positions
%
% Each shelf as 6 rack with 8 lines each:
%   Matrix of the shelf:
%
% [0,0,0,0,0,0,0,0;
%  0,0,0,0,0,0,0,0;
%  0,0,0,0,0,0,0,0;
%  0,0,0,0,0,0,0,0;
%  0,0,0,0,0,0,0,0]     -> It will be implemented 2 matrix to know the
%                          current number of cans in each shelf and its
%                          position on the shelf
%
% -> The last and first position in each shelf has fewer positions for cans


%% -------- CODE --------

classdef Stock_Manager
    properties
        cans_left_shelf             %control number of cans in the left shelf
        cans_right_shelf            %control number of cans in the right shelf
        conveyor_ctrl               %control conveyor
        storage                     %
        curr_state
        next_state
    end
    
    methods
        %% -------- MAIN FUNCTIONS --------
        function obj = Stock_Manager()
            % Initialize all shelves with 0 cans
            obj.cans_left_shelf = zeros(6,8,6); 
            obj.cans_right_shelf = zeros(6,8,6);
        end
        
        function [available_positions, total_available_spots] = check_availability(obj, shelf_index)
          % Define initial variables
          available_positions = {}; % Cell array to store available positions
          total_available_spots = 0;
        
          if shelf_index == 1 
            cans_on_shelf = obj.cans_left_shelf;
          elseif (shelf_index == 2)
            cans_on_shelf = obj.cans_right_shelf;     
          end
          
          % Iterate through lines and positions on the chosen shelf
          for line = 1:8
            for position = 1:6
              % Determine maximum capacity based on line position
              max_capacity = get_line_capacity(line);
        
              % Check if there's space for a can
              if cans_on_shelf(shelf_index, line, position) < max_capacity
                available_positions{end+1} = [shelf_index, line, position]; % Add position to list
                total_available_spots = total_available_spots + 1;
              end
            end
          end
        end

        function [cv_ctrl, vision_filter] = conveyor_control(obj, sensor_value, en_conv)
            if sensor_value
                cv_ctrl = 1;                %stop Conveyor
                vision_filter = 1;          %call vision function
            elseif en_conv                  %only enable coveyor after
                cv_ctrl = 0;
                vision_filter = 0;
            else
                vision_filter = 0;
            end
        end

        function [next_st] = workflowFSM(obj, current_state,recognized)

            switch current_state
                case 'idle'                                                 %Inital state where everything is initialized
                    next_st = 'start';
                case 'start'                                                %First state responsible start conveyor
                    next_st = 'conveyor';
                case 'conveyor'                                             %Check if conveyor as triggered the proximity sensor (YES)->call vsion and go to next_st (NO)->wait until a can is detected
                    next_st = 'move_to_conveyor';
                case 'move_to_conveyor'                                     %Move KUKA to the position to receive cans from conveyor (if it's in position use arm to pick up a can)
                    next_st = 'store_KUKA';
                case 'store_KUKA'                                           %State where the picked can will be putted in the staging aread in kuka or if the can is not recognized it moves to put the can in the desired place
                    if recognized
                        next_st = 'move-to_shelf';
                    elseif ~recognized
                        next_st = 'not_recognized';
                    end
                case 'move_to_shelf'                                        %After all positions in KUKA are full go to shelfs
                    next_st = 'rack_manager';
                case 'rack_manager'                                         %Check which rack and line is empty and available 
                    next_st = 'store_shelf';
                case 'store_shelf'                                          %Movement to pick a can from the KUKA to the right position in the shelf
                    next_st = 'start';
                case 'not_recognized'                                       %ERROR case a can is not specified place can in table
                    next_st = 'move_to_conveyor';
            end

        end

        %% -------- HELPER FUNCTIONS --------

        function capacity = get_line_capacity(line_index)
          if line_index == 1 || line_index == 8
            capacity = 5;
          else
            capacity = 6;
          end
        end
    end
end
