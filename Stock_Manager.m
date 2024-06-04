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
        storage                     %kuka storage 9 cans
        curr_can
    end
    
    methods
        %% -------- MAIN FUNCTIONS --------
        function obj = Stock_Manager()
            % Initialize all shelves with 0 cans
            obj.cans_left_shelf = zeros(6,8,6); 
            obj.cans_right_shelf = zeros(6,8,6);
            obj.storage = zeros(9,1);           % type 1 -> mushrooms 2 -> sausages
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
        
        
         
        function [num_available_spots, position] = set_can_storage(obj, type)
            % Sets a can in the storage area and returns available spots and position

            % Find available positions in the storage area
            available_positions = find(obj.storage == 0);

            % Determine the starting position based on the type of can
            if rem(type, 2) == 0  % Even type numbers start at position 9
                start_position = 9;
            else
                start_position = 1; % Odd type numbers start at position 1
            end

            % Filter available positions to start positions based on type
            start_positions = available_positions(start_position:9:end);

            % If there are available start positions, set the can in the first one
            if ~isempty(start_positions)
                position = start_positions(1);
                obj.storage(position) = type;
            else
                error('No available space in the storage area.');
            end
            
            % Calculate number of available spots after placing the can
            num_available_spots = length(available_positions) - length(start_positions) + 1;
        end
        
        function storage_info = get_storage_info(obj)
            % Returns information about cans in storage (type, position, quantity)

            storage_info = struct('type', {}, 'position', {}, 'quantity', {});

            % Find unique types of cans in storage
            unique_types = unique(obj.storage(obj.storage ~= 0));

            % Iterate over unique types and gather information
            for i = 1:length(unique_types)
                type = unique_types(i);
                positions = find(obj.storage == type);
                quantity = length(positions);
                storage_info(i).type = type;
                storage_info(i).position = positions;
                storage_info(i).quantity = quantity;
            end
        end
        
        function set_can_on_shelf(obj, shelf_index, position, type)
            % Sets a can on the shelf at a certain position if it's available
            if obj.cans_left_shelf(shelf_index, position(1), position(2)) < get_line_capacity(position(1))
                obj.cans_left_shelf(shelf_index, position(1), position(2)) = type;
            elseif obj.cans_right_shelf(shelf_index, position(1), position(2)) < get_line_capacity(position(1))
                obj.cans_right_shelf(shelf_index, position(1), position(2)) = type;
            else
                error('No available space on the shelf at the specified position.');
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
