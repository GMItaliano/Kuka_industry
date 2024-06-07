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
        TARGET_Number
    end
    
    methods
        %% -------- MAIN FUNCTIONS --------
        function obj = Stock_Manager()
            % Initialize all shelves with 0 cans
            obj.cans_left_shelf = zeros(6, 8);
            obj.cans_right_shelf = zeros(6, 8);
            obj.storage = zeros(18,1);           % type 1 -> mushrooms 2 -> sausages
            obj.TARGET_Number = 96;
        end

        
        %% STORAGE MANAGER
         
        function [num_available_spots, position] = set_can_storage(obj, type)
            % Sets a can in the storage area and returns available spots and position
        
            % Find available positions in the storage area
            available_positions = find(obj.storage == 0);
        
            % Determine the starting position based on the type of can
            if rem(type, 2) == 0  % Even type numbers start at position 9
                start_position = 18;
            else
                start_position = 1; % Odd type numbers start at position 1
            end
        
            % Filter available positions to start positions based on type
            start_positions = start_position:9:length(obj.storage);
            available_start_positions = intersect(available_positions, start_positions);
        
            % If there are available start positions, set the can in the first one
            if ~isempty(available_start_positions)
                position = available_start_positions(1);
                obj.storage(position) = type;
            else
                error('No available space in the storage area.');
            end
        
            % Calculate number of available spots after placing the can
            num_available_spots = length(find(obj.storage == 0));
        end

        
        function [storage_info, total_cans] = get_storage_info(obj)
            % Returns information about cans in storage (type, position, quantity)

            storage_info = struct('type', {}, 'position', {}, 'quantity', {});
            total_cans = 0;
            
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
                total_cans = total_cans + quantity;
            end

        end
        
        function [error, position, num_remaining] = remove_last_can_storage(obj, type)
            % Removes the last can of the specified type from storage
            % and returns its type, position, and remaining count
        
            % Find positions of cans of the specified type
            positions = find(obj.storage == type);
            if isempty(positions)
                fprintf('No cans of the specified type in storage\n');
                error = 0;
            else
                error = 1;
            end
        
            % Get the last position of the specified type
            position = positions(end);
            obj.storage(position) = 0;
            
            % Calculate the number of remaining cans of the specified type
            num_remaining = sum(obj.storage == type);
        end

        %% SHELVES MANAGER

        function [type, position, num_remaining] = remove_last_can_shelf(obj, type)
            % Removes the last can of the specified type from the shelf (left or right)
            % and returns its type, position, and remaining count
        
            % Find positions of cans of the specified type in both shelves
            positions_left = find(obj.cans_left_shelf == type);
            positions_right = find(obj.cans_right_shelf == type);
        
            % Determine the last position of the specified type across both shelves
            if isempty(positions_left) && isempty(positions_right)
                error('No cans of the specified type in the shelves.');
            elseif isempty(positions_right)
                position = positions_left(end);
                shelf_side = 'left';
            elseif isempty(positions_left)
                position = positions_right(end);
                shelf_side = 'right';
            else
                if positions_left(end) > positions_right(end)
                    position = positions_left(end);
                    shelf_side = 'left';
                else
                    position = positions_right(end);
                    shelf_side = 'right';
                end
            end
        
            % Remove the can from the specified position
            if strcmp(shelf_side, 'left')
                obj.cans_left_shelf(position) = 0;
            else
                obj.cans_right_shelf(position) = 0;
            end
        
            % Calculate the number of remaining cans of the specified type
            num_remaining_left = sum(obj.cans_left_shelf == type);
            num_remaining_right = sum(obj.cans_right_shelf == type);
            num_remaining = num_remaining_left + num_remaining_right;
        
            % Adjust position to the 1-96 range
            if strcmp(shelf_side, 'left')
                position = position; % Left shelf position is in the range 1-48
            else
                position = position + 48; % Right shelf position is in the range 49-96
            end
        end
        

        function [position, num_remaining] = set_can_shelf(obj, type)
            % Adds a can of the specified type to the shelf (left or right)
            % and returns its position and the number of cans remaining that can be added
            
            % Check for available positions on both shelves
            available_positions_left = find(obj.cans_left_shelf == 0, 1, 'first');
            available_positions_right = find(obj.cans_right_shelf == 0, 1, 'first');
            
            if isempty(available_positions_left) && isempty(available_positions_right)
                error('No available positions on the shelves.');
            elseif isempty(available_positions_right)
                position = available_positions_left;
                shelf_side = 'left';
            elseif isempty(available_positions_left)
                position = available_positions_right;
                shelf_side = 'right';
            else
                if available_positions_left(1) < available_positions_right(1)
                    position = available_positions_left(1);
                    shelf_side = 'left';
                else
                    position = available_positions_right(1);
                    shelf_side = 'right';
                end
            end
            
            % Place the can at the identified position
            if strcmp(shelf_side, 'left')
                obj.cans_left_shelf(position) = type;
            else
                obj.cans_right_shelf(position) = type;
            end
            
            % Calculate the number of remaining cans that can be added
            remaining_positions_left = sum(obj.cans_left_shelf == 0);
            remaining_positions_right = sum(obj.cans_right_shelf == 0);
            num_remaining = remaining_positions_left + remaining_positions_right;
            
            % Adjust position to the 1-96 range
            if strcmp(shelf_side, 'left')
                position = position; % Left shelf position is in the range 1-48
            else
                position = position + 48; % Right shelf position is in the range 49-96
            end
        end

        

        function shelf_info = get_shelf_info(obj, shelf_index)
            % Returns information about cans on the shelf (type, position, quantity)
            if shelf_index == 1
                cans_on_shelf = obj.cans_left_shelf;
            elseif shelf_index == 2
                cans_on_shelf = obj.cans_right_shelf;
            else
                error('Invalid shelf index');
            end

            shelf_info = struct('type', {}, 'position', {}, 'quantity', {});

            unique_types = unique(cans_on_shelf);
            unique_types = unique_types(unique_types ~= 0);

            for i = 1:length(unique_types)
                type = unique_types(i);
                positions = [];

                for line = 1:8
                    for rack = 1:6
                        if cans_on_shelf(rack, line) == type
                            positions = [positions; rack, line];
                        end
                    end
                end
                quantity = size(positions, 1);
                shelf_info(i).type = type;
                shelf_info(i).position = positions;
                shelf_info(i).quantity = quantity;
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
