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
    properties (Access = private)
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
            obj.cans_left_shelf = zeros(1, 48);
            obj.cans_right_shelf = zeros(1, 48);
            obj.storage = zeros(1,18);           % type 1 -> mushrooms 2 -> sausages
            obj.TARGET_Number = 96;
            obj.save_to_mat('stock_manager.mat');
        end

        
        %% STORAGE MANAGER
         
        function [num_available_spots, des_position] = set_can_storage(obj, type)
            % Sets a can in the storage area and returns available spots and position
            
            obj = obj.load_from_mat('stock_manager.mat');

            % Determine the starting position based on the type of can
            if rem(type, 2) == 0  % Even type numbers start at position 9
                start_position = 18;
                end_pos = 1;
                step = -1;
            else
                start_position = 1; % Odd type numbers start at position 1
                end_pos = 18;
                step = 1;
            end
            
            position = start_position;

            while  position >= 1 || position <= 18
            
                if obj.storage(position) == 0
                
                    obj.storage(position) = type;
                    des_position = position;
                    break;
                
                end
                position = position + step;
            end

            num_available_spots = 18 - sum(obj.storage == type);

            if position > 18 || position < 1
                num_available_spots = 0;
                fprintf("KUKA :: Storage FULL\n");
            end

            obj.save_to_mat('stock_manager.mat');
        end

        
        function [storage_type, total_cans] = get_storage_info(obj)
            % Returns information about cans in storage (type, position, quantity)
            
            obj = obj.load_from_mat('stock_manager.mat');

            total_cans = 0;
            storage_type = zeros(18,1);

            % Iterate over unique types and gather information
            for i = 1:length(obj.storage)
                storage_type(i) = obj.storage(i);
                if obj.storage(i) ~= 0
                    total_cans = total_cans + 1;
                end
            end

        end
        
        function [des_position, num_remaining] = remove_last_can_storage(obj, type)
            % Removes the last can in the storage area and returns remaining spots and position to be taken
            
            obj = obj.load_from_mat('stock_manager.mat');

            % Determine the starting position based on the type of can
            if rem(type, 2) == 0  % Even type numbers start at position 9
                start_position = 18;
                step = -1;
            else
                start_position = 1; % Odd type numbers start at position 1
                step = 1;
            end
            
            position = start_position;

            while  position >= 1 || position <= 18
            
                if obj.storage(position) == 0
                
                    des_position = position - step;
                    obj.storage(des_position) = 0;
                    break;
                
                end
           
                position = position + step;
            end
            
            num_remaining = sum(obj.storage == type);

            if num_remaining == 0 
                fprintf("KUKA :: Storage EMPTY\n");
            end

            obj.save_to_mat('stock_manager.mat');
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
        

        function [des_position, num_remaining] = set_can_shelf(obj, type)
            
            obj = obj.load_from_mat('stock_manager.mat');
            
            % add a can to the first position available
            if type == 2
                initial_position = 49;
                shelf = obj.cans_right_shelf;
            else 
                initial_position = 1;
                shelf = obj.cans_left_shelf;
            end

            % start putting can in position 
            start_position = initial_position + 17;       % Rack 3 Line 2
            end_position = start_position + 11;         % Rack 4 Line 4
            
            count = 1;

            while count > 0 &&  count < 48
            
                if shelf(count) == 0
                    shelf(count) = 1;
                    break;
                end

                count = count + 1;
            end
            
            des_position = start_position + count;

            num_remaining = 11 - count;

            if type == 2
                obj.cans_right_shelf = shelf;
            else 
                obj.cans_left_shelf = shelf;
            end

            obj.save_to_mat('stock_manager.mat');

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

        %% SAVE AND LOAD FUNCTIONS
        function save_to_mat(obj, filename)
            % Saves the current state of the Stock_Manager object to a .mat file
            cans_left_shelf = obj.cans_left_shelf;
            cans_right_shelf = obj.cans_right_shelf;
            storage = obj.storage;
            curr_can = obj.curr_can;
            TARGET_Number = obj.TARGET_Number;
            save(filename, 'cans_left_shelf', 'cans_right_shelf', 'storage', 'curr_can', 'TARGET_Number');
        end
        
        function obj = load_from_mat(obj, filename)
            % Loads the state of the Stock_Manager object from a .mat file
            data = load(filename);
            obj.cans_left_shelf = data.cans_left_shelf;
            obj.cans_right_shelf = data.cans_right_shelf;
            obj.storage = data.storage;
            obj.curr_can = data.curr_can;
            obj.TARGET_Number = data.TARGET_Number;
        end
    end
end
