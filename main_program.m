%   main_program.m
%   Main program for the control of manipulators
%   Author: Luís Louro, llouro@dei.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   % Copyright (C) 2024
%   2024/02/19
%--------------------------------------------------------------------------
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%--------------------------------------------------------------------------
clear

robot_name = 'LBR_iiwa_14_R820';

% need to choose the gripper/hand
hand_name = 'RG2';

%Number of targets for each colour of box
LATA_SALSICHAS_1 = 1;
LATA_SALSICHAS_2 = 2;
LATA_SALSICHAS_3 = 3;
LATA_COGUMELOS_1 = 4;
LATA_COGUMELOS_2 = 5;
LATA_COGUMELOS_3 = 6;

%Number of positions in the shelfes
%1st 48 left shelf Rack 6 -> Rack 1
%2st 48 right shelf Rack 1 -> Rack 6
max_positions = 96;

%Number of proximity sensors for the shelfes
%positioned in the same way as the positions
max_prox_shelf = 96;

% Creation of a communication class object with the simulator
[sim,error_sim] = simulator_interface();
if(error_sim==1)
    return;
end

%Creation of a communication class object with the manipulator arm
[robot_arm,error_man] = arm_interface(sim,robot_name,hand_name);
if error_man == 1
    sim.terminate();
    return;
end

[error,timestep] = sim.get_simulation_timestep();
%get time step value (normally dt=50ms)
if error == 1
    sim.terminate();
    return;
end
[error,nJoints,Links,DistanceHand,MinPositionJoint,MaxPositionJoint] = robot_arm.get_RobotCharacteristics();
if error == 1
    sim.terminate();
    return;
end

%--------------------------------------------------------------------------
%error = sim.move_conveyorbelt(); %Put the conveyor belt in motion
if error == 1
    sim.terminate();
    return;
end

error = robot_arm.open_hand(); %Initialize with hand opened
if error == 1
    sim.terminate();
    return;
end

start = tic;
m=1;
stop=0;

%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%

%% KUKA MOBILE

% Creation of a communication class object with the robot
%Input:
% sim - pointer to class kuka_interface
%Output:
% vehicle - pointer to class kuka_interface
[vehicle,error_kuka] = kuka_interface(sim);
if(error_kuka==1)
    return;
end

%This function returns relevant information about the mobile platform 
[error,rob_W,rob_L,theta_obs] = vehicle.get_RobotCharacteristics();
%Output:
% error = 1 - error in function
% rob_W - robot width (cm)
% rob_L - robot lenght (cm)
% theta_obs - Vector with angle value for each sector of obstacle dynamic 
% i (i = 1, . . . , 29) relative to the frontal direction (in rad)
if(error==1)
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize the mobile platform with zero speeds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vrobot_y = 0.0;     %cm/s
vrobot_x = 0.0;     %cm/s
wrobot = 0.0;       %rad/s
error = 0;
vel_front_left = 0.0;   %rad/s
vel_front_right = 0.0;  %rad/s
vel_rear_left = 0.0;    %rad/s
vel_rear_right = 0.0;   %rad/s

% Convert longitudinal speed, lateral speed and angular speed into wheel 
% speed
[error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = vehicle.Kinematics_vehicle(wrobot, vrobot_y,vrobot_x);
if(error==1)
    return;
end

% Set wheels speeds
[error, ~, ~, phi] = vehicle.set_velocity(vel_front_left,vel_front_right,vel_rear_left,vel_rear_right);
if(error==1)
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%
%       KUKA TARGETS
%%%%%%%%%%%%%%%%%%%%%%%%%

tg_land =   1;
tg_idle =   2;
tg_right =  3;
tg_left =   4;
tg_error =  5;

target_kuka = 0;
offset_tgx = 0;
guarantee_pos = 0;

[error, tgt_pos] = sim.get_KUKAtarget_position(tg_idle);

if error 
    fprintf("ERROR GETTING KUKA TARGETS\n");
    return
end

sig_reached = 0;

%% OTHER VARIABLES
%vision
[~,top_image] = sim.innit_top_Image();
[~,conv_image] = sim.innit_conv_Image();
[vision] = vision();

[~, arm_pos] = robot_arm.get_robot_position;

%% ARM INITIALIZATION
% Initialize combined control object
arm_control = arm_Kinematics(robot_arm, Links, MaxPositionJoint, MinPositionJoint, arm_pos);
pickNplace = pick_n_place(arm_control, MaxPositionJoint, MinPositionJoint);

theta_steps = 20;
steps_count = 1;
steps_count2 = 1;

posi_sig_count = 0;
posi_sig_max = 0;
iterations = 0;

current_theta = 0;

%% FSM control 

st_idle =           0;
st_docking =        1;
%st_registation =    2;
st_conv_pick =      3;
st_interm_place =   4;
st_transit =        5;
st_interm_pick =    6;
st_shelf_place =    7;

sig_finished = 1;
sig_working = 0;
sig_picked = 0;

curr_st = st_idle;
next_st = st_idle;
prev_st = st_idle;
update_st = 0;

% ---- FSM inside each state
stage = 0;          


%% CAN ORGANIZATION
% Right :: shelve mushrooms     -> type 1
% Left  :: shelve sausages      -> type 2

can_type = 0;                  % 0 -> type 2 || 1 -> type 1

stock_manager = Stock_Manager; 

% Tests     [ object_number , can_type, final position]
can_order = [
            LATA_COGUMELOS_1 , 1    ;       % {80} = 'R4_R8'
            LATA_COGUMELOS_2 , 2    ;       % {79} = 'R4_L7'
            LATA_SALSICHAS_1 , 2    ;       % {24} = 'R4_L8'
            LATA_COGUMELOS_3 , 1    ;
            LATA_SALSICHAS_2 , 1    ;
            LATA_SALSICHAS_3 , 2    
            ];

can_count = 1;

%% Movement flags

move_arm = 0;
move_kuka = 0;

move_kuka_stg = 0;

%% MAIN LOOP
while stop==0
    %----------------------------------------------------------------------
    %% Robot interface
    sim.ensure_all_data();
    
    % Convert longitudinal speed, lateral speed and angular speed into wheel 
    % speed
    [error,vel_front_left,vel_front_right,vel_rear_left,vel_rear_right] = vehicle.Kinematics_vehicle(wrobot, vrobot_y,vrobot_x);
    if(error==1)
        return;
    end

    [error,x, y, phi] = vehicle.set_velocity(vel_front_left,vel_front_right,vel_rear_left,vel_rear_right);
    if(error==1)
        return;
    end

     %[x, y, phi_2pi] = vehicle.get_vehicle_pose2pi();

    % trigger obstacles data reception
    error = vehicle.trigger_obstacles();
    % error = 1 - error in function
    if(error==1)
        return;
    end

    [error,dist] = vehicle.get_DistanceSensorAquisition(true, false);
    if(error==1)
        return;
    end

    % ReadArmJoints - get joint value (rad) for arm
    [error,ReadArmJoints] = robot_arm.get_joints();
    if error == 1
        sim.terminate();
        return;
    end

    % armPosition - get robot position
    [error,armPosition] = robot_arm.get_robot_position();
    if error == 1
        sim.terminate();
        return;
    end

    % objectPosition - get object position
    %[error,objectPosition]=sim.get_object_position(LATA_SALSICHAS_1);
    if error == 1
        sim.terminate();
        return;
    end

    % objectPosition - get target position
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE1);
    if error == 1
        sim.terminate();
        return;
    end

    %get simulation time
    [error,sim_time] = sim.get_simulation_time();
    if error == 1
        sim.terminate();
        return;
    end 

    %trigger simulation step
    sim.trigger_simulation();
    %----------------------------------------------------------------------
    % --- YOUR CODE --- %

    %% FSM
    
    [~,prox_conveyor] = sim.get_conveyor_sensor_value();

    if update_st 
        update_st = 0;
        prev_st = curr_st;
        curr_st = next_st;

        % Reset Variables
        stg = 1;

        % counters reset
        posi_sig_count = 0;
        iterations = 0;
        steps_count = 0;
        
        % flags for movement 
        move_kuka = 0;
        move_arm = 0;
        move_kuka_stg = 0;

        offset_tgx = 0;

        %signals
        sig_reached = 0;
                
    end
    
    switch curr_st
    
        case st_idle                    % -> Initial and end state 
            %% IDLE STATE

            % Move KUKA to Idle Position
            target_kuka = tg_idle;

            % Move Arm to Idle Position

            [error, theta_sol, all_pos, ee_pos] = pickNplace.idle(armPosition, ReadArmJoints);

            if error ~= 1
                fprintf("--> ERROR ocurred: #%d\n", error);
            end
            
            posi_sig = arm_control.check_positions(ReadArmJoints, ee_pos, stage);
            
            if posi_sig 
                posi_sig_count = posi_sig_count + 1;
            end

            % Check conditions
            if posi_sig_count > 30 && sig_reached
                
                update_st = 1;
                next_st = st_transit;
            else

                move_kuka = 1;
                move_arm = 1;
                gain = 0.3;
            end

        case st_conv_pick               % -> Pick from the conveyor 
            %% CONVEYOR PICK STATE

            switch stg
                
                case 1      % get vesion and specify the type
                    % -> ADD CONTROL LOGIC FOR VISION
                    can_position = can_order(can_count,1);
                    can_type = can_order(can_count,2);

                    [~,objectPosition]=sim.get_object_position(can_position);

                    offset = [0.19; -0.01; 0.0; 90;  30 ; 0; 0];       % -> IMPORTANT OFFSET FOR THE ARM
                    
                    stg = stg + 1;
                case 2
                    stg_1_pos = [objectPosition(1)+0.1; objectPosition(2); 1.4 ];
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset, phi);
                    gain = 0.1;
                case 3
                    offset(5) = 90;
                    stg_2_pos = [objectPosition(1); objectPosition(2); objectPosition(3)];
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_2_pos, ReadArmJoints, offset, phi);    
                    gain = 0.1;
                case 4
                    val_hand = robot_arm.close_hand();
                case 5 
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset, phi); 
            end
    
            if error ~= 1
            
                fprintf("--> ERROR ocurred: #%d\n", error);
    
            end
    
            [error_prox, prox_sig] = sim.get_conveyor_sensor_value;
            posi_sig = arm_control.check_positions(ReadArmJoints, ee_pos, curr_st);
                           

            if posi_sig 
                posi_sig_count = posi_sig_count + 1;
               
                if posi_sig_count > 30
                    if sig_finished && stg > 5
                    % states update
                    update_st = 1;
                    next_st = st_interm_place;
                    else
                        stg = stg + 1;
                        posi_sig_count = 1;
                        iterations = 0;
                    end
                
                end
            else
                move_arm = 1;
                iteration = iterations + 1;
            end

        case st_interm_place            % -> Place can on car
            %% PLACE STORAGE STATE

            switch stg
                
                case 1      % get target_position
                    offset_ps = [-0.2; 0; 0.1; 90; -90; 0; 0];      % offset place storage

                    if can_type == 1    
                        [num_available_spots, position] = stock_manager.set_can_storage(1);
                    else
                        [num_available_spots, position] = stock_manager.set_can_storage(2);
                    end

                    [~,storage_Position]=sim.get_intermediate_store_position(position);

                    stg = stg + 1;
                case 2
                     stg_1_pos = [storage_Position(1); storage_Position(2); 1.1];
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset_ps, phi);
                    gain = 0.05;
                case 3
                    % offset_ps(5) = -90;
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, storage_Position', ReadArmJoints, offset_ps, phi);    
                    gain = 0.05;
                case 4
                    val_hand = robot_arm.open_hand();
                case 5 
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset_ps, phi); 
            end
    
            if error ~= 1
            
                fprintf("--> ERROR ocurred: #%d\n", error);
    
            end
    
            [error_prox, prox_sig] = sim.get_conveyor_sensor_value;
            posi_sig = arm_control.check_positions(ReadArmJoints, ee_pos, stage);
                           

            if posi_sig 
                posi_sig_count = posi_sig_count + 1;
               
                if posi_sig_count > 30
                    if sig_finished && stg > 5
                    % states update
                    update_st = 1;
                    can_count = can_count + 1;
                    if prox_sig && num_available_spots > 0
                        next_st = st_conv_pick;
                    else
                        next_st = st_transit;
                    end
                        
                    else
                        stg = stg + 1;
                        posi_sig_count = 1;
                        iterations = 0;
                    end
                
                end
            else
                move_arm = 1;
                iteration = iterations + 1;
            end


        case st_transit                 % -> Move from conv <-> shelf
            %% TRANSIT STATE
            
            % check if there are cans in storage or on the hand
            [storage_info, storage_cans] = stock_manager.get_storage_info();
            if storage_cans > 0 
                count = 0;
                temp = 0;

                %check which type has more cans
                for i=1 : 18 
                    if storage_info.type(i) == 1
                        count = count + 1;
                    end
                end
                
                temp = storage_info.quantity - count;

                if temp >= 2                            % -> go to Left Shelve
                    target_kuka = tg_left;
                    can_type = 1;
                else                                    % -> go to Right Shelve 
                    target_kuka = tg_right;
                    can_type = 0;
                end
                
                next_st = st_interm_pick;

            else
                
                %Go to conveyor position
                target_kuka = tg_land;
                next_st = st_conv_pick;

            end

            if sig_reached
                update_st = 1;
            else
                move_kuka = 1;
            end

        case st_interm_pick             % -> Pick from car
            %% PICK STORAGE STATE
            

            switch stg
                case 1      % get target_position
                    offset = [-0.2; 0; 0.1; 90; -90; 0; 0];      % offset pick storage
                    
                    [error, position, num_remaining_storage] = stock_manager.remove_last_can_storage(can_type);

                    [~,storage_Position]=sim.get_intermediate_store_position(position);

                    stg = stg + 1;
                case 2
                     stg_1_pos = [storage_Position(1); storage_Position(2); 1.1];
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset, phi);
                    gain = 0.05;
                case 3
                    % offset_ps(5) = -90;
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, storage_Position', ReadArmJoints, offset, phi);    
                    gain = 0.05;
                case 4
                    val_hand = robot_arm.close_hand();
                case 5 
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset, phi);
            end
            
            if error ~= 1
            
                fprintf("--> ERROR ocurred: #%d\n", error);
    
            end

            %[error_prox, prox_sig] = sim.get_conveyor_sensor_value;
            posi_sig = arm_control.check_positions(ReadArmJoints, ee_pos, stage);
                           

            if posi_sig_count 
                posi_sig_count = posi_sig_count + 1;
               
                if posi_sig_count > 30
                    if sig_finished && stg > 5
                    % states update
                    update_st = 1;
                    next_st = st_shelf_place;
                        
                    else
                        stg = stg + 1;
                        posi_sig_count = 1;
                        iterations = 0;
                    end
                
                end
            else
                move_arm = 1;
                iteration = iterations + 1;
            end

        case st_shelf_place             % -> Place can on shelf
            %% PLACE STORAGE STATE

           offset = [0; -0.1; 0.07; 90; -90; 90; 0];

            switch stg
                case 1     
                    
                    % SHELF LOGIC AND CAN LOGIC
                    [position, num_remaining_shelf] = stock_manager.set_can_shelf(can_type);
                    [~,targetPosition] = sim.get_target_position(position);

                    offset = [0.19; -0.01; 0.0; 90;  30 ; 0; 0];       % -> IMPORTANT OFFSET FOR THE ARM
                    
                    stg = stg + 1;
                case 2
                    stg_1_pos = [targetPosition(1); 0.35; targetPosition(3)+0.17];
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset, phi);
                    gain = 0.05;
                case 3
                    offset(5) = 90;
                    stg_2_pos = [targetPosition(1);  targetPosition(2) ; targetPosition(3)+0.1];
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_2_pos, ReadArmJoints, offset, phi);    
                    gain = 0.05;
                case 4
                    val_hand = robot_arm.close_hand();
                case 5 
                    gain = 0.2;
                    [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, stg_1_pos , ReadArmJoints, offset, phi); 
            end
            
            %[error_prox, prox_sig] = sim.get_conveyor_sensor_value;
            posi_sig = arm_control.check_positions(ReadArmJoints, ee_pos, stage);
                           
            if error ~= 1
            
                fprintf("--> ERROR ocurred: #%d\n", error);
    
            end

            if posi_sig_count || override
                posi_sig_count = posi_sig_count + 1;
               
                if posi_sig_count > 30 || override
                    if sig_finished && stg > 5
                    % states update
                    update_st = 1;
                    
                    if num_remaining_storage > 0
                        next_st = st_interm_pick;
                    else
                        next_st = st_transit;
                    end

                    else
                    stg = stg + 1;
                    posi_sig_count = 1;
                    iterations = 0;
                    end
                end
            else
                move_arm = 1;
                iteration = iterations + 1;
            end

    end
    
    %% ARM MOVEMENT
    
    if move_arm            % move joints 
        current_theta = current_theta + (theta_sol - current_theta)*gain;
        robot_arm.set_joints(current_theta);
    end

    %% KUKA MOVEMENT
    
    if (move_kuka && ( curr_st == st_transit || curr_st == st_idle )) && ~sig_reached
        
        [~,targetPosition]=sim.get_KUKAtarget_position(target_kuka);
        kuka_pos(1) = x;
        kuka_pos(2) = y;
        x_target = targetPosition(1) + offset_tgx;
        [orientation, dist_target, ~, target_orientation] = moveKuka(x_target, targetPosition(2), y,x, theta_obs, rob_W, rob_L, dist, timestep, phi);
        %[rotation] = Docking_Kuka(orientation, dist_target, targetPosition, kuka_pos);

        switch move_kuka_stg
            case 0
                wrobot = orientation;
             
                if wrobot > 0.3
                    vrobot_x = 0;
                    vrobot_y = 0;
                else
                    vrobot_x = min(50, dist_target);
                    vrobot_y = 0;
                end

                if dist_target < 50 
                    move_kuka_stg = move_kuka_stg + 1;
                end
            case 1
                vrobot_x = dist_target;
                vrobot_y = 0;
                wrobot = target_orientation;
                if dist_target < 10 
                    move_kuka_stg = move_kuka_stg + 1;
                end
            case 2
                vrobot_x = (targetPosition(1)-x)*0.5;
                vrobot_y = (targetPosition(2)-y)*0.5;

                if target_kuka == tg_left 
                    offset_tgx = -50;
                elseif target_kuka == tg_idle
                    offset_tgx = 0;
                else
                    offset_tgx = 50;
                end

                wrobot = target_orientation;
                if abs(orientation) < 0.001 || curr_st == st_idle
                    guarantee_pos = guarantee_pos + 1;
                    if  guarantee_pos > 5 || curr_st == st_idle
                        move_kuka_stg = move_kuka_stg + 1;
                    end
                end
            case 3
                sig_reached = 1;
                move_kuka = 0;
                vrobot_x = 0;
                vrobot_y = 0;
                wrobot = 0;
                move_kuka_stg = 0;
                guarantee_pos = 0;
        end

    else
        vrobot_x = 0;
        vrobot_y = 0;
        wrobot = 0;
    end

    %% BUG ARM

    if iterations > 250
        if curr_st == st_place
            override = 1;
        else
            stage = st_idle;
            posi_sig_count = 0;
            sig_finished = 1;
            sig_working = 0;
            steps_count = 0;
            stg = 1;
            prev_st = curr_st;
            iterations = 0;
        end
    elseif can_count > 6
        can_count = 0;
        % fprintf("**END OF PICK and PLACE**\nNo more targets, returning to idle position\n");
        stage = st_idle;
        posi_sig_count = 0;
        sig_finished = 1;
        sig_working = 0;
        steps_count = 0;
        stg = 1;
        prev_st = curr_st;
        iterations = 0;
        [~, theta_sol, all_pos, ee_pos] = pickNplace.idle(armPosition, ReadArmJoints);
    else
        override = 0;
    end
    
    %% CONVEYOR CONTROL
    if prox_conveyor
        %disp("STOPPING Conveyor")
        error = sim.stop_conveyorbelt();            %stop
    else 
        disp("MOVING Conveyor")
        error = sim.move_conveyorbelt();            %motion
    end

    if error == 1
       sim.terminate();
       return;
    end

    m=m+1;
    %----------------------------------------------------------------------
    %It allows to guarantee a minimum cycle time of 50ms for the
    %computation cycle in Matlab
    time = toc(start);
    if time<0.05
        pause(0.05-time);
    else
        pause(0.01);
    end
    start = tic;
    %----------------------------------------------------------------------
end
error = sim.terminate();