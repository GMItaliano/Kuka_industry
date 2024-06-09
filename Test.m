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

%% PYTHON CONFIGURATION
% addpath(genpath('Vision_ML'));  % Add the entire Vision_ML folder
% script_path = fullfile(pwd, 'realpath', 'Vision_ML', 'sai.py');

% %terminate(pyenv);
% pyenv('Version', 'C:\Users\gugal\AppData\Local\Programs\Python\Python39\pythonw.exe');
% 
% % Check if NumPy is installed
% numpy_spec = py.importlib.util.find_spec('numpy');
% 
% % If NumPy is not installed, install it
% if isempty(numpy_spec)
%     py.pip.install('numpy');
% end
% 
% py_Vision_path = fileparts(which('Vision_ML\sai.py'));
% 
% disp(['Python Vision Path: ', py_Vision_path]);  % Debug: Print the path
% 
% if count(py.sys.path, py_Vision_path) == 0
%     insert(py.sys.path, int32(0), py_Vision_path)
% end
% 
% disp(py.sys.path);

% Import the sai_simple module correctly (without .py extension)
try
    vision = py.importlib.import_module('simple_sai');
    disp(vision);  % Debug: Display the imported module
catch e
    disp('Failed to import sai_simple module:');
    disp(e.message);
end

%% KUKA & Simulation INITIALIZATION

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
%2nd 48 right shelf Rack 1 -> Rack 6
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
[vehicle,error_kuka] = kuka_interface(sim);
if(error_kuka==1)
    return;
end

%This function returns relevant information about the mobile platform 
[error,rob_W,rob_L,theta_obs] = vehicle.get_RobotCharacteristics();
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

[error, tgt_pos] = sim.get_KUKAtarget_position(tg_idle);

if error 
    fprintf("ERROR GETTING KUKA TARGETS\n");
    return
end

%% OTHER VARIABLES
%classes
stock_manager = Stock_Manager;  
%armKin = FwdInvKinematics;

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

%% FSM control 

st_idle =           0;
st_docking =        1;
st_registation =    2;
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

% CONTROL CANs , can count where and how many are in each shelf 
current_theta = 0;
%% MAIN LOOP
while stop==0
    %----------------------------------------------------------------------
    %% Robot interface
    % set and get information to/from vrep
    % avoid do processing in between ensure_all_data and trigger_simulation
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
    % offset_ps = [-0.2; 0; 0.1; 90; -90; 0; 0];
    % 
    % % position = can_order(can_count,2);
    % % [error,targetPosition]=sim.get_intermediate_store_position(position);
    % 
    % position_test = [-2.2, -4.20, 0.7];
    % 
    % gain = 0.2;
    % [error, theta_sol, ee_pos] = pickNplace.move2pickNplace(armPosition, position_test', ReadArmJoints, 0, offset_ps);
    % move_arm = 1;
    % 
    % if move_arm            % move joints 
    %     current_theta = current_theta + (theta_sol - current_theta)*gain;
    %     robot_arm.set_joints(current_theta);
    % end

    % if prox_conveyor
    %     disp("STOPPING Conveyor")
    %     error = sim.stop_conveyorbelt();     %stop
    % else 
    %     disp("MOVING Conveyor")
    %     error = sim.move_conveyorbelt();     %motion
    % end
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