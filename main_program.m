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
% need to choose the arm to control 
%robot_name = 'UR10';
%robot_name = 'Kuka_LBRiisy';
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
% Try to connect to simulator
% Output:
% sim - pointer to class simulator_interface
% error_sim = 1 - impossible to connect to simulator
[sim,error_sim] = simulator_interface();
if(error_sim==1)
    return;
end

%Creation of a communication class object with the manipulator arm
% Input:
% sim - pointer to class simulator_interface
% robot_name - name of arm CoppeliaSim model
% hand_name - name of hand CoppeliaSim model
% Output:
% robot_arm - pointer to class arm_interface
% error_sim = 1 - impossible to connect to simulator
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
%nJoints - number of arm joints.
%Links - dimensions of the links between the axes of rotation
%DistanceHand - distance between the tip of the manipulator and the palm of
% the hand
%MinPositionJoint - array with minimum position for joint 1-6
%MaxPositionJoint - array with maximum position for joint 1-6
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
%Input:
% wrobot - angular speed
% vrobot_x - longitudinal speed (cm/s)
% vrobot_y - lateral speed (cm/s)
%Output:
% error = 1 - error in function
% vel_front_left - front left wheel rotation speed (rad/s)
% vel_front_right - front right wheel rotation speed (rad/s)
% vel_rear_left - rear left wheel rotation speed (rad/s)
% vel_rear_right - rear right wheel rotation speed (rad/s)
if(error==1)
    return;
end

% Set wheels speeds
[error, ~, ~, phi] = vehicle.set_velocity(vel_front_left,vel_front_right,vel_rear_left,vel_rear_right);
%Input:
% vel_front_left - rad/s
% vel_front_right - rad/s
% vel_rear_left - rad/s
% vel_rear_right - rad/s
%Output:
% robot  (xrobot,yrobot) in cm
% phirobot - rad
% error = 1 - error in function
if(error==1)
    return;
end

%% OTHER VARIABLES
%classes
stock_manager = Stock_Manager;  
%armKin = FwdInvKinematics;

%vision
[~,top_image] = sim.innit_top_Image();
[~,conv_image] = sim.innit_conv_Image();
[vision] = vision();

%variables
recognized = 0;
state_finished = 0;
loop_flag = 0;
state = 'idle';

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
    %[error,objectPosition]=sim.get_object_position(LATA_SALSICHAS_2);
    %[error,objectPosition]=sim.get_object_position(LATA_SALSICHAS_3);
    %[error,objectPosition]=sim.get_object_position(LATA_COGUMELOS_1);
    %[error,objectPosition]=sim.get_object_position(LATA_COGUMELOS_2);
    %[error,objectPosition]=sim.get_object_position(LATA_COGUMELOS_3);
    if error == 1
        sim.terminate();
        return;
    end

    % objectPosition - get target position
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE1);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE2);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE3);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE4);
    %[error,targetPosition]=sim.get_target_position(PRATELEIRA1_FRENTE5);
    % etc
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

    %Direct Kinematics calculation
    %Inverse Kinematics - Send values for joints
    %Write joints.
    %armJoints(1)=0*pi/180;
    %armJoints(2)=0*pi/180;
    % armJoints(3)=0*pi/180;
    % armJoints(4)=0*pi/180;
    % armJoints(5)=0*pi/180;
    % armJoints(6)=0*pi/180;
    % armJoints(7)=0*pi/180;
    % error = robot_arm.set_joints(armJoints) %send value for arm Joints in rad
    if error == 1
       sim.terminate();
       return;
    end

    %Functions that allows open/close the hand
    %error = robot_arm.open_hand();     %open
    %error = robot_arm.close_hand();    %close
    %if error == 1
    %    sim.terminate();
    %    return;
    %end

    %% FSM
    
    [~,targetPosition]=sim.get_KUKAtarget_position(3);
    kuka_pos(1) = x;
    kuka_pos(2) = y;
    %disp(targetPosition);
    [orientation, dist_target] = moveKuka(targetPosition(1), targetPosition(2), y,x, theta_obs, rob_W, rob_L, dist, timestep, phi);
    [rotation, v_y, v_x] = Docking_Kuka(orientation, dist_target, 3, kuka_pos, targetPosition);
    wrobot = rotation;
    % vrobot_x = v_x;
    % vrobto_y = v_y;
    if isinf(v_x)
        vel_x = 0;
    end
    if(wrobot > 0.3)
        vrobot_x = 0;
        vrobto_y = 0; 
    else
        vrobot_x = v_x;
        vrobto_y = v_y;
    end
    %vrobot_x = vel_x;
    fprintf("Vx: %d ; Vy: %d ; rot: %d ; dist_target: %d\n", vrobot_x, vrobto_y, wrobot, dist_target)
    fprintf("KUKA position X: %d, Y: %d\n", x,y);

    % if (state_finished)         % If state is finished get new state
    %     disp('NEXT STATE: ');
    %     state_finished = 0; 
    %     state = stock_manager.curr_state(state,recognized);
    %     disp(state);
    % else 
    %     switch state
    %         case 'idle'                         %Inital state where everything is initialized
    %             disp('IDLE STATE');
    %             recognized = 0;
    %             loop_flag = 0;
    % 
    %             [~,targetPosition]=sim.get_KUKAtarget_position(1);
    % 
    %             disp(targetPosition);
    %             [orientation, vel_x, dist_target] = moveKuka(targetPosition(1), targetPosition(2), y,x, theta_obs, rob_W, rob_L, dist, timestep, phi, vrobot_x, 1);
    % 
    %             wrobot = orientation;
    %             vrobot_X = vel_x;
    %             disp(dist_target);
    %             state_finished = 1;
    % 
    %         case 'start'                        %First state responsible start conveyor
    %             disp('START STATE');
    % 
    % 
    %         case 'conveyor'                     %Check if conveyor as triggered the proximity sensor (YES)->call vsion and go to next_st (NO)->wait until a can is detected
    %             disp('CONVEYOR STATE');
    %             %Function that allows you to put the conveyor belt in motion/stop
    %             [~,prox_conveyor] = sim.get_conveyor_sensor_value();
    %             %disp(prox_conveyor);
    % 
    %             if(prox_conveyor)
    %                 disp('Stop Conveyor');
    % 
    %                 if (loop_flag == 0)
    %                     disp('Stop Conveyor');
    %                     sim.stop_conveyorbelt();
    %                     disp('Processing Image');
    %                     [~, conv_img] = sim.get_conv_Image;
    %                     dominant_color = vision.findDominantColor(conv_img);
    %                     disp(dominant_color);
    %                     recognized = strcmp(dominant_color, 'ERROR');
    %                 end
    %                 state_finished  = 1;
    %             else
    %                sim.move_conveyorbelt();
    %             end
    % 
    %         case 'move_to_conveyor'             %Move KUKA to the position to receive cans from conveyor (if it's in position use arm to pick up a can)
    %             disp('MOVE TO CONVEYOR STATE');
    % 
    %         case 'store_KUKA'                   %State where the picked can will be putted in the staging aread in kuka
    %             disp('STORE IN KUKA STATE');
    % 
    %         case 'move_to_shelf'                %After all positions in KUKA are full go to shelfs
    %             disp('MOVE TO SHELF STATE');
    % 
    %         case 'rack_manager'                 %Check which rack and line is empty and available 
    %             disp('RACK MANAGER STATE');
    % 
    %         case 'store_shelf'                  %Movement to pick a can from the KUKA to the right position in the shelf
    %             disp('STORE IN SHELF STATE');
    % 
    %         case 'not_recognized'               %ERROR case a can is not specified
    %             disp('CAN NOT RECOGNIZED STATE');
    %     end
    % end

    %error = sim.move_conveyorbelt();     %motion
    %error = sim.stop_conveyorbelt();     %stop
    %if error == 1
    %    sim.terminate();
    %    return;
    %end

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