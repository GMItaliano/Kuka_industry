classdef pick_n_place
    properties
    
        arm_ctrl
        interface
        idle_xyz
        pick_rot
        place_rot

        place_rot_i2p
        place_rot_p2p

        interm_xyz

        link_size
        max_joint_rot
        min_joint_rot

        offset_pick
        offset_pick2place
        offset_idle2place
    end

    methods
        
        %% Constructor
        function obj = pick_n_place(arm_ctrl, max_rot, min_rot)
        
            obj.arm_ctrl = arm_ctrl;
            %obj.interface = interface;
            
            obj.idle_xyz = [0 ;  -0.5 ; 0.5 ; 90; 0 ; 0 ;  30 ];
                                   
            obj.place_rot = [0; 0; 0; 90];           %[90; 90; 90; 45];

            obj.place_rot_i2p = [0;     0;      0;  0];
            obj.place_rot_p2p = [90;   -90;    90;  0];
             
            obj.pick_rot = [90;  90 ; 0; 0];    
            obj.max_joint_rot = max_rot;
            obj.min_joint_rot = min_rot;

            obj.offset_pick = [0.18; -0.01; -0.0];      %[0.2; 0.01; -0]

            obj.offset_idle2place = [0;     -0.1;     0.07];
            obj.offset_pick2place = [0;     -0.1;     0.07];

            obj.interm_xyz = [ 0.2; 0 ; 1.6; 0; 90; 0; 30];

        end
        
        %% Main Functions
        

        % -> offset = [x,y,z, roll, pitch, yaw, alpha]
        function [error, theta_sol, ee_pos] = move2pickNplace(obj, arm_base, pickNplace_xyz, ReadArmJoints, pick, offset)
            
            if (pick)                     % -----> Pick <-----
                
                ee_pos = pickNplace_xyz + offset(1:3);

                P_A = obj.arm_ctrl.Copp2Arm(arm_base, 0, ee_pos);
                P_des = [P_A' , offset(4:6)'];
            
                % Inverse Kinematic:
                [error, theta_out] = obj.arm_ctrl.inverse_kinematics(P_des, offset(7,1));
                theta_sol = obj.arm_ctrl.solutionSel(theta_out, ReadArmJoints);
            
                % See if the arm is i the desired position:
                [hand_position, hand_orientation, ~, ~, ~] = obj.arm_ctrl.direct_kinematics(theta_out(:,2));
                
                p_C = obj.arm_ctrl.Arm2Copp(arm_base, 0, hand_position);
                
                %ee_pos = p_C;
    
                disp('Arm to Coppelia:');
                disp(p_C);
                disp('Hand orinentation');
                disp(rad2deg(hand_orientation))

                %theta_sol = theta_out(:,2);

            elseif (~pick)                 % ----> Place <-----
                
                
                % See if the arm is i the desired position:
                [hand_position, hand_orientation, ~, ~, ~] = obj.arm_ctrl.direct_kinematics(ReadArmJoints);
               
                %%%% TENTATIVA PARA CONTROLAR A PONTA
                ee_pos = pickNplace_xyz + offset(1:3);

                P_A = obj.arm_ctrl.Copp2Arm(arm_base, 0, ee_pos);
                P_des = [P_A' , offset(4:6)'];
            
                % Inverse Kinematic:
                [error, theta_out] = obj.arm_ctrl.inverse_kinematics(P_des, offset(7,1));
                theta_sol = obj.arm_ctrl.solutionSel(theta_out, ReadArmJoints);
                
                p_C = obj.arm_ctrl.Arm2Copp(arm_base, 0, hand_position);
                
                %ee_pos = p_C;
    
                disp('Arm to Coppelia:');
                disp(p_C);
                disp('Hand orinentation');
                disp(rad2deg(hand_orientation));
                

            else                                    % no mode is called return to idle
                error = -99;
                fprintf("ERROR: %d, No action decided returning to idle\n", error);
                [~, theta_sol, ~] = idle(arm_base, ReadArmJoints);
            end

        end

        function [error, theta_sol, all_pos, end_pos] = idle(obj, arm_base, ReadArmJoints)
            
            all_pos = zeros(5,3);
            end_pos = obj.idle_xyz(1:3,1);

            %P_A = obj.arm_ctrl.Copp2Arm(arm_base, 0, obj.idle_xyz(1:3,1));
            P_des = [obj.idle_xyz(1:3,1)' , obj.idle_xyz(4:6)'];
        
            % Inverse Kinematic:
            [error, theta_out] = obj.arm_ctrl.inverse_kinematics(P_des, obj.idle_xyz(7,1));
            theta_sol = obj.arm_ctrl.solutionSel(theta_out, ReadArmJoints);
        
            % See if the arm is i the desired position:
            [hand_position, hand_orientation, wrist_pos, elbow_pos, shoulder_pos] = obj.arm_ctrl.direct_kinematics(ReadArmJoints);
            
            end_pos = obj.arm_ctrl.Arm2Copp(arm_base, 0, hand_position);
        
            disp('Arm to Coppelia:');
            disp(end_pos);
            disp('Hand orinentation');
            disp(rad2deg(hand_orientation));

            all_pos(1,:) = hand_position';
            all_pos(2,:) = wrist_pos';
            all_pos(3,:) = elbow_pos';
            all_pos(4,:) = shoulder_pos';
            all_pos(5,:) = end_pos';

            %theta_sol = theta_out(:,2);

        end 

        %% Secundary Functions
        
        function [q_dot] = test_jacobian(obj, tolerance, ReadArmJoints, k0, obs)
        
            [current_pos, current_rpy, ~, ~, ~] = obj.arm_ctrl.direct_kinematics(ReadArmJoints);
            % Compute position error
            pos_error = obj.idle_xyz(1:3) - current_pos ;
            
            % Compute orientation error (RPY)
            rpy_error = deg2rad(obj.idle_xyz(4:6)) - current_rpy';

            % Check if the position error is within the tolerance
            if norm(pos_error) < tolerance
                disp('Target position reached.');
                return;
            end

            x_dot = [pos_error; rpy_error];

            % Compute joint velocities using inverse Jacobian
            q_dot = obj.arm_ctrl.inverse_jacobian(obs, ReadArmJoints, x_dot, k0);

        end

    end
end