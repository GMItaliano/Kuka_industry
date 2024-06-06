classdef arm_Kinematics
    properties
        interface
        link_size
        max_joint_rot
        min_joint_rot
        arm_pos
    end
    
    methods
        function obj = arm_Kinematics(arm_interface, links, max_rot, min_rot, pos_arm)
            obj.interface = arm_interface;
            obj.link_size = links;
            obj.max_joint_rot = max_rot;
            obj.min_joint_rot = min_rot;
            obj.arm_pos = pos_arm;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% MATRIXES
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function T = Transform_matrix(obj, a, alpha, d, theta)
            a = deg2rad(a);
            alpha = deg2rad(alpha);
            T = [
                cos(theta)                 -sin(theta)                0.0             a;
                sin(theta)*cos(alpha)      cos(theta)*cos(alpha)      -sin(alpha)     -sin(alpha)*d;
                sin(theta)*sin(alpha)      cos(theta)*sin(alpha)      cos(alpha)      cos(alpha)*d;
                0.0                         0.0                         0.0             1.0;
                ];
        end
        
        function hand_rpy = Rotation_matrix(obj, roll, pitch, yaw)

            yaw = deg2rad(yaw);
            pitch = deg2rad(pitch);
            roll = deg2rad(roll);

            hand_rpy = [
                    cos(roll)*cos(pitch),     -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw),     sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
                    sin(roll)*cos(pitch),     cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw),      -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw); 
                       -sin(pitch)    ,        cos(pitch)*sin(yaw)            ,                          cos(pitch)*cos(yaw)          
                    ];
        end
        
        function [roll_z , pitch_y, yaw_x, orientation] = rotation2RPY(obj, T07)
        
             % Grip Orientation:
            cos_pitch = sqrt( T07(1,1)*T07(1,1) + T07(2,1)*T07(2,1));
            
            % Pitch Rot in y:
            pitch_y = atan2(-T07(3,1), cos_pitch);

            % Roll Rot in Z:
            roll_z = atan2( T07(2,1) / cos_pitch  ,  T07(1,1) / cos_pitch );
            
            % Yaw Rot in x:
            yaw_x = atan2( T07(3,2) / cos_pitch  ,  T07(3,3) / cos_pitch );

            % Gets the orientation of the endeffector
            orientation = [yaw_x*  (abs( yaw_x ) > 0.01), ... 
                           pitch_y*(abs(pitch_y) > 0.01), ...
                           roll_z* (abs( roll_z) > 0.01) ];
        
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% KINEMATICS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function [hand_position, hand_orientation, wrist_pos, elbow_pos, shoulder_pos] = direct_kinematics(obj, joints_theta)
        
            %%%%%%%%%%%%%%%%
            %   DH PARAMS
            %%%%%%%%%%%%%%%%
    
            T0_1 = obj.Transform_matrix(0.0, 0.0, obj.link_size(1), joints_theta(1));
            T1_2 = obj.Transform_matrix(0.0, -90, 0,                joints_theta(2));
            T2_3 = obj.Transform_matrix(0.0,  90, obj.link_size(2), joints_theta(3));
            T3_4 = obj.Transform_matrix(0.0,  90, 0,                joints_theta(4));
            T4_5 = obj.Transform_matrix(0.0, -90, obj.link_size(3), joints_theta(5));
            T5_6 = obj.Transform_matrix(0.0, -90, 0,                joints_theta(6));
            T6_7 = obj.Transform_matrix(0.0,  90, obj.link_size(4), joints_theta(7));
    
            %%%%%%%%%%%%%%%%

            Tshoulder = T0_1;
            Telbow = Tshoulder * T1_2 * T2_3;
            Twrist = Telbow * T3_4 * T4_5;
            Thand = Twrist * T5_6 * T6_7;
            
            T07 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 + T5_6 * T6_7;
            %---Cartesian coordinates of S, E, W
            
            xS_0=Tshoulder(1,4);
            yS_0=Tshoulder(2,4);
            zS_0=Tshoulder(3,4);
            shoulder_pos = [xS_0, yS_0, zS_0]';

            xE_0=Telbow(1,4);
            yE_0=Telbow(2,4);
            zE_0=Telbow(3,4);
            elbow_pos = [xE_0, yE_0, zE_0]';

            xW_0=Twrist(1,4);
            yW_0=Twrist(2,4);
            zW_0=Twrist(3,4);
            wrist_pos = [xW_0, yW_0, zW_0]';
            
            %---Cartesian coordinates of Hand {7}
            % with respect to base {0}
            xh_0=Thand(1,4);
            yh_0=Thand(2,4);
            zh_0=Thand(3,4);
            Ph_0=[xh_0, yh_0, zh_0]';
            
            %---Orientation of Hand {7} with respect to base {0}: Roll-Pitch-Yaw
            %angles
            %R07 = Thand(1:3,1:3);
            [~, ~, ~, orientation] = obj.rotation2RPY(Thand);
            Hand_orientation_rad=orientation;
            
            %---conclusion of Direct kinematics
            % Hand pose:
            hand_position = Thand(1:3,4);
            hand_orientation = Hand_orientation_rad;

            disp('-- DEBUG -- DK --- hand XYZ');
            disp(hand_position);

        end
        
        % receives x y z and r p y and alpha
        function [error,joints_theta] = inverse_kinematics(obj, d_PosRot ,alpha)

            joints_theta = zeros(7,2);
            alpha = deg2rad(alpha);
            % multiplier = 0;
            % disp('-- DEBUG -- hand XYZ');
            % disp(d_PosRot(1:3,1));
            % disp('-- DEBUG -- hand RPY');
            % disp(d_PosRot(4:6,1));

            % --- Define desired hand position and angles

            % if(d_PosRot(1) < 0)
            %     d_PosRot(1) = abs(d_PosRot(1));
            %     multiplier = -1;
            % else
            %     multiplier = 1;
            % end


            Ph0_des = [d_PosRot(1) ; d_PosRot(2) ; d_PosRot(3)];
            R07_des = obj.Rotation_matrix(d_PosRot(4), d_PosRot(5), d_PosRot(6));
            

            %---From desired hand position compute desired wrist
            % position with respect to base{0}

            %z_07=T07_des(1:3,3);
            %Pw0_des = Ph0_des - obj.link_size(4) * z_07;

            W = Ph0_des - obj.link_size(4) * R07_des(1:3,3);
            %W = Pw0_des;
            
            % Position of the Shoulder:
            S = [0; 0; obj.link_size(1)];

            %------Compute theta4 Elbow angle -----
            Lsw = norm(W-S);
            
            L_2 = obj.link_size(2);
            L_3 = obj.link_size(3);

            arg4= (Lsw^2 - L_2^2 - L_3^2)/(2 * L_2 * L_3);

            if (arg4 > 1) || (arg4 < -1)
                error = -4; 
                return;
            end

            theta_4 =acos(arg4);                             %%%%%%%%%%%

            if (theta_4 > obj.max_joint_rot(4)) || (theta_4 < obj.min_joint_rot(4))
                error = -4; 
                return;
            end
            
            % Elbow (desired) Position from alpha

            %v_sw = (W - S); % direction vector, v_sw, from shoulder to wrist1
            %L_sw = norm(W - S); % direction nd==L_sw
            v_sw = (W - S) / Lsw; % unit vector from shoulder to wrist1, ^v_sw
            
            disp(v_sw);

            u = [v_sw(2); -v_sw(1); 0]; % n _|_ u
            u = u / norm(u);

            v = cross(u , v_sw);% % v = n X u cross product
            %v = v / norm(v);
            
            % u = u(:);
            % v = v(:);

            cosbeta = ( Lsw^2 + L_2^2 - L_3^2 ) / ( 2  * Lsw * L_2 ); % co-sines law

            C = S + cosbeta * L_2 * v_sw; % center of the circle described by the elbow
            R = sqrt(1- cosbeta^2) * L_2; % radius of the circle described by the elbow

            % Depends on alpha
            E = C + R*(cos(alpha)*u + sin(alpha)*v); % desired elbow position
            
            % unit vectors of arm links
            % v_SE_n = (E - S) / L_2;                     %unit vector v_SE=(E-S)/norm(E-S)
            v_EW_n = (W - E) / norm(W - E);             % unit vector v_EW=(WE)/norm(W-E)
            %v_SW_n = (W - S) / norm(W - S);                         % unit vector from Shouldr to wrist
            v_CE_n = (E - C) / norm(E - C);             %unit vector v_CE=(E-C)/norm(E-C)
            
            v_SW_n = v_sw (:);
            v_CE_n = v_CE_n(:);

            %disp(v_SW_n);
            %disp(v_CE_n);
            
            %%%% ROTATION THETA 4
            z4 = cross(v_sw , -v_CE_n);
            y4 = v_EW_n;
            x4 = cross(y4 , z4);
            
            R04 = [x4, y4, z4];

            %%%%%%%%%%%%%% DEFINE th1, th2, th3 %%%%%%%%%%%%%%%%%%%

            R34 = [
                cos(theta_4)  ,  -sin(theta_4)  ,  0 ;
                      0       ,        0        , -1 ;
                sin(theta_4)  ,   cos(theta_4)  ,  0
            ];

            R03 = R04 * R34';

            % Compute thetas:
            theta_1 = [0,0];
            theta_2 = [0,0];
            theta_3 = [0,0];
            
            sin_th2 = sqrt(1-R03(3,3)^2); 
            
            for i=1 : 2
            
                theta_2(i) = atan2(sin_th2, R03(3,3));

                if(theta_2(i) == 0)         %Singularity
                    theta_1(i) = 0;
                    theta_3(i) = atan2(-R03(2,1), R03(1,1));

                    if (theta_3(i) > obj.max_joint_rot(3)) || (theta_3(i) < obj.min_joint_rot(3))
                        error = -3; 
                        theta_3(i) = 0;
                        %return;
                    end
                else
                    if (theta_2(i) > obj.max_joint_rot(2)) || (theta_2(i) < obj.min_joint_rot(2))
                        error = -2; 
                        theta_2(i) = 0;
                        %return;
                    end
    
                    theta_1(i) = atan2((R03(2,3) / sin_th2), (R03(1,3) / sin_th2));
    
                    if (theta_1(i) > obj.max_joint_rot(1)) || (theta_1(i) < obj.min_joint_rot(1))
                        error = -1; 
                        theta_1(i) = 0;
                        %return;
                    end
    
                    theta_3(i) = atan2((R03(3,2) / sin_th2), (-R03(3,1) / sin_th2));
                    
                    if (theta_3(i) > obj.max_joint_rot(3)) || (theta_3(i) < obj.min_joint_rot(3))
                        error = -3; 
                        theta_3(i) = 0;
                        %return;
                    end

                end

                sin_th2 = -sin_th2;
            
            end
            
            %%%%%%%%%%%%%% DEFINE th5, th6, th7 %%%%%%%%%%%%%%%%%%%
        
            R47 = R04' * R07_des;

            % Compute thetas:
            theta_5 = [0,0];
            theta_6 = [0,0];
            theta_7 = [0,0];
            
            sin_th6 = sqrt(1-R47(2,3)^2);

            for i=1 : 2
            
                theta_6(i) = atan2(sin_th6, R47(2,3));

                if(theta_6(i) == 0)         %Singularity
                    theta_5(i) = 0;
                    theta_7(i) = atan2(-R47(3,1), -R47(3,2));

                    if (theta_7(i) > obj.max_joint_rot(7)) || (theta_7(i) < obj.min_joint_rot(7))
                        error = -7; 
                        theta_7(i) = 0;
                        %return;
                    end
                else
                    if (theta_6(i) > obj.max_joint_rot(6)) || (theta_6(i) < obj.min_joint_rot(6))
                        error = -6; 
                        theta_6(i) = 0;
                        %return;
                    end
    
                    theta_5(i) = atan2((-R47(3,3) / sin_th6), (R47(1,3) / sin_th6));
    
                    if (theta_5(i) > obj.max_joint_rot(5)) || (theta_5(i) < obj.min_joint_rot(5))
                        error = -5;
                        theta_5(i) = 0;
                        %return;
                    end
    
                    theta_7(i) = atan2((R47(2,2) / sin_th6), ((-R47(2,1) / sin_th6)));
                    
                    if (theta_7(i) > obj.max_joint_rot(7)) || (theta_7(i) < obj.min_joint_rot(7))
                        error = -7; 
                        theta_7(i) = 0;
                        %return;
                    end

                end

                sin_th6 = -sin_th6;
            
            end
            
            % Define the thetas
            joints_theta = [
                            theta_1;
                            theta_2;
                            theta_3;
                            theta_4, theta_4;
                            theta_5;
                            theta_6;
                            theta_7;
                            ];
            
            disp('-- DEBUG: -- IK --- Theta: ');
            disp(rad2deg(joints_theta));
            error = 1;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%            JACOBIAN 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function J = jacobian(obj, theta)
        
            T0_1 = obj.Transform_matrix(0.0, 0.0, obj.link_size(1), theta(1));
            T1_2 = obj.Transform_matrix(0.0,  90, 0,                theta(2));
            T2_3 = obj.Transform_matrix(0.0, -90, obj.link_size(2), theta(3));
            T3_4 = obj.Transform_matrix(0.0,  90, 0,                theta(4));
            T4_5 = obj.Transform_matrix(0.0, -90, obj.link_size(3), theta(5));
            T5_6 = obj.Transform_matrix(0.0, -90, 0,                theta(6));
            T6_7 = obj.Transform_matrix(0.0,  90, obj.link_size(4), theta(7));

            % Compute the transformation matrix from the base to the end-effector
            T0 = eye(4); % Base frame
            T1 = T0 * T0_1;
            T2 = T1 * T1_2;
            T3 = T2 * T2_3;
            T4 = T3 * T3_4;
            T5 = T4 * T4_5;
            T6 = T5 * T5_6;
            T7 = T6 * T6_7; % End-effector frame
            
            % Extract the rotation matrices and position vectors
            p0 = T0(1:3, 4);
            p1 = T1(1:3, 4);
            p2 = T2(1:3, 4);
            p3 = T3(1:3, 4);
            p4 = T4(1:3, 4);
            p5 = T5(1:3, 4);
            p6 = T6(1:3, 4);
            p7 = T7(1:3, 4); % Position of the end-effector
        
            z0 = [0; 0; 1]; % Base frame z-axis
            z1 = T1(1:3, 3); % z-axis of joint 1
            z2 = T2(1:3, 3); % z-axis of joint 2
            z3 = T3(1:3, 3); % z-axis of joint 3
            z4 = T4(1:3, 3); % z-axis of joint 4
            z5 = T5(1:3, 3); % z-axis of joint 5
            z6 = T6(1:3, 3); % z-axis of joint 6
        
            % Compute the Jacobian columns
            J1 = [cross(z0, p7 - p0); z0];
            J2 = [cross(z1, p7 - p1); z1];
            J3 = [cross(z2, p7 - p2); z2];
            J4 = [cross(z3, p7 - p3); z3];
            J5 = [cross(z4, p7 - p4); z4];
            J6 = [cross(z5, p7 - p5); z5];
            J7 = [cross(z6, p7 - p6); z6];
        
            % Assemble the Jacobian matrix
            J = [J1 J2 J3 J4 J5 J6 J7];

        end
        

        function q_dot = inverse_jacobian(obj, obs, theta, x_dot, k0)
            % Compute the Jacobian for the current joint configuration
            J = obj.jacobian(theta);
            J_pinv = pinv(J);
            
            % Forward kinematics to get control points (end-effector positions)
            control_points = obj.direct_kinematics(theta); % Define this function according to your manipulator's forward kinematics
            
            % Compute Jacobians for each control point
            num_points = size(control_points, 1);
            jacobians = zeros(6, length(theta), num_points); % Adjust dimensions as needed
            for i = 1:num_points
                jacobians(:,:,i) = obj.jacobian(theta); % This might need adjustment based on control points
            end
            
            % Compute distances and gradients
            [dists, grads] = obj.distancesAndGrads(control_points, 0.3, obs, jacobians);
            [~, index] = min(dists(:));
            q0_dot = k0 * grads(index, :)';
            
            % Compute the null space projection matrix
            null_space_proj = eye(size(J, 2)) - J_pinv * J;
            
            % Ensure x_dot is the same size as the number of rows of J (6x1)
            if numel(x_dot) ~= size(J, 1)
                error('Dimension of x_dot does not match the number of rows of the Jacobian');
            end
            
            % Compute the joint velocities
            q_dot = J_pinv * x_dot(:) + null_space_proj * q0_dot;
        end
        
        function [dists, grads] = distancesAndGrads(obj, control_points, sphere_radius, obs, jacobians)
            num_points = size(control_points, 1);
            num_obs = size(obs, 1);
            
            dists = zeros(num_points, num_obs);
            grads = zeros(num_points, size(jacobians, 2));
            
            for i = 1:num_points
                cp = control_points;
                
                for j = 1:num_obs
                    sc = obs;
                    
                    % Ensure cp and sc are 3x1 vectors
                    if length(cp) ~= 3 || length(sc) ~= 3
                        error('Control point or obstacle center does not have 3 elements');
                    end
                    
                    % Compute the distance
                    dists(i, j) = norm(cp - sc) - sphere_radius;
                    
                    % Compute the gradient only if within sphere radius
                    if dists(i, j) < 0
                        J = jacobians(:,:,i);
                        grads(i, :) = obj.gradient_to_sphere(cp, sc, sphere_radius, J);
                    end
                end
            end
        end
        
        function dist = distance_to_sphere(obj, control_point, sphere_center, sphere_radius)
            center_distance = norm(control_point - sphere_center);
            dist = center_distance - sphere_radius;
        end
        
        function grad = gradient_to_sphere(obj, control_point, sphere_center, sphere_radius, jacobian)
            % Calculate the normalized vector from the sphere center to the control point
            center_vector = control_point - sphere_center;
            center_vector_normalized = center_vector / norm(center_vector);
            
            % Ensure that the center_vector_normalized is a row vector
            if size(center_vector_normalized, 1) > size(center_vector_normalized, 2)
                center_vector_normalized = center_vector_normalized';
            end
            
            % Calculate the gradient
            grad = center_vector_normalized * jacobian;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%
        %% Conversion functions
        %%%%%%%%%%%%%%%%%%%%%%%%%

        % ---------- MATLAB TO COPPELIA
         function P_C = Arm2Copp(obj, arm_pos, theta, P_A)
            
            % Trans(x, y, z) and Rot(z, theta)
            th = theta + pi/2; 
            T_CA = [
                cos(th),   -sin(th),    0,   arm_pos(1);
                sin(th),    cos(th),    0,   arm_pos(2);
                      0,           0,   1,   arm_pos(3);
                      0,           0,   0,        1
                ];

            P_Ah = [P_A;1];
            P_Ch = T_CA * P_Ah;
            
            P_C = P_Ch(1:3);
        end

        % ---------- COPPELIA TO MATLAB
         function P_A = Copp2Arm(obj, arm_pos, theta, P_C)
            % COPP2ARM Transforms a point from the object coordinate frame to the arm coordinate frame.
            % 
            % P_A = COPP2ARM(obj, arm_pos, theta, P_C) transforms the point P_C 
            % from the object's coordinate frame to the arm's coordinate frame 
            % given the arm's position arm_pos and the orientation angle theta.
            %
            % Inputs:
            %   obj - the object instance (not used in this function but part of the method signature)
            %   arm_pos - a 3-element vector representing the arm's position [x, y, z]
            %   theta - the orientation angle (in radians) of the arm
            %   P_C - a 3-element vector representing the point in the object's coordinate frame
            %
            % Output:
            %   P_A - a 3-element vector representing the point in the arm's coordinate frame
        
            % Calculate the transformation angle
            th = theta + pi/2;
            
            % Debug: Print inputs
            % disp('Input arm position:');
            % disp(arm_pos);
            % disp('Input theta:');
            % disp(theta);
            % disp('Input point P_C:');
            % disp(P_C);
            
            % Define the homogeneous transformation matrix (Rotation around z-axis and Translation)
            T_AC = [
                cos(th),      sin(th),      0,     -arm_pos(1)*cos(th)-arm_pos(2)*sin(th);
                -sin(th),     cos(th),      0,     arm_pos(1)*sin(th)-arm_pos(2)*cos(th);
                0,                  0,      1,     -arm_pos(3);
                0,                  0,      0,     1
            ];
            
            

            % Debug: Print transformation matrix
            % disp('Transformation matrix T_AC:');
            % disp(T_AC);
            % 
            % Convert the point to homogeneous coordinates
            P_Ch = [P_C;1];
        
            % Apply the transformation
            P_Ah = T_AC * P_Ch;
        
            % Convert back from homogeneous coordinates to Cartesian coordinates
            P_A = P_Ah(1:3);
            
            % Debug: Print output
            % disp('Transformed point P_A:');
            % disp(P_A);
        end 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %% Best Solution thetas
        %%%%%%%%%%%%%%%%%%%%%%%%%

        % function to calculate the less movemnt needed dor the arm  <--- esta aqui na posição inicial 
               
        function theta_sol = solutionSel(obj, th_I, th_J)
            % Initialize the solution vector
            theta_sol = zeros(7, 1);
        
            % Calculate all possible solutions and their displacements
            sol_dis = [
                th_J(1)-th_I(1,1),  th_J(1)-th_I(1,1),  th_J(1)-th_I(1,2),  th_J(1)-th_I(1,2);
                th_J(2)-th_I(2,1),  th_J(2)-th_I(2,1),  th_J(2)-th_I(2,2),  th_J(2)-th_I(2,2);
                th_J(3)-th_I(3,1),  th_J(3)-th_I(3,1),  th_J(3)-th_I(3,2),  th_J(3)-th_I(3,2);
                th_J(4)-th_I(4,1),  th_J(4)-th_I(4,1),  th_J(4)-th_I(4,2),  th_J(4)-th_I(4,2);
                th_J(5)-th_I(5,1),  th_J(5)-th_I(5,2),  th_J(5)-th_I(5,1),  th_J(5)-th_I(5,2);
                th_J(6)-th_I(6,1),  th_J(6)-th_I(6,2),  th_J(6)-th_I(6,1),  th_J(6)-th_I(6,2);
                th_J(7)-th_I(7,1),  th_J(7)-th_I(7,2),  th_J(7)-th_I(7,1),  th_J(7)-th_I(7,2)
            ];
        
            sol = [
                th_I(1,1), th_I(1,1), th_I(1,2), th_I(1,2);
                th_I(2,1), th_I(2,1), th_I(2,2), th_I(2,2);
                th_I(3,1), th_I(3,1), th_I(3,2), th_I(3,2);
                th_I(4,1), th_I(4,1), th_I(4,2), th_I(4,2);
                th_I(5,1), th_I(5,2), th_I(5,1), th_I(5,2);
                th_I(6,1), th_I(6,2), th_I(6,1), th_I(6,2);
                th_I(7,1), th_I(7,2), th_I(7,1), th_I(7,2)
            ];
        
            % Calculate the sum of absolute displacements for joints 1, 3, 4, 5, and 7
            displacements = sum(abs(sol_dis([1, 3, 4, 5, 7], :)), 1);
        
            % Find the index of the minimum displacement
            [~, min_idx] = min(displacements);
        
            % Assign the corresponding solution to theta_sol
            theta_sol = sol(:, min_idx);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Positions Check and Worksapce
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function isReachable = check_reachability(obj, target_pos)
            % Compute the maximum reach of the arm
            max_reach = sum(obj.link_size);
            
            % Compute the minimum reach (assuming the arm can't compress further)
            min_reach = abs(obj.link_size(1) - obj.link_size(2) - obj.link_size(3) - obj.link_size(4));
            
            % Compute the distance from the base to the target position
            distance_to_target = norm(target_pos);
            
            % Check if the target is within the reach of the arm
            if distance_to_target <= max_reach && distance_to_target >= min_reach
                isReachable = true;
            else
                isReachable = false;
            end
        end

        % Function to check if the position of the arm is equal to the desired position
        function error =  check_positions(obj, arm_joints, des_ee, state)
        
            [hand_position, ~, ~, ~, ~] = obj.direct_kinematics(arm_joints);

            curr_pos = obj.Arm2Copp(obj.arm_pos, 0, hand_position);
            
            %val = abs(des_ee(1) - curr_pos(1)) > 0.02) || (abs(des_ee(2) - curr_pos(2)) > 0.02) || (abs(des_ee(3) - curr_pos(3)) > 0.02;
            
            if state == 3 || state == 6               % -> Pick
                treshold = 0.01;
            elseif state == 4 || state == 7           % -> Place
                treshold = 0.02;
            else                        % -> Idle
                treshold = 0.025;
            end

            % if (any(abs(des_ee - curr_pos) > treshold) || (abs(des_ee(3) - curr_pos(3)) > 0.3))
            if (abs(des_ee - curr_pos) > treshold) 
                fprintf("Going to Desired Position\n");
                error = 0;
            else
                fprintf("Reached Desired Position\n");
                error = 1;
            end
        
        end
        
        function [is_in_workspace, can_reach] = is_in_workspace(obj, target_pos, target_orientation)
            % Check if the position is within the workspace
            is_in_workspace = obj.check_reachability(target_pos);
            
            % If it is within the workspace, check if the robot can reach it
            if is_in_workspace
                % Define desired position and orientation
                desired_pos_rot = [target_pos; target_orientation];
                alpha = 0; % You may adjust alpha based on your requirements
                
                % Check if the robot can achieve the desired position and orientation
                [error, ~] = obj.inverse_kinematics(desired_pos_rot, alpha);
                can_reach = (error == 1);
            else
                can_reach = false;
            end
        end

    end

end