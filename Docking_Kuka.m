 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This Function is responsible to adjust the Kuka robot conforming the target
%
% 1st stage - Align the robot with target
% 2nd stage - rotate robot for better arm reach
% 3rd stage - final adjustments using v_yrobot (center the arm with the shelf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function [rotation, v_yrobot, v_xrobot] = Docking_Kuka(orientation, distance, target_num, target_xy, kuka_xy )
% 
%     dx = target_xy(1) - kuka_xy(1);
%     dy = target_xy(2) - kuka_xy(2);
% 
%     target_orientation = atan2(dy, dx);
%     fprintf("\ntarget Orientation: %d ; dx = %d ; dy = %d\n", target_orientation, dx, dy);
% 
%     if (distance > 35) 
%         rotation = orientation;
%         v_yrobot = 0;
%         v_xrobot = 50;
%     else %ROTATION
%         disp('Orientation: ');
%         disp(orientation);
%         v_xrobot = 0;
%         v_yrobot = 0;
%         rotation = 0;
%          switch target_num
%             case 1      % conveyor position
%                 if(orientation < -0.1 || orientation > 0.1)
%                      rotation = sign(target_orientation - orientation) * 0.5;
%                 end
%             case 2      % Idle
%                 if(orientation < -0.1 || orientation > 0.1)
%                     rotation =  sign(target_orientation - orientation) * 0.5; 
%                 end
%             case 3      % left shelf
%                 if(orientation < -0.1 || orientation > 0.1)
%                     rotation =  sign(target_orientation - orientation) * 0.5; 
%                 end
%             case 4      % right shelf
%                 if(orientation < -0.1 || orientation > 0.1)
%                     rotation =  sign(target_orientation - orientation) * 0.5; 
%                 end
%             case 5      % Out Docking
%                 rotation = 0;
%                 v_xrobot = 50;
%         end
% 
%         % Aligning to the X-axis for better arm reach
%         if (abs(dx) > 10)
%             v_xrobot = sign(dx) * 10;  % Move along the X-axis
%         else
%             v_xrobot = 0;
%         end
% 
%         if (abs(dy) > 10)
%             v_yrobot = sign(dy) * 10;  % Move along the Y-axis to center with the shelf
%         else
%             v_yrobot = 0;
%         end
% %             if (distance < 10)
% %                 v_xrobot = 0;
% %                 v_yrobot = 0;
% %             else
% %                 v_xrobot = 10;
% %                 v_yrobot = 0;
% %             end
%     end
% end
        
function [rotation] = Docking_Kuka(orientation, distance, target_xy, kuka_xy)
    
    % Define a new target by increasing the X-coordinate of the original target
    new_target_xy = [target_xy(1) + 2, target_xy(2)];
    
    % Calculate the direction to the new target
    dx = new_target_xy(1) - kuka_xy(1);
    dy = new_target_xy(2) - kuka_xy(2);
    target_orientation = atan2(dy, dx);
    
    fprintf("\nCurrent Orientation: %f\n", orientation);
    fprintf("Distance to Target: %f\n", distance);
    fprintf("Target Orientation: %f\n", target_orientation);

    if distance > 35
        rotation = orientation;
    else % ROTATION and final adjustments
        disp('Orientation: ');
        disp(orientation);
        rotation = 0;

        % Adjust the robot orientation to point towards the new target
        if abs(target_orientation - orientation) > 0.01
            rotation = abs(target_orientation - orientation) * 0.1;
        end
    end
end