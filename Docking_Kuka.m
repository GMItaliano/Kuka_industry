%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This Function is responsible to adjust the Kuka robot conforming the target
%
% 1st stage - Align the robot with target
% 2nd stage - rotate robot for better arm reach
% 3rd stage - final adjustments using v_yrobot (center the arm with the shelf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [rotation, v_yrobot, v_xrobot] = Docking_Kuka(orientation, distance, target_num, target_xy, kuka_xy )
    
    dx = target_xy(1) - kuka_xy(1);
    dy = target_xy(2) - kuka_xy(2);
    
    target_orientation = atan2(dy, dx);
    fprintf("\ntarget Orientation: %d ; dx = %d ; dy = %d\n", target_orientation, dx, dy);

    if (distance > 80) 
        rotation = orientation;
        v_yrobot = 0;
        v_xrobot = 50;
    else
        %ROTATION
        disp('Rotation');
        if(orientation > 0.1 && distance < 40)
            % switch target_num
            %     case 1      % conveyor position
            %         if(dy ~= 0)
            %             rotation = 0.1; 
            %         else 
            %             rotation = 0;
            %         end
            %     case 2      % right shelf
            %         if(dx ~= 0)
            %             rotation = 0.1; 
            %         else 
            %             rotation = 0;
            %         end
            %     case 3      % left shlef
            %         if(dx ~= 0)
            %             rotation = 0.1; 
            %         else 
            %             rotation = 0;
            %         end
            %     case 4      % not recognized
            %         rotation = 0.5;
            %     case 5      % Idle / charging
            %         rotation = 0.5;
            % end

            if (distance < 10)
                v_xrobot = 0;
                v_yrobot = 0;
            else
                v_xrobot = 10;
                v_yrobot = 0;
            end
        else
            v_xrobot = 20;
            v_yrobot = 0;
            rotation = 0;
        end
        
    
    end
end