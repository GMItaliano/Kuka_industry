%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This Function is responsible to adjust the Kuka robot conforming the target
%
% 1st stage - Align the robot with target
% 2nd stage - rotate robot for better arm reach
% 3rd stage - final adjustments using v_yrobot (center the arm with the shelf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [rotation, v_yrobot, v_xrobot] = Docking_Kuka(orientation, distance, target)
    
    
    if (distance < 60)  
        %ROTATION
        disp('Rotation');
        switch target
            case 1      % conveyor position
                if (orientation > 0.1)
                    v_yrobot = 0;
                    v_xrobot = 0; 
                    rotation = orientation;
                else
                    
                    rotation = 0.5;
                    
                end
            case 2      % right shelf
    
            case 3      % left shlef
    
            case 4      % not recognized
    
            case 5      % Idle / charging
    
        end
        
        if(distance < 20)
            v_xrobot = 0;
            v_yrobot = 10;
        end

    else
        rotation = orientation;
        v_xrobot = 50;
        v_yrobot = 0;
    end


end