function [orientation, Xvelocity, Yvelocity, distance_target] = moveKuka(YTARGET , XTARGET, y,x, theta_obs, rob_W, rob_L, dist, timestep, phi, innit)
    if innit == 1
        P=0;
        alpha=0;
        N=length(theta_obs);
        d_inter=zeros(N,1);
    end

    %% 1. TARGET ACQUISITION
    Tau_tar = 20*timestep;
    Lambda_tar = 1 / Tau_tar;
    Q=0.001;

    % target aquisition
    psi_tar = atan2(YTARGET-y,XTARGET-x);
    f_tar = - Lambda_tar * sin(phi-psi_tar);
    f_stoch = sqrt(Q) * randn(1,1);

    %% 2. OBSTACLE AVOIDANCE
    %2.1 Set parameter values
    N=length(theta_obs); %numero de sensores
    Too_FAR = 80; %cm
    Tau_obs = 5*timestep;
    beta1   = 1/Tau_obs;
    beta2   = 20; %este valor é flexível (beta2 = 30...)
    Dtheta  = theta_obs(2)- theta_obs(1);

    %2.2 Create vectors for psi_obs, _lambda_obs, sigma e fobs
    psi_obs    = zeros(N,1);
    lambda_obs = zeros(N,1);
    sigma      = zeros(N,1);
    fobs       = zeros(N,1);  
    %For each sector i (1...N) compute the contribution of the repulsive force
    Fobs=0;
    c=10;
    for i=1:N
        if i<11 || i>19
            
            d_inter(i)=abs((rob_W/2)/sin(theta_obs(i)));
            d=dist(i)-d_inter(i);
             if d < 30
                if i<11
                  %  vrobot_y=30;
                else
                   % vrobot_y=-30;
               end
            else
                vrobot_y=0;
                
            end
            
        else
            d_inter(i)=abs((rob_L/2)/cos(theta_obs(i)));
            d=dist(i)-d_inter(i);
           
        end
        %compute repeller value
        psi_obs(i)=phi+theta_obs(i);
        %compute magnitude of repulsion lambda_obs
        if(d>=Too_FAR)
            lambda_obs(i)=0;
        else
            lambda_obs(i)=beta1*exp(-(d/beta2));
        end
        %compute range of repulsion
        sigma(i)=atan(tan(Dtheta/2)+((rob_W/2)/((rob_L/2)+d)));
        %compute fobsi
        fobs(i) = lambda_obs(i) * (phi-psi_obs(i)) * exp(-((phi-psi_obs(i))^2)/(2*(sigma(i)^2)));
        %Potencial
        K=(lambda_obs(i)*(sigma(i)^2))/(sqrt(exp(1)));
        P=P+(lambda_obs(i)*(sigma(i)^2)*exp(-(phi-psi_obs(i))^2/(2*sigma(i)^2)))-K;
        alpha=alpha+10*atan(c*P)/pi;
        Fobs=Fobs+fobs(i);
    end
        
        orientation=f_tar+Fobs+f_stoch;

        distance=sqrt((XTARGET-x)^2+(YTARGET-y)^2);
        distance_target = distance;
        
        % if(distance<30)
        %     lambda_max=1/(vrobot_x/distance);
        %     vrobot_x=lambda_max*(distance);                       
        % else
        %     vrobot_x=70;
        % end 
        % Xvelocity = vrobot_x;
        
        [rotation, v_yrobot, v_xrobot] = Docking_Kuka(orientation, distance, target);
        
        orientation = rotation;
        Xvelocity = v_xrobot;
        Yvelocity = v_yrobot;
end

