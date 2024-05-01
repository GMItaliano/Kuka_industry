classdef kuka_movement
    properties
        a
        P
        alpha
        distance_ant
        N
        d_inter
        sentido
        enable_ftotal
        estacionar

        Too_FAR 
        Tau_obs
        beta1   
        beta2  
        Dtheta  

        psi_obs    
        lambda_obs 
        sigma      
        fobs  
    end

    methods
        %constructor
        function obj = kuka_init()
            obj.a=0;
            obj.P=0;
            obj.alpha=0;
            obj.distance_ant=0;
            obj.N=length(obj.theta_obs);
            obj.d_inter=zeros(obj.N,1);
            obj.sentido=0;
            obj.enable_ftotal=0;
            obj.estacionar=0;

            obj.Too_FAR = 80; %cm
            obj.Tau_obs = 5*timestep;
            obj.beta1   = 1/obj.Tau_obs;
            obj.beta2   = 20; %este valor é flexível (beta2 = 30...)
            obj.Dtheta  = obj.theta_obs(2)- obj.theta_obs(1);

            obj.psi_obs    = zeros(obj.N,1);
            obj.lambda_obs = zeros(obj.N,1);
            obj.sigma      = zeros(obj.N,1);
            obj.fobs       = zeros(obj.N,1);
        end
        
        %Target Acquisition
        function [f_tar, f_stoch] = target_acq(obj, YTARGET, XTARGET, y, x,timestep, phi)
            assert(isscalar(timestep), 'time_step must be a scalar.');
             Tau_tar = 20* timestep;
             Lambda_tar = 1 / Tau_tar;
             Q=0.001;
        
             % target aquisition
             psi_tar = atan2(YTARGET-y,XTARGET-x);
             f_tar = - Lambda_tar * sin(phi-psi_tar);
             f_stoch = sqrt(Q) * randn(1,1);
        end

        %Obstacle Avoidance 
        function [Fobs] = obst_avoidance_vel(obj,theta_obs, rob_W, rob_L, dist, phi)
            Fobs=0;
            c=10;
            for i=1:obj.N
                if i<11 || i>19
                    
                    obj.d_inter(i)=abs((rob_W/2)/sin(theta_obs(i)));
                    d=dist(i)-obj.d_inter(i);
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
                    obj.d_inter(i)=abs((rob_L/2)/cos(theta_obs(i)));
                    d=dist(i)-obj.d_inter(i);
                   
                end
                %compute repeller value
                obj.psi_obs(i)=phi+theta_obs(i);
                %compute magnitude of repulsion lambda_obs
                if(d>=obj.Too_FAR)
                    obj.lambda_obs(i)=0;
                else
                    obj.lambda_obs(i)=obj.beta1*exp(-(d/obj.beta2));
                end
                %compute range of repulsion
                obj.sigma(i)=atan(tan(obj.Dtheta/2)+((rob_W/2)/((rob_L/2)+d)));
                %compute fobsi
                obj.fobs(i) = obj.lambda_obs(i) * (phi-obj.psi_obs(i)) * exp(-((phi-obj.psi_obs(i))^2)/(2*(obj.sigma(i)^2)));
                %Potencial
                K=(obj.lambda_obs(i)*(obj.sigma(i)^2))/(sqrt(exp(1)));
                obj.P=obj.P+(obj.lambda_obs(i)*(obj.sigma(i)^2)*exp(-(phi-obj.psi_obs(i))^2/(2*obj.sigma(i)^2)))-K;
                obj.alpha=obj.alpha+10*atan(c*obj.P)/pi;
                Fobs=Fobs+obj.fobs(i);
            end
        end
        %Velocity and Orientation
        function [vel_x, orientation] = vel_orientation(obj, YTARGET, XTARGET, y, x, theta_obs, rob_W, rob_L, dist, timestep, phi, velocity)
            
            Fobs = obj.obst_avoidance_vel(theta_obs, rob_W, rob_L, dist, phi);
            [f_tar, f_stoch] = obj.target_acq(YTARGET, XTARGET, y, x, timestep, phi);

            % if( obj.alpha <= 0 )
            %     %f_total=f_tar+f_stoch;
            %    f_total = f_tar+Fobs+f_stoch;
            % else
            %   % f_total=f_tar+Fobs+f_stoch;
            %    f_total=f_tar+Fobs+f_stoch;
            % end
            f_total=f_tar+Fobs+f_stoch;
            
            orientation = f_total;
            %vrobot_x=25;
            distance=sqrt((XTARGET-x)^2+(YTARGET-y)^2);
            vel_x = velocity;

            if(distance<60)
                % lambda_max=1/(vel_x/distance);
                % vel_x=lambda_max*(distance);  
                vel_x = 0;
            else
                vel_x=70;
            end 

        end

    end
end