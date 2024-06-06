function [orientation, distance_target, p_avoidance, f_tar] = moveKuka(XTARGET, YTARGET, y, x, theta_obs, rob_W, rob_L, dist, timestep, phi)
    persistent P alpha N d_inter

    % Initialize persistent variables only once
    if isempty(P)
        P = 0;
    end
    if isempty(alpha)
        alpha = 0;
    end
    if isempty(N)
        N = length(theta_obs);
    end
    if isempty(d_inter)
        d_inter = zeros(N, 1);
    end    

    %% 1. TARGET ACQUISITION
    Tau_tar = 20 * timestep;
    Lambda_tar = 1 / Tau_tar;
    Q = 0.001;

    % Target acquisition
    psi_tar = atan2(YTARGET - y, XTARGET - x);
    f_tar = -Lambda_tar * sin(phi - psi_tar);
    f_stoch = sqrt(Q) * randn(1, 1);

    %% 2. OBSTACLE AVOIDANCE
    % Set parameter values
    Too_FAR = 50; % cm
    Tau_obs = 5 * timestep;
    beta1 = 1 / Tau_obs;
    beta2 = 20; % Flexibility parameter (e.g., beta2 = 30...)
    Dtheta = theta_obs(2) - theta_obs(1);

    % Create vectors for psi_obs, lambda_obs, sigma, and fobs
    psi_obs = zeros(N, 1);
    lambda_obs = zeros(N, 1);
    sigma = zeros(N, 1);
    fobs = zeros(N, 1);
    
    % Calculate distance to each obstacle and compute repulsive forces
    Fobs = 0;
    c = 10;
    for i = 1:N
        if i < 11 || i > 19
            % Calculate distance to the obstacle using sensor data
            d_inter(i) = abs((rob_W / 2) / sin(theta_obs(i)));
        else
            % Calculate distance to the obstacle using sensor data
            d_inter(i) = abs((rob_L / 2) / cos(theta_obs(i)));
        end
        
        % Compute actual distance from the robot to the obstacle
        d = dist(i) - d_inter(i);
        
        % Compute repeller value
        psi_obs(i) = phi + theta_obs(i);
        
        % Compute magnitude of repulsion (lambda_obs)
        if d >= Too_FAR
            lambda_obs(i) = 0;
        else
            lambda_obs(i) = beta1 * exp(-(d / beta2));
        end
        
        % Compute range of repulsion (sigma)
        sigma(i) = atan(tan(Dtheta / 2) + ((rob_W / 2) / ((rob_L / 2) + d)));
        
        % Compute fobsi
        fobs(i) = lambda_obs(i) * (phi - psi_obs(i)) * exp(-((phi - psi_obs(i))^2) / (2 * (sigma(i)^2)));
        
        % Compute potential (P) and alpha for avoidance behavior
        K = (lambda_obs(i) * (sigma(i)^2)) / sqrt(exp(1));
        P = P + (lambda_obs(i) * (sigma(i)^2) * exp(-((phi - psi_obs(i))^2) / (2 * sigma(i)^2))) - K;
        alpha = alpha + 10 * atan(c * P) / pi;
        
        % Accumulate total repulsive force (Fobs)
        Fobs = Fobs + fobs(i);
    end
    
    % Calculate combined orientation including target, avoidance, and stochastic components
    orientation = f_tar + Fobs + f_stoch;

    % Calculate distance to the target (Euclidean distance)
    distance_target = sqrt((XTARGET - x)^2 + (YTARGET - y)^2);
    
    % Return avoidance potential for analysis/debugging
    p_avoidance = P;
end
