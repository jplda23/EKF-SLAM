function debug = ekf_ML(odom_data, sensor_data, landmarks, real, odom)

    INF = 9999;
    
    N = length(landmarks);
    
    observedLandmarks = false(1,N);
    
    % Initialize belief:
    % mu: 2N+3x1 vector representing the mean of the normal distribution
    % The first 3 components of mu correspond to the pose of the robot,
    % and the landmark poses (xi, yi) are stacked in ascending id order.
    % sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
    mu = zeros(3, 1);
    mu(1) = 0.5; mu(2) = 0.5;
    
    sigma = zeros(3);
    
    pose_est = struct('x', 0, 'y', 0, 'theta', 0);
         
    % toogle the visualization type
    showGui = true;  % show a window while the algorithm runs
    %showGui = false; % plot to files instead
    
    iteracao = zeros(size(odom_data, 2), 1);
    
    Total_seen_landmarks = zeros(N,1);
    had_it_been_seen_before = zeros(N,1);
    
    debug = struct('iteracao', cell(1,1), 'vistos_agora', cell(1,1), 'tinham_sido_vistos_qm', cell(1,1), 'vistos_total', cell(1,1), 'N_ML', cell(1,1), 'N_real', cell(1,1), 'pi', cell(1,1), 'mu', cell(1,1), 'mu_landmarks_position', cell(1,1), 'thought_seen_landmarks', cell(1,1));
    
    mul = [];
    
    for t = 1:size(odom_data, 2)
    %for t = 1:80
    
        % Perform the prediction step of the EKF
        [mu, sigma] = prediction_step_ML(mu, sigma, odom_data(t).odometry, t == 1);
    
        
        % Perform the correction step of the EKF
        if ~isnan(sensor_data(t).sensor(1).id)
            [mu, sigma, SeenLandmarks, pik, indexes] = correction_step_ML(mu, sigma, sensor_data(t).sensor);

            N_ML = (length(mu)-3)/2;
        
            [Total_seen_landmarks, had_it_been_seen_before, N_real, mul] = seen_before(Total_seen_landmarks, SeenLandmarks, mul);
        
            debug(t).iteracao = t;
            debug(t).vistos_agora = SeenLandmarks;
            debug(t).tinham_sido_vistos_qm = had_it_been_seen_before;
            debug(t).vistos_total = Total_seen_landmarks;
            debug(t).N_ML = N_ML;
            debug(t).N_real = N_real;
            debug(t).pi = pik;
            debug(t).mu = mu;
            debug(t).mu_landmarks_position = mul;
            debug(t).thought_seen_landmarks = indexes;

        end
        
        pose_est.x(t) = mu(1); pose_est.y(t) = mu(2);
        % Generate visualization plots of the current state of the filter
        plot_state_ML(real, odom, pose_est, mu, sigma, landmarks, t, observedLandmarks, sensor_data(t).sensor, showGui);
    
    end

end
