function [debug, saved_mu, saved_sigma, mul] = ekf_ML(odom_data, sensor_data, landmarks, real, odom)

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
    saved_sigma = cell(length(odom_data),1);
    
    sigma = zeros(3);

    N_ML = 0;
    
    pose_est = struct('x', 0, 'y', 0, 'theta', 0);
    saved_mu = zeros (2*N + 3);
         
    % toogle the visualization type
    showGui = false;  % show a window while the algorithm runs
    %showGui = false; % plot to files instead
    
    iteracao = zeros(size(odom_data, 2), 1);
    
    Total_seen_landmarks = zeros(N,1);
    had_it_been_seen_before = zeros(N,1);
    
    debug = struct('iteracao', cell(1,1), 'vistos_agora', cell(1,1), 'tinham_sido_vistos_qm', cell(1,1), 'vistos_total', cell(1,1), 'N_ML', cell(1,1), 'N_real', cell(1,1), 'pi', cell(1,1), 'mu', cell(1,1), 'mu_landmarks_position', cell(1,1), 'thought_seen_landmarks', cell(1,1));
    
    mul = [];
    j=1;
    
    for t = 1:size(odom_data, 2)
    
        % Perform the prediction step of the EKF
        [mu, sigma] = prediction_step_ML(mu, sigma, odom_data(t).odometry, t == 1);
    
        
        % Perform the correction step of the EKF
        if ~isnan(sensor_data(t).sensor(1).id)
            [mu, sigma, SeenLandmarks, pik, indexes] = correction_step_ML(mu, sigma, sensor_data(t).sensor);

            N_ML = (length(mu)-3)/2;
        
            [Total_seen_landmarks, had_it_been_seen_before, N_real, mul] = seen_before(Total_seen_landmarks, SeenLandmarks, mul);

        
            debug(j).iteracao = t;
            debug(j).vistos_agora = SeenLandmarks;
            debug(j).tinham_sido_vistos_qm = had_it_been_seen_before;
            debug(j).vistos_total = Total_seen_landmarks;
            debug(j).N_ML = N_ML;
            debug(j).N_real = N_real;
            debug(j).pi = pik;
            debug(j).mu = mu;
            debug(j).mu_landmarks_position = mul;
            debug(j).thought_seen_landmarks = indexes;

            j = j+1;


        end
        saved_mu(:,t) = [mu; zeros(2*N + 3 - length(mu), 1)];
        saved_sigma{t} = sigma;
        
        %Para isto estou a dar a dimensão certa ao 'saved_mu' porque só serve para o plot
        pose_est.x(t) = mu(1); pose_est.y(t) = mu(2);

        % Generate visualization plots of the current state of the filter
        if t == length(odom_data)
            plot_state_ML(real, odom, pose_est, mu, sigma, landmarks, t, N_ML, sensor_data(t).sensor, showGui);
            saveas(gcf,"./figures/ML_trajectory.png");
        end
    end

end
