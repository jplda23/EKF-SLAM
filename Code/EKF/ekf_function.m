%% Function EKF

function [saved_mu, saved_sigma, pose_est, pik] = ekf_function(odom_data, sensor_data, landmarks, real, odom)

    showGui = true;  % show a window while the algorithm runs

    INF = 20;

    N = length(landmarks);
    
    observedLandmarks = false(1,N);

    % Initialize belief:
    % mu: 2N+3x1 vector representing the mean of the normal distribution
    % The first 3 components of mu correspond to the pose of the robot,
    % and the landmark poses (xi, yi) are stacked in ascending id order.
    % sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
    mu = zeros(2*N+3, 1);
    mu(1) = 0.5; mu(2) = 0.5;
    robSigma = diag([0.5 0.5 0.5]);
    robMapSigma = zeros(3,2*N);
    mapSigma = INF*eye(2*N);
    sigma = [[robSigma robMapSigma];[robMapSigma' mapSigma]];
    saved_sigma = cell(length(odom_data),1);

    pose_est = struct('x', 0, 'y', 0, 'theta', 0);
    saved_mu = zeros(2*N+3,length(odom_data));

    pik = zeros(4, length(odom_data));

    for t = 1:length(odom_data)
    
        % Perform the prediction step of the EKF
        
        [mu, sigma] = prediction_step(mu, sigma, odom_data(t).odometry, t == 1);

        % Perform the correction step of the EKF
        for a = 1:size(sensor_data(t).sensor, 2)
            if ~isnan(sensor_data(t).sensor(a).id)
                if sensor_data(t).sensor(a).id ~= 1
                    [mu, sigma, observedLandmarks, pik] = correction_step(mu, sigma, sensor_data(t).sensor(a), observedLandmarks, t, pik);
                end
            end     
        end
        
        pose_est.x(t) = mu(1); pose_est.y(t) = mu(2);
        saved_mu(:,t) = mu;
        saved_sigma{t} = sigma;

        % Generate visualization plots of the current state of the filter
        if t == length(odom_data)-1
            plot_state(real, odom, pose_est, mu, sigma, landmarks, t, observedLandmarks, sensor_data(t).sensor, showGui,saved_mu,saved_sigma);
            saveas(gcf,"./figures/trajectory.png");
        end
    end

%     p = 1:length(pik(1, :));
%     figure()
%     plot(p, pik(2, :))
%     hold on
%     %plot(p, pik(2, :))
%     %plot(p, pik(3, :))
%     %plot(p, pik(4, :))
%     legend('Landmarks 1', 'Landmarks 2', 'Landmarks 3', 'Landmarks 4')
%     xlabel("N?? iterations")
%     ylabel("pi")
%     title("pi")

end