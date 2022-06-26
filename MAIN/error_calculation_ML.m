function error_calculation_ML(real, odom, saved_mu, landmarks, lnd_order, sensor_data)

    t = 1:length(saved_mu(1, :)); 
    
    % Real vs Odometry
    
    d_real_odom = sqrt((real.x - odom.x).^2 + (real.y - odom.y).^2);
    d_real_odom = d_real_odom(t);
    
    % Real vs Estimated
    
    d_real_estimated = [saved_mu(1, :)];
    
    for m = 1:length(saved_mu(1, :))
        d_real_estimated(m) = sqrt((real.x(1, m) - saved_mu(1, m)).^2 + (real.y(1, m) - saved_mu(2, m)).^2);
    end
    
    % Landmaks 
    
    d_landmark = zeros(length(landmarks(:, 1)), length(saved_mu(1, :)));
    
    for m = 1:length(saved_mu(1, :))
        for l = 1:length(lnd_order)
            lnd = lnd_order(l);
            if saved_mu(3 + 2*l - 1, m) == 0 && saved_mu(3 + 2*l, m) == 0
                d_landmark(l, m) = 0;
            else
                d_landmark(l, m) = sqrt((landmarks(lnd, 2) - saved_mu(3 + 2*l - 1, m))^2 + (landmarks(lnd, 3) - saved_mu(3 + 2*l, m))^2);
            end
        end
    end
    
    % Plot Trajectory Error
    
    figure()
    plot(t, d_real_odom)
    hold on
    plot(t, d_real_estimated)
    legend('Real vs Odometry', 'Real vs Estimated')
    xlabel("Iteration number")
    ylabel("Error(m)")
    title("Errors of the robot's trajectory (m)")
    saveas(gcf,"./figures/ML_trajectory_error_iterations.png");
    
    % Plot Landmark Estimation Error
    
     figure()
    for l = 1:length(lnd_order)

        plot(find(d_landmark(l,:)), d_landmark(l,find(d_landmark(l,:))));
        hold on
    end
    hold off
    legend('Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5','Landmark 6', 'Landmark 7', 'Landmark 8', 'Landmark 9', 'Landmark 10', 'Landmark 11', 'Landmark 12','Landmark 13', 'Landmark 14')
    xlabel("Iteration number")
    ylabel("Error(m)")
    title("Errors of the landmarks' position estimation (m)")
    saveas(gcf,"./figures/ML_landmarks_iterations.png");

end

% Plot Trajectory Error vs Number of Landmarks
