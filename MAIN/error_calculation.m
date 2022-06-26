function [d_real_odom, d_real_estimated, observed_landmarks] = error_calculation(real, odom, saved_mu, landmarks, sensor_data)

t = 1:length(saved_mu(1, :)) ; 

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
    for l = 1:length(landmarks(:, 1))
        if saved_mu(3 + 2*l - 1, m) == 0 && saved_mu(3 + 2*l, m) == 0
            d_landmark(l, m) = 0;
        else
            d_landmark(l, m) = sqrt((landmarks(l, 2) - saved_mu(3 + 2*l - 1, m))^2 + (landmarks(l, 3) - saved_mu(3 + 2*l, m))^2);
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
ylabel("Error (m)")
title("Errors of the robot's trajectory (m)")
saveas(gcf,"./figures/error_sim_trajectory_iterations.png");

observed_landmarks = 1;
% plot doble y axis landmarks seen and error
% % figure()
% % plot(real.time(t), d_real_odom)
% % hold on
% % plot(real.time(t), d_real_estimated)%,'Color',[0.8500 0.3250 0.0980])
% % xlabel("Iteration number")
% % ylabel("Error (m)")
% % title("Errors of the robot's trajectory (m)")
% % yyaxis("right")
% % observed_landmarks = zeros(1,length(t));
% % for i = 1:length(sensor_data)-1
% %     if(isnan(sensor_data(i).sensor(1).id))
% %         observed_landmarks(1,i) = 0;
% %     else
% %         observed_landmarks(1,i) = length(sensor_data(i).sensor);
% %     end
% % end
% % ylabel("Number of seen landmarks")
% % plot(real.time, observed_landmarks,'Color',[0.4660 0.6740 0.1880])
% % legend('Real vs Odometry', 'Real vs Estimated')
% % set(gca, 'YTick', 0:length(landmarks))
% % set(gca,'ycolor',[0.4660 0.6740 0.1880])
% % saveas(gcf,"./figures/error_sim_trajectory_observations_time.png");


% Plot Landmark Estimation Error

 figure()
 for l = 1:length(landmarks(:, 1))
     %if l ~= 14%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         plot(find(d_landmark(l,:)), d_landmark(l,find(d_landmark(l,:))));
         hold on
    %end
 end
 hold off
 legend('Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5','Landmark 6', 'Landmark 7', 'Landmark 8', 'Landmark 9', 'Landmark 10', 'Landmark 11', 'Landmark 12','Landmark 13', 'Landmark 14')
 xlabel("Iteration number")
 ylabel("Error (m)")
 title("Errors of the landmarks' position estimation (m)")
 saveas(gcf,"./figures/landmark_error_sim_iterations.png");

end

% Plot Trajectory Error vs Number of Landmarks


