function [d_real_odom, d_real_estimated, observed_landmarks] = error_calculation(real, odom, saved_mu, landmarks,sensor_data)

t = 1:length(saved_mu(1, :));
length(t)

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

figure(1)
plot(t, d_real_odom)
hold on
plot(t, d_real_estimated)
legend('Real vs Odometry', 'Real vs Estimated')
xlabel("Nº iterations")
ylabel("Error (m)")
title("Errors of the robot's trajectory (m)")
saveas(gcf,"./figures/error_sim_trajectory_iterations.png");
% 
% figure(saved_mu)
% plot(odom.time(t), d_real_odom)
% hold on
% plot(real.time(t), d_real_estimated)
% legend('Real vs Odometry', 'Real vs Estimated')
% xlabel("Time (s)")
% ylabel("Error (m)")
% title("Errors of the robot's trajectory (m)")
% saveas(gcf,"./figures/error_sim_trajectory_time.png");

% plot doble y axis landmarks seen and error
figure()
plot(odom.time(t), d_real_odom)
hold on
plot(real.time(t), d_real_estimated)%,'Color',[0.8500 0.3250 0.0980])
xlabel("Time (s)")
ylabel("Error (m)")
title("Errors of the robot's trajectory (m)")
yyaxis("right")
observed_landmarks = zeros(1,length(t));
for i = 1:length(sensor_data)-1
    if(isnan(sensor_data(i).sensor(1).id))
        observed_landmarks(1,i) = 0;
    else
        observed_landmarks(1,i) = length(sensor_data(i).sensor);
    end
end
ylabel("Number of Landmarks seen")
plot(real.time, observed_landmarks,'Color',[0.4660 0.6740 0.1880])
legend('Real vs Odometry', 'Real vs Estimated')
set(gca, 'YTick', 0:length(landmarks))
set(gca,'ycolor',[0.4660 0.6740 0.1880])
saveas(gcf,"./figures/error_sim_trajectory_observations_time.png");


% Plot Landmark Estimation Error

 figure()
 for l = 1:length(landmarks(:, 1))
     plot(find(d_landmark(l,:)), d_landmark(l,find(d_landmark(l,:))));
     hold on
 end
 hold off
%  plot(t, d_landmark(2,:))
%  plot(t, d_landmark(3,:))
%  plot(t, d_landmark(4,:))
%  plot(t, d_landmark(5,:))
%  plot(t, d_landmark(6,:))
%  plot(t, d_landmark(7,:))
% plot(t, d_landmark(8,:))
 legend('Landmark 0', 'Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5','Landmark 6', 'Landmark 7')
 xlabel("Nº iterations")
 ylabel("Error")
 title("Errors of the landmarks' position estimation")
 saveas(gcf,"./figures/landmark_error_sim_iterations.png");



% Plot Trajectory Error vs Number of Landmarks

figure()
 for l = 1:length(landmarks(:, 1))
     plot(real.time(find(d_landmark(l,:))), d_landmark(l,find(d_landmark(l,:))));
     hold on
 end
 hold off
%  plot(t, d_landmark(2,:))
%  plot(t, d_landmark(3,:))
%  plot(t, d_landmark(4,:))
%  plot(t, d_landmark(5,:))
%  plot(t, d_landmark(6,:))
%  plot(t, d_landmark(7,:))
% plot(t, d_landmark(8,:))
 legend('Landmark 0', 'Landmark 1', 'Landmark 2', 'Landmark 3', 'Landmark 4', 'Landmark 5','Landmark 6', 'Landmark 7')
 xlabel("Time (s)")
 ylabel("Error (m)")
 title("Errors of the landmarks' position estimation")
 saveas(gcf,"./figures/landmark_error_sim_time.png");









end



