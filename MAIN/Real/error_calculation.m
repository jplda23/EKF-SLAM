function error_calculation(real, odom, saved_mu, landmarks)

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
xlabel("Nº iterations")
ylabel("Error")
title("Errors of the robot's trajectory")

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

end

% Plot Trajectory Error vs Number of Landmarks


