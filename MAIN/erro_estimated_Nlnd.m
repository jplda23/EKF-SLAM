%% Plot erro vs nº landmarks 

saved_mu5 = load('data\saved_mu7-5.mat');
saved_mu7 = load('data\saved_mu7-7.mat');
saved_mu9 = load('data\saved_mu7-9.mat');
saved_mu11 = load('data\saved_mu7-11.mat');
saved_mu12 = load('data\saved_mu7-12.mat');

t = 1:length(saved_mu5.saved_mu(1, :));

% Real vs Estimated 4 Landmarks

saved_mu5 = saved_mu5.saved_mu;
d_real_estimated5 = [saved_mu5(1, :)];

    for m = 1:length(saved_mu5(1, :))
        d_real_estimated5(m) = sqrt((real.x(1, m) - saved_mu5(1, m)).^2 + (real.y(1, m) - saved_mu5(2, m)).^2);
    end

% Real vs Estimated 8 Landmarks

saved_mu7 = saved_mu7.saved_mu;%(:, 1:end-10);
d_real_estimated7 = [saved_mu7(1, :)];

    for m = 1:length(saved_mu7(1, :))
        d_real_estimated7(m) = sqrt((real.x(1, m) - saved_mu7(1, m)).^2 + (real.y(1, m) - saved_mu7(2, m)).^2);
    end


% Real vs Estimated 12 Landmarks

saved_mu9 = saved_mu9.saved_mu;
d_real_estimated9 = [saved_mu9(1, :)];

    for m = 1:length(saved_mu9(1, :))
        d_real_estimated9(m) = sqrt((real.x(1, m) - saved_mu9(1, m)).^2 + (real.y(1, m) - saved_mu9(2, m)).^2);
    end


% Real vs Estimated 16 Landmarks

saved_mu11 = saved_mu11.saved_mu;
d_real_estimated11 = [saved_mu11(1, :)];

    for m = 1:length(saved_mu11(1, :))
        d_real_estimated11(m) = sqrt((real.x(1, m) - saved_mu11(1, m)).^2 + (real.y(1, m) - saved_mu11(2, m)).^2);
    end

 % Real vs Estimated 16 Landmarks

saved_mu12 = saved_mu12.saved_mu;
d_real_estimated12 = [saved_mu12(1, :)];

    for m = 1:length(saved_mu12(1, :))
        d_real_estimated12(m) = sqrt((real.x(1, m) - saved_mu12(1, m)).^2 + (real.y(1, m) - saved_mu12(2, m)).^2);
    end   


% Plot

% Plot Trajectory Error

figure()
plot(t, d_real_estimated5)
hold on
plot(t, d_real_estimated7)
plot(t, d_real_estimated9)
plot(t, d_real_estimated12)
plot(t, d_real_estimated11)
legend('5 Landmarks', '7 Landmarks', '9 Landmarks', '10 Landmarks', '11 Landmarks')
xlabel("Iteration number")
ylabel("Error")
title("Real vs Estimated - Error of the robot's trajectory")



