%% EKF-SLAM

% clear all variables and close all windows
clear all;
close all;

% Make tools available
addpath('tools');
load('real.mat');


% Landmarks. The true landmark positions are not given to the robot
landmarks = read_world('../data/landmarks.dat');

% Read sensor readings, i.e. odometry and range-bearing sensor
data = read_data('RealvsOdom.mat', 'landmark_data.mat');

INF = 9999;

N = size(landmarks,2);

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

odom = struct('x', 0, 'y', 0, 'theta', 0);
for t = 1:size(data.timestep, 2)
    odom.x(t) = data.timestep(t).odometry.x;
    odom.y(t) = data.timestep(t).odometry.y;
    odom.theta(t) = data.timestep(t).odometry.theta;
end

pose_est = struct('x', 0, 'y', 0, 'theta', 0);
pose_nolandmark = struct('x', 0, 'y', 0, 'theta', 0);

real = struct('x', 0, 'y', 0, 'theta', 0);
for t = 1:size(data.timestep, 2)
    real.x(t) = data.timestep(t).real.x;
    real.y(t) = data.timestep(t).real.y;
    real.theta(t) = data.timestep(t).real.theta;
end

% for t = 2:size(data.timestep, 2)
%     odom.x(t) = odom.x(t-1) + data.timestep(t-1).odometry.t * cos(wrapToPi(odom.theta(t-1) + data.timestep(t-1).odometry.r1));
%     odom.y(t) = odom.y(t-1) + data.timestep(t-1).odometry.t * sin(wrapToPi(odom.theta(t-1) + data.timestep(t-1).odometry.r1));
%     odom.theta(t) = wrapToPi(odom.theta(t-1) + data.timestep(t-1).odometry.r1 + data.timestep(t-1).odometry.r2);
% end

% toogle the visualization type
showGui = true;  % show a window while the algorithm runs
%showGui = false; % plot to files instead

for t = 1:size(data.timestep, 2)
%for t = 1:80

    % Perform the prediction step of the EKF
    [mu, sigma] = prediction_step(mu, sigma, data.timestep(t).odometry, t == 1);

    pose_nolandmark.x(t) = mu(1); pose_nolandmark.y(t) = mu(2);
    
    % Perform the correction step of the EKF
    if ~isnan(data.timestep(t).sensor.id)
        [mu, sigma, observedLandmarks] = correction_step(mu, sigma, data.timestep(t).sensor, observedLandmarks);
    end

    % Generate visualization plots of the current state of the filter
    plot_state(real, odom, pose_nolandmark, mu, sigma, landmarks, t, observedLandmarks, data.timestep(t).sensor, showGui);
    disp("Current state vector:")
    disp("mu = "), disp(mu)

    %pose_est.x(t) = mu(1); pose_est.y(t) = mu(2);

end

disp("Final system covariance matrix:"), disp(sigma)
% Display the final state estimate
disp("Final robot pose:")
disp("mu_robot = "), disp(mu(1:3)), disp("sigma_robot = "), disp(sigma(1:3,1:3))
