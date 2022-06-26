%% Main SLAM algorithm

%% INITIALIZE
% clear all variables and close all windows
clear all;
close all;

% Make tools available
addpath('tools');
addpath('data');
addpath('data_sets');
addpath('Simulador');
addpath('EKF');
addpath('ML');
addpath('Real');

%set to true if microsimulator; set to false if real data.
microsim_flag = false;
ML_flag = true;

%% Directory creation
if ~exist('./imagens', 'dir')
    mkdir('imagens');end
if ~exist('./imagens/camera', 'dir')
    mkdir('imagens/camera');
end
if ~exist('./imagens/video', 'dir')
    mkdir('imagens/video');
end

%% DATA TREATMENT

if (microsim_flag)
    waypoints_file = 'simple2.0.dat';
    landmarks_file = 'landmarks_sim2.0.dat';
    landmarks = microsimulator(waypoints_file,landmarks_file);
    [odom_data] = read_data_sim('real_odom_sim.mat');
    load('sensor_data_sim.mat');
else
%     rosbag_name = 'long_run.bag';
%     rosbag(rosbag_name);
    landmarks_file = 'landmarks12.dat';
    fileID = fopen(landmarks_file,'r');
    [landmarks,~] = fscanf(fileID, ['%d' '%f' '%f' '\n'],[3,Inf]);
    landmarks = landmarks';
    fclose(fileID);

% Read sensor readings, i.e. odometry and range-bearing sensor
    [odom_data, ~] = read_data('RealvsOdom7.mat', 'sensor_data7.mat', size(landmarks,1));
end


%to do the plot
for t = 1:size(odom_data.timestep, 2)
    odom.x(t) = odom_data.timestep(t).odometry.x;
    odom.y(t) = odom_data.timestep(t).odometry.y;
    odom.theta(t) = odom_data.timestep(t).odometry.theta;
end

real = struct('x', cell(1,1), 'y', cell(1,1), 'theta', cell(1,1));
for t = 1:size(odom_data.timestep, 2)
    real.x(t) = odom_data.timestep(t).real.x;
    real.y(t) = odom_data.timestep(t).real.y;
    real.theta(t) = odom_data.timestep(t).real.theta;
end


%% RUN EKF

%simulation data
if(microsim_flag)
    if (~ML_flag) %LANDMARKS WITH ID's
        1
        [saved_mu, saved_sigma, pose_nolandmark] = ekf_function(odom_data.timestep, sensor_data, landmarks, real, odom); 
        error_calculation(real, odom, saved_mu, landmarks); 
    
    elseif (ML_flag) %LANDMARKS WITHOUT ID's
        2
        [debug, saved_mu, saved_sigma, lnd_order] = ekf_ML(odom_data.timestep, sensor_data, landmarks, real, odom);
        error_calculation_ML(real, odom, saved_mu, landmarks, lnd_order);
    end
%real data
elseif(~microsim_flag)
    if (~ML_flag) %LANDMARKS WITH ID's
        3
        [saved_mu, saved_sigma, pose_nolandmark, pik] = ekf_function(odom_data.timestep, odom_data.timestep, landmarks, real, odom);
        error_calculation(real, odom, saved_mu, landmarks, odom_data.timestep); 

    elseif (ML_flag) %LANDMARKS WITHOUT ID's
        4
        [debug, saved_mu, saved_sigma, lnd_order] = ekf_ML(odom_data.timestep, odom_data.timestep, landmarks, real, odom);
        error_calculation_ML(real, odom, saved_mu, landmarks, lnd_order);
    end
end

%% Save data

save('data\saved_mu7-12.mat', "saved_mu")


%% Make a video of the robots' prespective
% Uses the previous images to make a video
% Long runtime - Avoid running if unnecessary - used make_video for flag

make_video = false; %Change to true if you want to make a video

if (make_video)
    imageNames = dir(fullfile("imagens/camera",'image_','*.png'));
    imageNames = {imageNames.name}';
    outputVideo = VideoWriter(fullfile("/imagens/video",'shuttle_out.avi'));
    
    %Choose the framerate of the video
    outputVideo.FrameRate = 10; 
    
    open(outputVideo)
    
    for ii = 1:length(odom_data.timestep)
       img = imread(fullfile("imagens/camera","image_"+ii+".png"));
       writeVideo(outputVideo,img)
    end
    
    close(outputVideo)
end

    %save('./Simulador/real_odom_sim.mat',"Real","Odometria");