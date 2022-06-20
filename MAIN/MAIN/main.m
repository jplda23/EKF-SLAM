%% Main SLAM algorithm

%% INITIALIZE
% clear all variables and close all windows
clear all;
close all;

% Make tools available
addpath('tools');
addpath('data');
addpath('Simulador');
addpath('EKF');
addpath('ML');

%set to true if microsimulator; set to false if real data.
microsim_flag = true;
ML_flag = true;

%% Directory creation
if ~exist('./imagens', 'dir')
    mkdir('imagens');
end
if ~exist('./imagens/camera', 'dir')
    mkdir('imagens/camera');
end
if ~exist('./imagens/video', 'dir')
    mkdir('imagens/video');
end

%% DATA TREATMENT

if (microsim_flag)
    waypoints_file = 'waypoints.dat';
    landmarks_file = 'landmarks_sim.dat';
    landmarks = microsimulator(waypoints_file,landmarks_file);
    [odom_data] = read_data_sim('real_odom_sim.mat');
    load('sensor_data_sim.mat');
else
    landmarks_file = 'landmarks.dat';
    fileID = fopen(landmarks_file,'r');
    [landmarks,~] = fscanf(fileID, ['%d' '%f' '%f' '\n'],[3,Inf]);
    landmarks = landmarks';
    fclose(fileID);

    % Read sensor readings, i.e. odometry and range-bearing sensor
    [odom_data, ~] = read_data('newRealvsOdom.mat', 'sensor_data.mat');
    sensor_data = odom_data;
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

if(microsim_flag)
    if (~ML_flag)
        [saved_mu, saved_sigma,pose_nolandmark] = ekf_function(odom_data.timestep, sensor_data, landmarks, real, odom);    
    
    elseif (ML_flag)
        debug = ekf_ML(odom_data.timestep, sensor_data, landmarks, real, odom);
    end
end


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










