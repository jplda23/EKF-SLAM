% Micro-simulator
function [landmarks] = microsimulator(waypoints_file,landmarks_file)

    rng(2) %Set the seed to get the same results every iteration. Seed 1 or 2.
    % Get data from files
    fileID = fopen(waypoints_file,'r');
    [waypoints,~] = fscanf(fileID, ['%f' '%f' '\n'],[2,Inf]);
    waypoints = waypoints';
    fclose(fileID);
    
    fileID = fopen(landmarks_file,'r');
    [landmarks,~] = fscanf(fileID, ['%d' '%f' '%f' '\n'],[3,Inf]);
    landmarks = landmarks';
    fclose(fileID);
    
    % Generate trajectory and sensor data
    %Generates a trajectory that follows waypoints with velocity robot_vel.
    %The odometry is obtained using a white gaussian noise function.
    %The sensor parameters are set to read the landmarks that are in a radius
    %of alpha at a minimum distance of line_of_sight.
    
    %simulation parameters
    sample_freq = 10; % Hz %sample frequency
    robot_vel = 0.5; % m/s %robot velocity
    
    sim_time = 30; %seg %maximum simulation time
    init_time = 0; %seg %initial simulation time
    
    %sensor parameters
    line_of_sight = 5; %m
    alpha = pi/2; %radius of observation for the camera
    
    %Create Real data matrix
    Real = zeros(sim_time*sample_freq,4);
    Real(1,1) = init_time;
    Real(1,2) = waypoints(1,1);
    Real(1,3) = waypoints(1,2);
    Real(1,4) = atan2(waypoints(2,2) - waypoints(1,2),waypoints(2,1) - waypoints(1,1));
    
    %Creat odometry data matrix
    Odometria = zeros(sim_time*sample_freq,4);
    Odometria(1,1) = init_time;
    Odometria(1,2) = waypoints(1,1);
    Odometria(1,3) = waypoints(1,2);
    Odometria(1,4) = atan2(waypoints(2,2) - waypoints(1,2),waypoints(2,1) - waypoints(1,1));
    
    %Creat sensor data struct
    sensor_data.sensor = struct;
    j = 1;
    for i = 2:length(waypoints)

        theta = atan2(waypoints(i,2) - waypoints(i-1,2),waypoints(i,1) - waypoints(i-1,1));
  
    
        while sqrt((waypoints(i,1) - Real(j,2))^2 + (waypoints(i,2) - Real(j,3))^2) > 0.2
    
            sensor = struct('time',cell(1,0),'id',cell(1,0),'range',cell(1,0),'bearing',cell(1,0));
            
    
            j = j+1;
    
        
            Real(j,1) = init_time + j/sample_freq;
            Real(j,2) = Real(j-1,2) + (robot_vel)*(1/sample_freq)*cos(theta);
            Real(j,3) = Real(j-1,3) + (robot_vel)*(1/sample_freq)*sin(theta);
            Real(j,4) = theta;
    
            % ADD NOISE TO ODOMETRY
            Odometria(j,1) = init_time + j/sample_freq; 
            Odometria(j,2) = Odometria(j-1,2) + (robot_vel)*(1/sample_freq)*cos(theta) + wgn(1,1,-35);
            Odometria(j,3)  = Odometria(j-1,3) + (robot_vel)*(1/sample_freq)*sin(theta) + wgn(1,1,-35);
            Odometria(j,4) = theta + wgn(1,1,-42);
          
    
            % SENSOR ESTIMATION
            empty = 1;
            for l = 1:length(landmarks)
                range = sqrt((landmarks(l,2) - Real(j,2))^2 + (landmarks(l,3) - Real(j,3))^2);% + wgn(1,1,-42);
    
                if(range < line_of_sight)                
                    bearing = -theta + atan2(landmarks(l,3) - Real(j,3),landmarks(l,2) - Real(j,2));% + wgn(1,1,-42);
    
                    if (-alpha < bearing) && (bearing < alpha)
                        reading = struct;
                        reading.time    = Real(j,1);
                        reading.id      = landmarks(l,1);
                        reading.range   = range;
                        reading.bearing = bearing;
                        sensor(end+1) = reading;
                        empty = 0;
                    end
                end
                if (l == length(landmarks) && empty == 1)
                    reading = struct;
                    reading.time    = Real(j,1);
                    reading.id      = NaN;
                    reading.range   = NaN;
                    reading.bearing = NaN;
                    sensor(end+1) = reading;
                end
            end
            sensor_data(end+1).sensor = sensor;
        end
    
        if(i == length(waypoints))
            sensor_data = sensor_data(2:end);
            Real = Real(1:j-1,:);
            Odometria = Odometria(1:j-1,:);
        end
    
    end

    % Make noise more uniforme.
%         Odometria(:,2) = Real(:,2) + 1;%wgn(1,1,-10); 
%         Odometria(:,3) = Real(:,3) + 1;%wgn(1,1,-10);
%         Odometria(:,4) = Real(:,4) + wgn(1,1,-10);

    save('./Simulador/real_odom_sim.mat',"Real","Odometria");
    save('./Simulador/sensor_data_sim',"sensor_data");

end


%% garbish
% for i = 5:length(Real)-5
%     if(Real(i,4) ~= Real(i+1,4))
%         dif = Real(i+1,4) - Real(i,4);
%         for j = -5:5
%              Real(i+j,4) = Real(i-j,4) + dif*j;
%         end
%     end
% 
% end

% k = 1;
% for i = 1:length(data)
%     for j = 1:length(data(i).sensor)
%         array(k,1) = data(i).sensor(j).time;
%         array(k,2) = data(i).sensor(j).id;
%         array(k,3) = data(i).sensor(j).range;
%         array(k,4) = data(i).sensor(j).bearing;
%         k = k+1;
%     end
% end


%% Plot Data 
% % plot_step(sensor_data, Odometria, Real, landmarks)

%% RUN SET LANDMARKS SLAM
%ainda não funciona

% [mu_1, sigma_1, observedLandmarks_1, pose_est_1, pose_nolandmark_1] = EKF_ids(Odometria, landmarks,sensor_data);
% 
% figure()
% plot(pose_est_1.x,pose_est_1.y)
% figure()
% plot(pose_nolandmark_1.x,pose_nolandmark_1.y)

%% RUN NO ID LANDMARKS SLAM
%ainda não funciona

% [mu, sigma, observedLandmarks, pose_est, pose_nolandmark] = EKF(Odometria, landmarks,sensor_data);
% 
% plot(pose_est.x,pose_est.y)
