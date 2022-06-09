% Data analyses ROS
%% Inicialize
clear
close all;
clc
rosshutdown;

%% Directory creation
if ~exist('./imagens', 'dir')
    mkdir('imagens');
end
if ~exist('./imagens/trajectory', 'dir')
    mkdir('imagens/trajectory');
end
if ~exist('./imagens/camera2', 'dir')
    mkdir('imagens/camera2');
end
if ~exist('./video', 'dir')
    mkdir('video');
end

%% Import data
rosbag_name = ('./rosbags/full_info.bag');

bag = rosbag(rosbag_name);
start = bag.StartTime;
finish = bag.EndTime;

%% Depth Measurements
cameraselect = select(bag,"Topic","/head_camera/depth_registered/points");
camerastruct = readMessages(cameraselect);

%cloud is a length(camerastruct) by 1 cell that contains all the depth information
% in X, Y and Z for each frame in the rosbag simulation

%The frame is 640x480 and it starts counting from the bottom right corner
%to the bottom left corner, then up to the bottom left+1 and to the right
%again until all the pixels are covered.
cloud = cell(length(camerastruct),1);

for i=1:length(camerastruct)
    cloud{i} = readXYZ(camerastruct{i,1});
end

%Display a specific frame in 3D with the scatter3 function:

% pointcloud = camerastruct{4,1};
% scatter3(pointcloud)

%% Get real trajectory from Gazebo

%The topic gazebo/model_states stores the position of all structures of the
%simulation including the robot. Therefore we can obtain the real position
%of the robot from the Pose(46), corresponding to mbot

bag_gazebo = select(bag,"Topic","/gazebo/model_states");
msg_gazebo = readMessages(bag_gazebo,"DataFormat","struct");

xReal = cellfun(@(m) double(m.Pose(46).Position.X),msg_gazebo);
yReal = cellfun(@(m) double(m.Pose(46).Position.Y),msg_gazebo);
zReal = cellfun(@(m) double(m.Pose(46).Position.Z),msg_gazebo);

%% Gather Relevant Data in TimeSeries
bag_odom = select(bag,"Topic","/odom_diff");
bag_real = select(bag,"Topic","/odom_gt");

%Gather time, X, Y and Theta values of odometry in Odometria matrix
ts = timeseries(bag_odom,"Pose.Pose.Position.X","Pose.Pose.Position.Y");
or = timeseries(bag_odom,"Pose.Pose.Orientation.X","Pose.Pose.Orientation.Y","Pose.Pose.Orientation.Z","Pose.Pose.Orientation.W");
Odometria(:,2:3) = ts.Data;
Odometria(:,1) = ts.time;
matrix_aux = zeros(length(Odometria),3);
for i = 1:length(Odometria)
    matrix_aux(i,1:3) = quat2eul(or.Data(i,:));
end
Odometria(:,4) = matrix_aux(:,3);

%Gather time, X, Y and Theta real values in Real matrix
%The topic odom_gt receives the ground truth from gazebo/model_states in a 
% 50Hz frequency, therefore we can use it instead of the full data
ts2 = timeseries(bag_real,"Pose.Pose.Position.X","Pose.Pose.Position.Y");
or2 = timeseries(bag_real,"Pose.Pose.Orientation.X","Pose.Pose.Orientation.Y","Pose.Pose.Orientation.Z","Pose.Pose.Orientation.W");
Real(:,2:3) = ts2.Data;
Real(:,1) = ts2.time;
matrix_aux2 = zeros(length(Real),3);
for i = 1:length(Real)
    matrix_aux2(i,1:3) = quat2eul(or2.Data(i,:));
end
Real(:,4) = matrix_aux2(:,3);

%remove the next 2 comments in case of crash at the end of the simmulation:
Odometria = Odometria(1:end-100,:);  
Real = Real(1:end-100,:);

%% Save Workspace
save('RealvsOdom',"Real","Odometria")

% Different way of saving. Used to save all except some files:
% % save('data_new', '-regexp', '^(?!(b_IMU|bag|bag_odom|bag_vel|bsel|cameraselect|pointcloud)$).')

%% Different way of accessing Data from Rosbag - used for debug - Commented

odom = readMessages(bag_odom,'DataFormat','struct');
real = readMessages(bag_real,'DataFormat','struct');

x_odom = cellfun(@(m) double(m.Pose.Pose.Position.X),odom);
y_odom = cellfun(@(m) double(m.Pose.Pose.Position.Y),odom);
theta_odom = cellfun(@(m) double(m.Twist.Twist.Angular.Z),odom);

x_real = cellfun(@(m) double(m.Pose.Pose.Position.X),odom);
y_real = cellfun(@(m) double(m.Pose.Pose.Position.Y),odom);
theta_real = cellfun(@(m) double(m.Twist.Twist.Angular.Z),odom);

bag_vel = select(bag,"Topic","/cmd_vel");
b_IMU = select(bag,"Topic","/imu/data");

msgVel = readMessages(bag_vel,'DataFormat','struct');
msgIMU = readMessages(b_IMU,'DataFormat','struct');
aux = timeseries(bag_vel,"Linear.X");
velX = timeseries2timetable(aux);

%% Print rosbag topics in screen - used for debug
rosbag info './rosbags/full_info.bag';

%% Slam Map Builder - visualization tool for laser data - Commented
% slamMapBuilder

%% Plot pos in map x,y

%Reference shift from Robot frame to the World frame
x = Odometria(:,2)+0.5;
y = Odometria(:,3)+0.5;

%Plot real data vs odometry data
figure()
plot(x,y)
hold on
plot(Real(:,2),Real(:,3))
legend('Odometria','Real')
xlabel("X")
ylabel("Y")
title("Trajectory of the robot")
saveas(gcf,"./imagens/trajectory/trajectory.png");


%% Visualization of the frontal robot camera - visualization - 
% long runtime - Avoid running if unnecessary - commented for safety
% remove double '% %' to run the code Select + 'Ctrl - Shift - R' twice

% % startT = bag.StartTime;
% % endT = bag.EndTime;
% % depthLim =1000; %remove all depth data greater than depthLim
% % dataType = {'/head_camera/rgb/image_raw';'/head_camera/depth_registered/points';'/neck_camera/depth/image_raw'};    
% % 
% % for i=2
% %     dataT=dataType{i};
% %     bagselect1 = select(bag, 'Topic', dataT);
% %     bagselect2 = select(bag, 'Time', [startT  endT], 'Topic', dataT);
% %     bagselect3 = select(bagselect2, 'Time', [startT  endT]);
% %     msgs = readMessages(bagselect3);
% %     msgs{1};
% %     for j=1:size(msgs,1)
% %         [img,alpha] = readImage(msgs{j});
% %         try
% %             figure(1)
% %             imshow(img,[min(min(img)) max(max(img))/10]);colormap(jet)
% %             img=double(img);
% %             img(img>depthLim)=0; 
% %             img(img==0)=NaN; % get rid of spurious data from mesh plot
% %             figure(2); mesh(img)
% %         catch
% %             figure(3);
% %             imshow(img);
% %             saveas(gcf,"./imagens/camera2/image_"+j+".png");
% %         end
% %         clear img
% %     end
% % end
% % close all
% % 

%% Make a video of the robots' prespective
% Uses the previous images to make a video
% Long runtime - Avoid running if unnecessary - commented for safety
% remove double '% %' to run the code Select + 'Ctrl - Shift - R' twice

imageNames = dir(fullfile("imagens/camera",'image_','*.png'));
imageNames = {imageNames.name}';
outputVideo = VideoWriter(fullfile("video",'shuttle_out.avi'));

%Choose the framerate of the video
outputVideo.FrameRate = 10; 

open(outputVideo)

for ii = 1:size(msgs,1)
   img = imread(fullfile("imagens/camera2","image_"+ii+".png"));
   writeVideo(outputVideo,img)
end

close(outputVideo)
