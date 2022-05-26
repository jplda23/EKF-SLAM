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
rosbag_name = ('./rosbags/arucu_rosbag.bag');

bag = rosbag(rosbag_name);
start = bag.StartTime;
finish = bag.EndTime;

bagselect1 = select(bag,"Topic","/odom_gt");

%% Print topics in screen
rosbag info './rosbags/arucu_rosbag.bag';

%% Slam Map Builder
%slamMapBuilder

%% Plot pos in map x,y

bSel = select(bag,'Topic','/odom_gt');
msgStructs = readMessages(bSel,'DataFormat','struct');

xPoints = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yPoints = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);

xPoints = xPoints(1:end);
yPoints = yPoints(1:end);

figure()
plot(xPoints,yPoints)
xlabel("X")
ylabel("Y")
title("Trajectory of the robot")
saveas(gcf,"./imagens/trajectory/trajectory.png");


%% Show image of the frontal camera

startT = bag.StartTime;
endT = bag.EndTime;
%depthLim =1000; %remove all depth data greater than depthLim
dataType = {'/head_camera/rgb/image_raw';'/head_camera/depth/image_raw';'/neck_camera/depth/image_raw'};    

for i=1
    dataT=dataType{i};
    bagselect1 = select(bag, 'Topic', dataT);
    bagselect2 = select(bag, 'Time', [startT  endT], 'Topic', dataT);
    %bagselect2.AvailableTopics{1,end};
    bagselect3 = select(bagselect2, 'Time', [startT  endT]);
    msgs = readMessages(bagselect3);
    msgs{1};
    for j=1:size(msgs,1)
        [img,alpha] = readImage(msgs{j});
        try
            figure(1)
            imshow(img,[min(min(img)) max(max(img))/10]);colormap(jet)
            img=double(img);
            %img(img>depthLim)=0; 
            img(img==0)=NaN; % get rid of spurious data from mesh plot
            figure(2); mesh(img)
        catch
            figure(3);
            imshow(img);
            saveas(gcf,"./imagens/camera2/image_"+j+".png");
        end
        clear img
    end
end
close all


%% Make a video of the robots' prespective

imageNames = dir(fullfile("imagens/camera2",'image_','*.png'));
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

%% Plot velocity over time
bagselect = select(bag,"Time",[start finish]);
ts = timeseries(bagselect1,"Pose.Pose.Position.X","Twist.Twist.Angular.Z");
ts1 = timeseries(bagselect1,"Pose.Pose.Position.X","Pose.Pose.Position.Y");

ts.Data;

% x = max()

% bag  = rosbag('rosbag.bag');
% cam = select(bag,'Topic','/head_camera/rgb/image_raw');
% msgStructs = readMessages(cam,1,"DataFormat","struct"); %Read 1 message
% 
% I = readImage(msgStructs);



%% Image visualization


% image = readImage(cam);
% imshow(image)



% % msgStructs = readMessages(cam, 'DataFormat', 'struct');

% function msg = copyImage(msgStruct)
%     msg = rosmessage(msgStruct.MessageType);
%     fNames = fieldnames(msg);
%     for k = 1:numel(fNames)
%         if ~any(strcmp(fNames{k}, {'MessageType', 'Header'}))
%             msg.(fNames{k}) = msgStruct.(fNames{k});
%         end
%     end
% end
