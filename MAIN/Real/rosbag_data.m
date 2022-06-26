% Data analyses ROS retrieve data from rosbag and store in two separate
% files
function rosbag_data(rosbag_name)

    bag = rosbag(rosbag_name);
    start = bag.StartTime;
    finish = bag.EndTime;
    frames = bag.AvailableFrames;
    
%     baseToHead = getTransform(bag,frames{4},frames{15}, start);
%     headToCamera = getTransform(bag,frames{15},frames{12}, start);
%     baseToHeadTranslation = baseToHead.Transform.Translation.X;
%     headToCameraTranslation = headToCamera.Transform.Translation.X;

%     cameraTF = baseToHeadTranslation + headToCameraTranslation;

    i = 1;

%     for t = start:1/50:finish
%         aux = getTransform(bag,frames{4},frames{15},t);
%         headRot(i,1:3) = quat2eul([aux.Transform.Rotation.X aux.Transform.Rotation.Y aux.Transform.Rotation.Z aux.Transform.Rotation.W]);
%         i=i+1;
%     end
    
    % Gather Relevant Data in TimeSeries
    bag_odom = select(bag,"Topic","/odom_diff");
    bag_real = select(bag,"Topic","/odom_gt");
    bag_tf = select(bag,"Topic","/tf");
    tf = readMessages(bag_tf,"DataFormat","struct");
    
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
    Odometria = Odometria(1:end,:);  
    Real = Real(1:end,:);
    
    % Save Workspace
    save('RealvsOdom',"Real","Odometria")
end
