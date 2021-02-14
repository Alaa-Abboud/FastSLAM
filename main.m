clear all;
close all;
rosshutdown;
setenv('ROS_MASTER_URI','http://alaa-s-rig:11311/');
rosinit;
robotposeworld=rossubscriber('/gazebo/link_states');
robotposeodom=rossubscriber('/husky_velocity_controller/odom');

robotlidar=rossubscriber('/scan');

% Initial position is known (given). Thus use World position to initialize position instead of
% odometry !
myworldpose=robotposeworld.LatestMessage;
worldX=myworldpose.Pose(37).Position.X; 
worldY=myworldpose.Pose(37).Position.Y;
worldquat=[robotposeworld.LatestMessage.Pose(37).Orientation.X,robotposeworld.LatestMessage.Pose(37).Orientation.Y,robotposeworld.LatestMessage.Pose(37).Orientation.Z,robotposeworld.LatestMessage.Pose(37).Orientation.W];
eulZYX=quat2eul(worldquat);
thetaW=eulZYX(3);
pose(1,:)=[worldX,worldY,thetaW];

% Log lidar data ( angles and ranges of all beams)
lidMinAng=robotlidar.LatestMessage.AngleMin;
lidMaxAng=robotlidar.LatestMessage.AngleMax;
lidAngles=lidMaxAng:-(lidMaxAng-lidMinAng)/719:lidMinAng;
lidRanges=robotlidar.LatestMessage.Ranges;

%initialize map
m_prev=ones(200,200)*100;
m_current=updated_occupancy_grid(lidRanges,lidAngles,pose(1,:),m_prev);%update map based on initial WORLD position

for i=1:40 %number of samples
    old_cloud(i,:)=pose(1,:);
    m_prev(:,:,i)=m_current; % m_current is 2D pixelated map matrix - m_prev is a 3D matrix containing a map for each sample point in the cloud
end

for j=2:10
    %retrieve position and orientation info from odometry
    myodom=robotposeodom.LatestMessage;
    odompose=myodom.Pose.Pose.Position;
    odomquat=[myodom.Pose.Pose.Orientation.X,myodom.Pose.Pose.Orientation.Y,myodom.Pose.Pose.Orientation.Z,myodom.Pose.Pose.Orientation.W];
    eulZYX=quat2eul(odomquat);
    theta=eulZYX(3);
    odomX=odompose.X;
    odomY=odompose.Y;
    odomZ=odompose.Z;
    pose(j,:)=[odomX,odomY,theta];
    
    % Generating new cloud containing new possible positions / sample
    % points
    new_cloud=Sample_motion_model(pose(j-1,:),pose(j,:),old_cloud); % (previous pose, new or current pose, old cloud)
    
    lidRanges=robotlidar.LatestMessage.Ranges;
    lidAngles=lidAngles+theta; %% +theta accounts for robot heading;
    
    for k=1:40
        X=new_cloud(k,:);
        wt(k)=measurement_model_map(lidRanges,lidAngles,X,m_prev(:,:,k));
        m_current(:,:,k)=updated_occupancy_grid(lidRanges,lidAngles,X,m_prev(:,:,k));
    end
    wt=wt./sum(wt); % Normalization

    %resampling
    xyt=1:40;
    XYT=randsample(xyt,40,true,wt);
    for a=1:40
        b=XYT(a);
        old_cloud(a,:)=new_cloud(b,:);
        m_prev(:,:,a)=m_current(:,:,b);
    end

    [' move! ',num2str(j)]
    pause(10) % choose time to re-allocate robot based on your convinience.
end
