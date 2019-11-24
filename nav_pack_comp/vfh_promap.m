%Init roscore
rosshutdown;
clc;
clear;
close all;

ipaddress = 'localhost';
rosinit(ipaddress);

global pose0 map vfh

laser = rossubscriber('/front/scan');
jackal = rospublisher('/jackal_velocity_controller/cmd_vel');
odom = rossubscriber('/odometry/filtered');
velmsg = rosmessage(jackal);

x_length=50;% meter
y_length=50;
resolution=5;% grid/meter
map = robotics.OccupancyGrid(x_length,y_length,resolution);
map.GridLocationInWorld=[-x_length/2,-y_length/2];
map.OccupiedThreshold=0.8;

vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;
vfh.DistanceLimits = [0 2];
vfh.RobotRadius = 0.3;
vfh.MinTurningRadius = 0.3;
vfh.SafetyDistance = 0.1;
vfh.HistogramThresholds = [1 1];

targetDir = 0;
distance=10;

rate = robotics.Rate(10);

pose = receive(odom);
quat = [pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
    pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z];
eul = quat2eul(quat);
pose0 = [pose.Pose.Pose.Position.X; pose.Pose.Pose.Position.Y; eul(1)];
clear pose;

while rate.TotalElapsedTime < 50   % exploring
    % Get laser scan data
    laserScan = receive(laser);
    pose = receive(odom);
    [v,w] = VFH_nav(0, laserScan);
    
    % Assign and send velocity commands
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
    
    waitfor(rate);
    
    occval=mapping2(laserScan,pose);
    show(map)
end

inflate(map,0.2);

occval=fliplr(occval');
index=occval==0;
occval(occval==1)=0;
occval(index)=1;
[row,col] = find(occval == 0);
    
while 1
    pose = receive(odom);
    x=pose.Pose.Pose.Position.X;
    y=pose.Pose.Pose.Position.Y;
    
    Start=[x,y]
    
    Goal = input(...
        'enter goal position:\n For example: [0, 0]\n Use data cursor on the occupancy grid to help locate goal position!!!\n')
    
    OptimalPath = astar2(Start,Goal);
    % astarpathshow(OptimalPath);
    
    %%%%%%%%%%%%%%%%%%%%% comment when running pure vfh %%%%%%%%%%%%%%%%%%%%%%
    figure
    plot(row,col,'.k')
    hold on;
    axis equal;
    plot(OptimalPath(end,1),OptimalPath(end,2),'xb')
    plot(OptimalPath(:,1),OptimalPath(:,2),'-r')
    set(gca,'xtick',[],'xticklabel',[])
    set(gca,'ytick',[],'yticklabel',[])
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % set arrival condition
    n=1;
    nmax=length(OptimalPath);
    range_goal=10;
    previous=0;
    w_record = 0;
    tic;
    
    while 1
        % receive from odom
        pose = receive(odom);
        odom_pose = odometry(pose);
        x = odom_pose(1);
        y = odom_pose(2);
        theta_odom = odom_pose(3);
        
        % read from lidar
        laserScan = receive(laser);
        
        % convert goal from global to local
        R_goal = [cos(theta_odom),sin(theta_odom);...
            -sin(theta_odom),cos(theta_odom)];
        
        %%%%%%%%%%%%%%%%%%%%%%%% comment to run pure vfh %%%%%%%%%%%%%%%%%%%%%%%%%%
        local = R_goal*(([OptimalPath(n,1);OptimalPath(n,2)] -...
            [(x+x_length/2)*resolution;(y+y_length/2)*resolution]));
        
        x_local = local(1);
        y_local = local(2);
        bearing_goal = atan2(y_local,x_local);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%% comment to run vfh astar %%%%%%%%%%%%%%%%%%%%%%%%%%
        %     local = R_goal * ([Goal(1);Goal(2)] - [x;y]);
        %
        %     x_local = local(1);
        %     y_local = local(2);
        %     bearing_goal = atan2(y_local,x_local);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        [v,w] = VFH_nav(bearing_goal,laserScan);
        
        hold on;
        plot((x+x_length/2)*resolution,(y+y_length/2)*resolution,'.b');
        axis equal;
        
        w_record = [w_record w];
        
        % send speed command
        velmsg.Linear.X = v;
        velmsg.Angular.Z = w_record(end);
        send(jackal,velmsg);
        
        % update arrival condition
        range_goal = sqrt((OptimalPath(n,1)-(x+x_length/2)*resolution)^2 +...
            ((OptimalPath(n,2)-(y+y_length/2)*resolution))^2);
        if(range_goal <= 0.5*resolution)
            if(n < nmax)
                n = n + 1;
            elseif(n == nmax)
                break;
            end
            range_goal = sqrt((OptimalPath(n,1)-(x+x_length/2)*resolution)^2 +...
                ((OptimalPath(n,2)-(y+y_length/2)*resolution))^2);
        end
        
    end
    
    total_time = toc
    
    % whether continue setting point 
    state = input('continue? (y/n) \n', 's');
    if strcmp(state,'n')
        break;
    elseif strcmp(state,'y')
        continue;
    end
    
end

rosshutdown