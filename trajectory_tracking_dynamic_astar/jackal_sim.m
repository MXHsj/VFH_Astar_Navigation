%% initialize
clc;
clear;
close all;
rosshutdown;

ipaddress = 'localhost';
rosinit(ipaddress);

burger = rospublisher('/jackal_velocity_controller/cmd_vel');
velmsg = rosmessage(burger);
laser = rossubscriber('/front/scan');
odom = rossubscriber('/odometry/filtered');

global x_length y_length resolution map
x_length=40;    % meter
y_length=40;
resolution=5; % grid/meter
map = robotics.OccupancyGrid(x_length,y_length,resolution);
map.GridLocationInWorld=[-x_length/2,-y_length/2];
map.OccupiedThreshold=0.75;

global vfh
vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;
vfh.DistanceLimits = [0 2];
vfh.RobotRadius = 0.3;
vfh.MinTurningRadius = 0.4;
vfh.SafetyDistance = 0.1;
vfh.HistogramThresholds = [3 10];

rate = robotics.Rate(10);

pose = receive(odom);
quat = [pose.Pose.Pose.Orientation.W, ...
    pose.Pose.Pose.Orientation.X, ...
    pose.Pose.Pose.Orientation.Y, ...
    pose.Pose.Pose.Orientation.Z];
eul = quat2eul(quat);
global pose0
pose0 = [pose.Pose.Pose.Position.X; ...
    pose.Pose.Pose.Position.Y; ...
    eul(1)];

%% mapping 
map_time = 80;
reset(rate);
while rate.TotalElapsedTime < map_time
    
    laserScan = receive(laser);
    pose = receive(odom);
    [v,w] = VFH_nav(0, laserScan);
    
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(burger,velmsg);
    waitfor(rate);
    occval=mapping2(laserScan,pose);
    show(map)
end

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(burger,velmsg);

occval=fliplr(occval');
index=occval==0;
occval(occval==1)=0;
occval(index)=1;
[row,col] = find(occval == 0);

%% planning
pose = receive(odom);
x=pose.Pose.Pose.Position.X;
y=pose.Pose.Pose.Position.Y;

Start = [x,y]
goal_x = input('enter goal position x:  ');
goal_y = input('enter goal position y:  ');
goal_theta = input('enter goal heading theta: ');
Goal = [goal_x, goal_y, goal_theta]

OptimalPath = astar2(Start,Goal(1:2));

figure
axis equal
hold on
plot(row,col,'.k')
plot(OptimalPath(end,1),OptimalPath(end,2),'xb')
plot(OptimalPath(:,1),OptimalPath(:,2),'-r')

%% execution
n=1;
nmax = length(OptimalPath);
tic;
stop_flag = false;
while ~stop_flag
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
    
    local = R_goal*(([OptimalPath(n,1);OptimalPath(n,2)] -...
        [(x+x_length/2)*resolution;(y+y_length/2)*resolution]));
    
    x_local = local(1);
    y_local = local(2);
    bearing_goal = atan2(y_local,x_local);
    
    % update arrival condition
    range_goal = sqrt((OptimalPath(n,1)-(x+x_length/2)*resolution)^2 +...
        ((OptimalPath(n,2)-(y+y_length/2)*resolution))^2);
    e_th = Goal(3) - odom_pose(3);
    
    if n == nmax
        [v,w] = kin_controller(Goal, odom_pose);
        disp('approaching goal ...')
        if range_goal <= 0.1*resolution && e_th < 0.08
            break
        end
    else
        [v,w] = VFH_nav(bearing_goal,laserScan);
        if range_goal <= 0.3*resolution
            n = n + 1;
            range_goal = sqrt((OptimalPath(n,1)-(x+x_length/2)*resolution)^2 +...
            ((OptimalPath(n,2)-(y+y_length/2)*resolution))^2);
        end
    end
    
    plot((x+x_length/2)*resolution,(y+y_length/2)*resolution,'.b');
    
    % send speed command
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(burger,velmsg);
    
end
total_time = toc;

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(burger,velmsg);
