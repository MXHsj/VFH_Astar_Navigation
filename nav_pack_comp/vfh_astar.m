%Init roscore
rosshutdown;
clc;
clear;
close all;

ipaddress = 'localhost';
rosinit(ipaddress);

global laserScan pose ranges map

laser = rossubscriber('/front/scan');
jackal = rospublisher('/jackal_velocity_controller/cmd_vel');
odom = rossubscriber('/odometry/filtered');
velmsg = rosmessage(jackal);

x_length=50;% meter
y_length=50;
resolution=5;% grid/meter
map = robotics.BinaryOccupancyGrid(x_length*resolution,y_length*resolution,1);
map.GridLocationInWorld=[-x_length*resolution/2,-y_length*resolution/2];
% 
% map = robotics.OccupancyGrid(x_length*resolution,y_length*resolution,1);
% map.GridLocationInWorld=[-x_length*resolution/2,-y_length*resolution/2];


v = 0;
w = 0;

spinVelocity = 0.3;
forwardVelocity = 0.2;
backwardVelocity = -0.02;
distanceThreshold = 0.6;

vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;    
vfh.DistanceLimits = [0 2];
vfh.RobotRadius = 0.3;
vfh.MinTurningRadius = 0.3;
vfh.SafetyDistance = 0.05;
vfh.HistogramThresholds = [1 1];

targetDir = 0;

% h = figure;
distance=10;

r_max = 6.28;
r_min = 0.0;
ob_dist = [];
omod_vel = [];

rate = robotics.Rate(10);

odom_vel_x = [];
odom_vel_z = [];

odom_pos_x = [];
odom_pos_y = [];

previous = 0;
pose = receive(odom);
x0=pose.Pose.Pose.Position.X;
y0=pose.Pose.Pose.Position.Y;

while rate.TotalElapsedTime < 50
    
    % Get laser scan data
    laserScan = receive(laser);
    pose = receive(odom);
   
    ranges = double(laserScan.Ranges);
   
    angles = double(laserScan.readScanAngles);
    
    odom_vel_x = [odom_vel_x, pose.Twist.Twist.Linear.X];
    odom_vel_z = [odom_vel_z, pose.Twist.Twist.Angular.Z];
    odom_pos_x = [odom_pos_x, pose.Pose.Pose.Position.X];
    odom_pos_y = [odom_pos_y, pose.Pose.Pose.Position.Y];
    
%     targetDir = (r_max-r_min).*rand();
%     targetDir=atan((y_dest-odom_pos_y)/(x_dest-odom_pos_x));
%     distance=sqrt((y_dest-odom_pos_y).^2+(x_dest-odom_pos_x).^2);
    
    % Call VFH object to computer steering direction
    steerDir= vfh(ranges, angles, targetDir);
    ob_dist = [ob_dist, min(ranges) ];
    
    % Calculate velocities
    if ~isnan(steerDir) % If steering direction is valid
        desiredV = 0.3;
        w = exampleHelperComputeAngularVelocity(steerDir, 0.65);
        % w(inc) = kp*steerDir;   % proportional control
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.5;
    end
    
    % Infinite Impulse Response Filter
    w = (7/8)*previous + (1/8)*w;
    
    % Assign and send velocity commands
    velmsg.Linear.X = desiredV;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
    
    previous = w;
    
    waitfor(rate);
    
    gridmap(x_length,y_length,resolution);
end

inflate(map,1);

% figure(2)
% show(map)
% title('Binary Occupancy Grid (Inflated)');
% xlabel('X');
% ylabel('Y');

for i=1:1:x_length*resolution
    for j=1:1:y_length*resolution
        occval(i,j)=getOccupancy(map,[i-x_length/2*resolution j-y_length/2*resolution]);
    end
end
x=pose.Pose.Pose.Position.X;
y=pose.Pose.Pose.Position.Y;


Start=[x,y]
StartX=round((x+x_length/2)*resolution);
StartY=round((y+y_length/2)*resolution);

Goal=[x0,y0]
GoalX=round((Goal(1)+x_length/2)*resolution);
GoalY=round((Goal(2)+y_length/2)*resolution);


GoalRegister=int8(zeros(x_length*resolution,y_length*resolution));
GoalRegister(GoalX,GoalY)=1;

Connecting_Distance=16;

OptimalPath=ASTARPATH(StartY,StartX,occval,GoalRegister,Connecting_Distance);

OptimalPath=flipud(OptimalPath);

[row,col] = find(occval == 1);
figure(3)
plot(row,col,'.k')
hold on;axis equal;
plot(GoalX,GoalY,'xb')
plot(OptimalPath(:,1),OptimalPath(:,2),'-r')


%% go to goal

% set arrival condition
n=1;
nmax=length(OptimalPath);
% range_goal = sqrt((OptimalPath(n,1)-(x+20)*5)^2 + ((OptimalPath(n,2)-(y+20)*5))^2)
range_goal=10;
% while range_goal > 0.8
i=0;
previous=0;
while 1
    i=i+1;
    % successively receive from model state
    pose = receive(odom);
    x = pose.Pose.Pose.Position.X;
    y = pose.Pose.Pose.Position.Y;
    W = pose.Pose.Pose.Orientation.W;
    X = pose.Pose.Pose.Orientation.X;
    Y = pose.Pose.Pose.Orientation.Y;
    Z = pose.Pose.Pose.Orientation.Z;
    Orientation = quat2eul([W X Y Z]);
    theta_odom = Orientation(1);
    
    % read from lidar
    scan = receive(laser);
    data = readCartesian(scan);
    x_scan = data(:,1);
    y_scan = data(:,2);
%     distance = sqrt((x_scan).^2 + (y_scan).^2);
%     range_obs = min(distance);
%     obj_index = find(distance == range_obs);
%     bearing_obs = atan2(y_scan(obj_index),x_scan(obj_index));
%     threshold = 1;
    
    % convert goal from global to local
    R_goal = [cos(theta_odom),sin(theta_odom);...
                -sin(theta_odom),cos(theta_odom)];
            
    local = R_goal*(([OptimalPath(n,1);OptimalPath(n,2)] -...
                 [(x+x_length/2)*resolution;(y+y_length/2)*resolution]));
             
    x_local = local(1);
    y_local = local(2);
    bearing_goal = atan2(y_local,x_local);
    
    [v,w] = VFH_nav(bearing_goal,scan);
    % speed control
%     k1 = 5.0;
%     k2 = 6.0;
% 
% 
%     if abs(bearing_goal)>pi/6
%         w1(i) = abs(bearing_goal)/bearing_goal*0.5;
%         v1(i) = 0.1/bearing_goal;
%     else
%         w1(i) = k1*bearing_goal;
%         v1(i) = 0.4;
%     end
%     
%     v1(i) = (6/8)*previous + (2/8)*v1(i);
% 
%     if w1(i) > 0.5
%         w1(i) = 0.5;
%     elseif w1(i) < -0.5
%         w1(i) = -0.5;
%     end
%     
%     if v1(i) > 0.5
%         v1(i) = 0.5;
%     elseif v1(i) < 0
%         v1(i) = 0;
%     end
%     previous=v1(i);
%     
    hold on;
    plot((x+x_length/2)*resolution,(y+y_length/2)*resolution,'.b');
    axis equal;
    
    % send speed command
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
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
% figure
% i=1:i;
% plot(i,v1,'r',i,w1,'b')
rosshutdown