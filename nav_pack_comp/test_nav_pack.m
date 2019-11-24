clc;
clear;
close all;
rosshutdown;

ipaddress = 'localhost';
rosinit(ipaddress);

jackal = rospublisher('/jackal_velocity_controller/cmd_vel');
velmsg = rosmessage(jackal);

move_base = rospublisher('/move_base/goal');
nav_goal = rosmessage(move_base);

laser = rossubscriber('/front/scan');

odom = rossubscriber('/odometry/filtered');

move_base_status = rossubscriber('/move_base/status');

% define vfh object
global vfh
vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;
vfh.DistanceLimits = [0 2];
vfh.RobotRadius = 0.35;
vfh.MinTurningRadius = 0.4;
vfh.SafetyDistance = 0.1;
vfh.HistogramThresholds = [1 1];

rate = robotics.Rate(20);
explore_time = 80;
reset(rate);
while rate.TotalElapsedTime < explore_time   
    laserScan = receive(laser);
    [v,w] = VFH_nav(0, laserScan);
    
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
    waitfor(rate);
end

%%
% set goal
pose = receive(odom);
odom_pose = odometry(pose);

nav_goal.Goal.TargetPose.Header.FrameId = 'base_link';
x_global = input('enter target X: ');
y_global = input('enter target Y: ');
theta = input('enter target heading: ');

R = [cos(odom_pose(3)),sin(odom_pose(3));...
       -sin(odom_pose(3)),cos(odom_pose(3))];

goal = R*([x_global; y_global] - [odom_pose(1); odom_pose(2)]);

goal_quat = eul2quat([deg2rad(theta)-odom_pose(3),0,0]);

nav_goal.Goal.TargetPose.Pose.Position.X = goal(1);
nav_goal.Goal.TargetPose.Pose.Position.Y = goal(2);
nav_goal.Goal.TargetPose.Pose.Position.Z = 0;
nav_goal.Goal.TargetPose.Pose.Orientation.W = goal_quat(1);
nav_goal.Goal.TargetPose.Pose.Orientation.X = goal_quat(2);
nav_goal.Goal.TargetPose.Pose.Orientation.Y = goal_quat(3);
nav_goal.Goal.TargetPose.Pose.Orientation.Z = goal_quat(4);

send(move_base,nav_goal);

tolerence = 0.5;
tic
while true
    status = receive(move_base_status);
    cond = status.StatusList.Status;
    pose = receive(odom);
    odom_pose = odometry(pose);
    dist2goal = sqrt((x_global-odom_pose(1))^2+(y_global-odom_pose(2))^2);
    fprintf('distance to goal: %f\n',dist2goal);
    if cond == 3
        disp('complete!');
        break
    end
end
total_time = toc;
disp(total_time);