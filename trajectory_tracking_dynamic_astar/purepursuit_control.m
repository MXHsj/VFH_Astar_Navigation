%% initialize
clc;
clear;
close all;
rosshutdown;

ipaddress = 'localhost';
rosinit(ipaddress);

burger = rospublisher('/cmd_vel');
velmsg = rosmessage(burger);
odom = rossubscriber('/odom');

pose = receive(odom);
quat = [pose.Pose.Pose.Orientation.W, ...
    pose.Pose.Pose.Orientation.X, ...
    pose.Pose.Pose.Orientation.Y, ...
    pose.Pose.Pose.Orientation.Z];
eul = quat2eul(quat);

pose0 = [pose.Pose.Pose.Position.X; ...
    pose.Pose.Pose.Position.Y; ...
    eul(1)];

%% test pure pursuit
goal_pose = [1, -1, 0];

rate = robotics.Rate(100);
run_time = 20;
reset(rate);

controller = robotics.PurePursuit;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 0.6;
controller.Waypoints = goal_pose(1:2);

tolerence = 0.08;
dist2goal_rec = [];
x_record = [];
y_record = [];
v_record = [];
w_record = [];

while rate.TotalElapsedTime < run_time
    
    pose = receive(odom);
    odom_pose = odometry(pose);
    
    [v,w] = controller(odom_pose);
    
    dist2goal = ...
        sqrt((odom_pose(1)-goal_pose(1))^2 ...
        + (odom_pose(2)-goal_pose(2))^2);
    
    dist2goal_rec = [dist2goal_rec, dist2goal];
    x_record = [x_record, odom_pose(1)];
    y_record = [y_record, odom_pose(2)];
    
    fprintf('distance to goal: %f\n',dist2goal);
    if dist2goal < tolerence
        v = 0;
        w = 0;
        % break
    end
    
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(burger,velmsg);
    
    v_record = [v_record, v];
    w_record = [w_record, w];
    
    waitfor(rate);
    
end

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(burger,velmsg);

total_time = linspace(0,rate.TotalElapsedTime,length(x_record));
% figure  % plot error
% plot(total_time,dist2goal_rec,'g','LineWidth',1.5)
% grid on
% xlabel('time [sec]')
% ylabel('error')
% legend('distance error [m]')
 
% figure  % plot trajectory
% plot(x_record,y_record,'k','LineWidth',1.5)
% hold on
% plot(goal_pose(1),goal_pose(2),'*b');
% grid on
% xlabel('x [m]')
% ylabel('y [m]')
% title('X-Y trajectory')

figure  % plot velocity
plot(total_time,v_record,'b','LineWidth',1.5)
hold on
plot(total_time,w_record,'r','LineWidth',1.5)
grid on
xlabel('x [m]')
ylabel('velocity')
legend('linear vel [m/s]','angular vel [rad/s]');