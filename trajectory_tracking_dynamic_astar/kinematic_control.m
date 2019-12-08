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

%% test kinematics controller
goal_pose = [-1, 1, 0];

rate = robotics.Rate(100);
run_time = 60;
reset(rate);

Kp = 0.2;
Ka = 4.6;
Kb = -1.8;

e_th_record = [];
rho_record = [];
x_record = [];
y_record = [];
v_record = [];
w_record = [];

while rate.TotalElapsedTime < run_time
    
    pose = receive(odom);
    odom_pose = odometry(pose);
    
    beta = -atan2(goal_pose(2)-odom_pose(2), ...
        goal_pose(1)-odom_pose(1));
    beta = rem((beta + pi),2*pi) - pi;
    
    alpha = -beta - odom_pose(3);
    alpha = rem((alpha + pi),2*pi) - pi;
    
    rho = sqrt((odom_pose(1)-goal_pose(1))^2 ...
        + (odom_pose(2)-goal_pose(2))^2);
    
    % control law
    v = Kp*rho;
    w = Ka*alpha + Kb*beta;
    
    if v > 0.4
        v = 0.4;
    elseif v < -0.4
        v = -0.4;
    end
    
    if w > 0.65
        v = (0.65/w)*v;
        w = 0.65;
    elseif w < -0.65
        v = (-0.65/w)*v;
        w = -0.65;
    end
    
    fprintf('v: %f w: %f\n',v,w)
    % fprintf('alpha: %f beta: %f\n',(alpha), (beta));
    
    e_th = goal_pose(3) - odom_pose(3);
    
    e_th_record = [e_th_record, e_th];
    rho_record = [rho_record, rho];
    x_record = [x_record, odom_pose(1)];
    y_record = [y_record, odom_pose(2)];
    
    if rho <= 0.05 && abs(e_th) <= 0.08
        disp('goal reached')
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

total_time = linspace(0,rate.TotalElapsedTime,length(rho_record));
% figure  % plot error
% plot(total_time,rho_record,'g','LineWidth',1.5)
% hold on
% plot(total_time,e_th_record,'r','LineWidth',1.5)
% grid on
% xlabel('time [sec]')
% ylabel('error')
% legend('distance error [m]','heading angle error [rad]')

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