%% initialize
clc; clear; close all
rosshutdown;

ipaddress = 'localhost';
rosinit(ipaddress);

burger = rospublisher('/cmd_vel');
velmsg = rosmessage(burger);
laser = rossubscriber('/scan');
odom = rossubscriber('/odom');

global x_length y_length resolution map
x_length = 15;    % meter
y_length = 15;
resolution = 5; % grid/meter
map = robotics.OccupancyGrid(x_length,y_length,resolution);
map.GridLocationInWorld=[-x_length/2,-y_length/2];
map.OccupiedThreshold=0.75;

global vfh
vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;
vfh.DistanceLimits = [0 1.5];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.48;
vfh.SafetyDistance = 0.12;
vfh.HistogramThresholds = [1 1];

rate = robotics.Rate(20);

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
reset(rate);
figure;
while rate.TotalElapsedTime < 50
    
    laserScan = receive(laser);
    pose = receive(odom);
    [v,w] = VFH_nav(0, laserScan);
    
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(burger,velmsg);
    
    occval=mapping2(laserScan,pose);
    show(map)
    
    waitfor(rate);
end

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(burger,velmsg);

occval=fliplr(occval');
index=occval==0;
occval(occval==1)=0;
occval(index)=1;
[row,col] = find(occval == 0);
inflate(map,0.2);

%% motion planning
% path planning
pose = receive(odom);

Start = [pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y]
goal_x = input('enter goal position x:  ');
goal_y = input('enter goal position y:  ');
goal_theta = input('enter goal heading theta: ');
Goal = [goal_x, goal_y, goal_theta]

OptimalPath = astar2(Start,Goal(1:2));
[globalx, globaly] = astar2global2(OptimalPath);

figure;
figHandles = findobj('Type', 'figure');
ax1 = axes('parent', figHandles(1));
axis(ax1,'equal');
hold(ax1,'on');
plot(ax1,row,col,'.k')
plot(ax1,OptimalPath(:,1),OptimalPath(:,2),'-r')
plot(ax1,OptimalPath(1,1),OptimalPath(1,2),'xm')
plot(ax1,OptimalPath(end,1),OptimalPath(end,2),'xb')

% trajectory planning
range_goal = sqrt((goal_x-Start(1))*(goal_x-Start(1))+(goal_y-Start(2))*(goal_y-Start(2)));
avg_vel = 0.5;
% tf = 1.2*range_goal/0.5;
tf = 30;
num_samp = length(OptimalPath);
time_samp = 0:tf/(num_samp-1):tf;

order = 5;
A = zeros(2*num_samp,order+1);
A1 = zeros(num_samp,order+1);
A2 = zeros(num_samp,order+1);
for i = 1:num_samp
    for j = 1:order+1
        A1(i,j) = time_samp(i)^(j-1);
    end
end
for i = 1:num_samp
    for j = 2:order+1
        A2(i,j) = (j-1)*time_samp(i)^(j-2);
    end
end
A(1:2:2*num_samp-1,:) = A1;
A(2:2:2*num_samp,:) = A2;

vd_x = avg_vel;
vd_y = avg_vel;

bx = vd_x*ones(2*num_samp,order+1);
bx(1:2:2*num_samp-1) = globalx.*ones(num_samp,1);
bx(2) = 0; bx(end) = 0;

by = vd_y*ones(2*num_samp,order+1);
by(1:2:2*num_samp-1) = globaly.*ones(num_samp,1);
by(2) = 0; by(end) = 0;

ax = A\bx;
ay = A\by;

%% execution
t = 0;
vfh_flag = false;
reset(rate);
rate = robotics.Rate(60);
v_rec = [];
w_rec = [];
e_rec = [];
while rate.TotalElapsedTime <= 1.8*tf
    
    % current reference trajectory
    t = rate.TotalElapsedTime;
    if t <= tf
        x_traj = ax(1) + ax(2)*t + ax(3)*t^2 + ax(4)*t^3 + ax(5)*t^4 + ax(6)*t^5;
        y_traj = ay(1) + ay(2)*t + ay(3)*t^2 + ay(4)*t^3 + ay(5)*t^4 + ay(6)*t^5;
        dx_traj = ax(2) + 2*ax(3)*t + 3*ax(4)*t^2 + 4*ax(5)*t^3 + 5*ax(6)*t^4;
        dy_traj = ay(2) + 2*ay(3)*t + 3*ay(4)*t^2 + 4*ay(5)*t^3 + 5*ay(6)*t^4;
        ddx_traj = 2*ax(3) + 6*ax(4)*t + 12*ax(5)*t^2 + 20*ax(6)*t^3;
        ddy_traj = 2*ay(3) + 6*ay(4)*t + 12*ay(5)*t^2 + 20*ay(6)*t^3;
        theta_traj = atan2(dy_traj,dx_traj);
    end
    
    % receive from sensors
    pose = receive(odom);
%     occval=mapping2(laserScan,pose);
%     show(map)
    odom_pose = odometry(pose);
    x = odom_pose(1);
    y = odom_pose(2);
    theta = odom_pose(3);
    laserScan = receive(laser);
    
    % convert waypoint from global to local
    R = [cos(theta),sin(theta), 0; ...
        -sin(theta),cos(theta), 0; ...
        0, 0, 1];
    local = R(1:2,1:2)*(([x_traj; y_traj] - [x; y]));
    bearing_wp = atan2(local(2),local(1));
    
    % update goal arrival condition
    range_goal = sqrt((goal_x-x)*(goal_x-x)+(goal_y-y)*(goal_y-y));
    
    % calculate error
    e = R*[x_traj-x; y_traj-y; theta_traj-theta];
    e_rec = [e_rec, e];
    
    vd = sqrt(dx_traj*dx_traj+dy_traj*dy_traj);
    if vd <= 0.01
        vd = 0.01;      % linearization does not work for 0 vd
    end
    wd = (ddy_traj*dx_traj-ddx_traj*dy_traj)/ ...
        (dx_traj*dx_traj+dy_traj*dy_traj);
    
    % approxiemate linearization control law
    ksai = 0.707;       % damping ratio
    a = 0.8;          % natual frequency
    k1 = 2*ksai*a;
    k3 = k1;
    k2 = (a*a - wd*wd) / vd;
    u1 = -k1*e(1);
    u2 = -k2*e(2) - k3*e(3);
    
%     % non-linear control law
%     ksai = 0.707;       % damping ratio
%     a = 1.0;          % natual frequency
%     k1 = 2*ksai*a;
%     k3 = k1;
%     k2 = 0.5;
%     u1 = -k1*e(1);
%     u2 = -k2*vd*sin(e(3))/e(3)*e(2) - k3*e(3);
    
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);
    steerDir= vfh(ranges, angles, bearing_wp);
    
    % pick controller
    if min(ranges)<0.4 && ~vfh_flag
        vfh_flag = true;
        if ~isnan(steerDir) % If steering direction is valid
            v = 0.3;
            w = exampleHelperComputeAngularVelocity(steerDir, 0.65);
        else
            v = 0.0; w = 0.65;
        end
        fprintf('switch to vfh, Dir: %f, dist: %f\n',abs(steerDir), min(ranges));
    else
        if min(ranges)>=0.4 && vfh_flag
            vfh_flag = false;
            disp('switch to velocity controller')
        else
            if ~isnan(steerDir) % If steering direction is valid
                v = 0.3;
                w = exampleHelperComputeAngularVelocity(steerDir, 0.65);
            else
                v = 0.0; w = 0.65;
            end
        end
    end
    if ~vfh_flag
        v = vd*cos(e(3)) - u1;
        w = wd - u2;
    end
    
    % fprintf('v: %f, w: %f\n',v,w);
    
    if range_goal <= 0.1 && e(3) < 0.1
        disp('reached!')
        break
    end
    
    plot(ax1,(x+x_length/2)*resolution,(y+y_length/2)*resolution,'.b');
    
    % velocity saturation
    if v > 0.6
        v = 0.6;
    elseif v < -0.6
        v = -0.6;
    end
    if w > 0.65
        v = (0.65/w)*v;
        w = 0.65;
    elseif w < -0.65
        v = (-0.65/w)*v;
        w = -0.65;
    end
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(burger,velmsg);
    v_rec = [v_rec, v];
    w_rec = [w_rec, w];
    
    waitfor(rate);
end

grid(ax1,'on');
legend(ax1,'obstacle','planned path','start','goal','trajectory','Location','southeast')
total_time = t

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(burger,velmsg);

figure
time = 0:1/60:(length(v_rec))*1/60;
plot(time,e_rec(1,:),'-r');
hold on
plot(time,e_rec(2,:),'-g');
plot(time,e_rec(3,:),'-b');
legend('e1','e2','e3');