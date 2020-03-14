%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% senior design game main function
% Version: 0.5
% Date: Feb/25th/2018
% Description: based on game rule1, first wall following
% team member: Xihan Ma, Song Cui, Honglin Sun, Boqun Yin, Xiaoyu Hou
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;
rosshutdown;

ipaddress = 'localhost';
rosinit(ipaddress);

% define global varaibles & ROS topics
global velmsg pose kinect_color_msg map vfh pose0 laserScan

jackal = rospublisher('/jackal_velocity_controller/cmd_vel');
velmsg = rosmessage(jackal);

odom = rossubscriber('/odometry/filtered');
pose = receive(odom);

kinect_color = rossubscriber('/kinect2/qhd/image_color_rect');
kinect_color_msg=rosmessage(kinect_color);

laser = rossubscriber('/front/scan');

% receive robot initial pose from odometry
odom_init_pos_x = pose.Pose.Pose.Position.X;
odom_init_pos_y = pose.Pose.Pose.Position.Y;
init_quat = [pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X...
    pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z];
odom_init_theta = quat2eul(init_quat);
odom_init_theta = odom_init_theta(1);

% pose0 = [odom_init_pos_x; odom_init_pos_y; odom_init_theta];
pose0 = [0; 0; 0];

% define ROS time
% rate = robotics.Rate(20);

% define vfh object
vfh = robotics.VectorFieldHistogram;

% define map
x_length=120;    % meter
y_length=120;
resolution=5;   % grid/meter
map = robotics.OccupancyGrid(x_length,y_length,resolution);
map.GridLocationInWorld=[-x_length/2,-y_length/2];
% map.OccupiedThreshold=0.8;

% define initial value for loop-changing variables
pose_x = [];
pose_y = [];
pose_theta = [];

% read template
rectangle_temp = imread('red_square.JPG');    % detect circle
circle_temp = imread('red_ball.JPG');    % detect rectangle-alike

circle_template = template_processing(circle_temp);
rectangle_template = template_processing(rectangle_temp);

% initialize motor (0.1 close 1 open)
ard=arduino('/dev/ttyACM1','Uno','Libraries','Servo');
ser=servo(ard,'D9');
writePosition(ser,0.1);

%% wall following + claim corner
% initial variables
% iscorner = false;

% while 1
%     dist2obj = readVoltage(ard,'A1');
%     fprintf('dist2obj: %f\n',dist2obj);
% end

% while rate.TotalElapsedTime < 1000
 while 1
    pose = receive(odom);
    laserScan = receive(laser);
    color=receive(kinect_color);
    Img = kinect2RGM(color);
    Img = Img(1:200,:,:);   % avoid seeing objects
%     imshow(Img,'InitialMagnification','fit');
    [iscorner,goal_pose] = claimCorner(Img,pose);
    if iscorner
        break;
    end
    laserScan = receive(laser);
    [v,w] = wall_follower(laserScan);
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    % send(jackal,velmsg);
    
end
fprintf('corner_pose_x: %f corner_pose_y: %f\n',goal_pose(1),goal_pose(2));

%% fetch objects
% initial variables
pose=receive(odom);

w_prev = 0;
kwp = 0.005;
kwd = 0.002;
error_prev = 0;

catch_flag = false;
iscorner = false;
pathplan_flag = false;
trackwall_flag = false;
arrive_flag = true;

r_min = 0.0;
r_max = 6.28;

dist2obj = inf;

w = 0;
v = 0;
% while rate.TotalElapsedTime < 1000
while 1
    laserScan = receive(laser);
    pose = receive(odom);
    odom_pose = odometry(pose);
    
    scan_angle = linspace(-135,135,length(laserScan.Ranges));
    front_index = scan_angle >= -30 & scan_angle <= 30;   % front region
    front_dist = min(laserScan.Ranges(front_index));  % front distance
    
    mapping2(laserScan,pose);
    
%     targetDir = r_min + (r_max-r_min).*rand();
    targetDir = 0;
    [v,w] = VFH_nav(targetDir,laserScan);
    w = IIRfilter(w,w_prev);
    w_prev = w;
    
    color=receive(kinect_color);
    Img = kinect2RGM(color);
    Img_obj = Img(200:end,:,:);
    [circle_matrix] = ...
        template_match2_redball(Img_obj, circle_template, rectangle_template);
    [square_matrix] = ...
        template_match2_bluesquare(Img_obj, circle_template, rectangle_template);
    
    % ball prior to square
    if ~isnan(circle_matrix) & ~isnan(square_matrix)
        square_matrix = NaN;
    end
    
    % track red ball if detected
    if ~isnan(circle_matrix) & ~catch_flag
%         subplot(1,2,1);
%         imshow(Img,'InitialMagnification','fit');
        writePosition(ser,1);
        img_center_row = size(Img_obj,1)/2;
        img_center_col = size(Img_obj,2)/2 - 100;
        [~,c] = find(circle_matrix == 1);
        c_center = mean(c);
        if front_dist >= 0.8   % if not running into obstacle
            error = img_center_col - c_center;
            errord = error - error_prev;
            w = kwp*error + kwd*errord;
            if w > 0.45
                w = 0.45;
            elseif w < -0.45
                w = -0.45;
            end
            v = 0.2;
            
            % subplot(1,2,2);
            % imshow(circle_matrix,'InitialMagnification','fit');
            % depth = area2depth(circle_matrix);
            depth = readVoltage(ard,'A1');
            fprintf('depth: %f, error: %f\n',depth,error);
            
            if abs(depth-3)<=0.7 && abs(error) <= 65 % day 300 65 night 300 50
                catch_flag = true;
            else
                catch_flag = false;
            end
            error_prev = error;
        end
    end
    
    % track blue square if detected
    if ~isnan(square_matrix) & ~catch_flag
%         subplot(1,2,1);
%         imshow(Img,'InitialMagnification','fit');
        writePosition(ser,1);
        img_center_row = size(Img_obj,1)/2;
        img_center_col = size(Img_obj,2)/2 - 100;
        [~,c] = find(square_matrix == 1);
        c_center = mean(c);
        if front_dist >= 1.2   % if not running into obstacle
            error = img_center_col - c_center;
            errord = error - error_prev;
            w = kwp*error + kwd*errord;
            if w > 0.45
                w = 0.45;
            elseif w < -0.45
                w = -0.45;
            end
            v = 0.2;
            
            % subplot(1,2,2);
            % imshow(circle_matrix,'InitialMagnification','fit');vc
            % depth = area2depth(square_matrix);
            depth = readVoltage(ard,'A1');
            fprintf('depth: %f, error: %f\n',depth,error);
            
            if abs(depth-3.0)<=0.5 && abs(error) <= 100 % day 250 50 night 300 50
                catch_flag = true;
            else
                catch_flag = false;
            end
            error_prev = error;
        end
    end
    
    if isnan(square_matrix) & isnan(circle_matrix) & ~catch_flag
        writePosition(ser,0.1);
    end
    
    if catch_flag && ~pathplan_flag
        writePosition(ser,0.1);     % close gate
        
        map_temp=copy(map);
        inflate(map_temp,0.2);
        
        [OptimalPath] = astar2_1(odom_pose(1:2)',goal_pose',map_temp);
        OptimalPath_global = astar2global2(OptimalPath);
        arrive_flag = false;
        pathplan_flag = true;
        % astarpathshow(OptimalPath);
    end
    
    if catch_flag && pathplan_flag && ~arrive_flag
        Img_corner =  Img(1:190,:,:); % if sees the corner
        iscorner = go2Corner(Img_corner);
        if iscorner
            trackwall_flag = true;
        elseif ~iscorner
            travelwall_flag = false;
        end
        if trackwall_flag
            [v,w,dist2goal] = trackCorner(Img_corner);
        else
            [v,w,dist2goal] = trajectory3(goal_pose,pose);
        end
        disp('going back...');
        % pose1=[odom_pose(1),odom_pose(2)];
        % pose2=global2astar2(pose1);
        % plot(pose2(1),pose2(2),'.b')
    end
    
    if ~arrive_flag
%         pathplan_flag = true;
        if dist2goal<0.25
            arrive_flag = true;
            pathplan_flag = false;
            catch_flag = false;
            trackwall_flag = false;
            writePosition(ser,1);   % open gate
            tic;
            while toc < 2.2              % backwards
                velmsg.Linear.X = -0.3;
                velmsg.Angular.Z = 0;
                send(jackal,velmsg);
            end
            writePosition(ser,0.1);
            tic;
            while toc < 6
                velmsg.Linear.X = 0;        % spin
                velmsg.Angular.Z = 0.5;
                send(jackal,velmsg);
            end
            
        end
    end
    
    %     occmapshow(occval);
    
    if isnan(w)
        w = 0;
    end
    
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
    
end
% figure;
% occmapshow(occval);