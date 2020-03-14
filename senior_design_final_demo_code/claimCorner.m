%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: This function detects corner and returns location of the
% corner
% Version: 1.0
% Author: Xihan Ma
% Input: rgb image from kinect
% Output: corner flag, corner location
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [iscorner,goal_pose] = claimCorner(Img,pose)

global laserScan

% bw(:,:) = Img(:,:,1)>120 & Img(:,:,2)<90 & Img(:,:,3)<90; % red corner night
bw(:,:) = Img(:,:,1)>100 & Img(:,:,2)<60 & Img(:,:,3)<60; % red corner day

scan_angle = linspace(-135,135,length(laserScan.Ranges));
front_index = scan_angle >= -10 & scan_angle <= -5;   % front region
front_dist = mean(laserScan.Ranges(front_index));  % front distance

% image filtering
bw = medfilt2(bw,[10 10]);      % mediam filtering
bw2 = bwareaopen(bw,500,8);    % delete isolate pixels
bw2 = imfill(bw2,'holes');

% receive current pose
curr_pose = odometry(pose);
wall_area = sum(sum(bw2 == 1));
if  wall_area > 5000 && front_dist <= 1.7
    iscorner = true;
%     goal_pose = curr_pose(1:2);
    %
    theta = curr_pose(3);
    ranges = laserScan.Ranges(front_index);
    scan_x = ranges.*cos(scan_angle(front_index)');
    scan_y = ranges.*sin(scan_angle(front_index)');
    
    R = [cos(theta),-sin(theta);
        sin(theta),cos(theta)];
    
    global_frame = R * [scan_x';scan_y'] + [curr_pose(1);curr_pose(2)];
    scan_x_global = mean(global_frame(1:2:end));
    scan_y_global = mean(global_frame(2:2:end));
    goal_pose = [((0/6)*curr_pose(1)+(6/6)*scan_x_global);...
      ((0/6)*curr_pose(2)+(6/6)*scan_y_global)];
    % goal_pose = [2*scan_x_global-curr_pose(1);2*scan_y_global-curr_pose(2)];
else
    iscorner = false;
    goal_pose = NaN;
end
fprintf('corner flag: %d area: %d dist: %f\n',iscorner,wall_area,front_dist);

