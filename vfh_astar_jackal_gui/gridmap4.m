function [occval] = gridmap4(laserScan,pose)
% Version: 0.3

global map

ranges = double(laserScan.Ranges);
angles = double(laserScan.readScanAngles);


ranges(ranges>30) = [];

index = find(ranges>=9 & ranges<=30);

cart=readCartesian(laserScan);
% cart = pol2cart(angles,ranges);
scan_x=cart(:,1);
scan_y=cart(:,2);
scan_x(index) = [];
scan_y(index) = [];

odom_pose = odometry(pose);
x = odom_pose(1);
y = odom_pose(2);
theta = odom_pose(3);

R = [cos(theta),-sin(theta);
    sin(theta),cos(theta)];

global_frame = R * [scan_x';scan_y'] + [x;y];

scan_x_global = global_frame(1:2:end);
scan_y_global = global_frame(2:2:end);

setOccupancy(map,[(scan_x_global)' (scan_y_global)'],1);

% pose2=[x, y, theta(1)];
maxrange=double(laserScan.RangeMax);
angles(index)=[];
ranges(index)=[];

% insertRay(map,odom_pose',ranges,angles,maxrange);

occval = occupancyMatrix(map);

imfill(occval,'holes');