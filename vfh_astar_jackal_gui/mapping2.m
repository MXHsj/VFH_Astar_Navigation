function [occval] = mapping2(laserScan,pose)
% Version: 0.5

global map

ranges = double(laserScan.Ranges);
angles = double(laserScan.readScanAngles);

cart=readCartesian(laserScan, 'RangeLimits', [0 10]);
scan_x=cart(:,1);
scan_y=cart(:,2);

odom_pose = odometry(pose);
x = odom_pose(1);
y = odom_pose(2);
theta = odom_pose(3);

R = [cos(theta),-sin(theta);
    sin(theta),cos(theta)];

global_frame = R * [scan_x';scan_y'] + [x;y];

scan_x_global = global_frame(1:2:end);
scan_y_global = global_frame(2:2:end);

updateOccupancy(map,[(scan_x_global)' (scan_y_global)'],0.7);

% maxrange=double(laserScan.RangeMax);

insertRay(map,odom_pose',ranges,angles,10);

occval = occupancyMatrix(map,"ternary");

imfill(occval,'holes');
