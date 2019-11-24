function [occval] = mapping2temp(laserScan,pose)
% Version: 0.5

global map

ranges = double(laserScan.Ranges);
angles = double(laserScan.readScanAngles);

ranges(ranges>30) = [];

index = find(ranges>=9 & ranges<=30);

cart=readCartesian(laserScan);

scan_x=cart(:,1);
scan_y=cart(:,2);
scan_x(index) = [];
scan_y(index) = [];

x = pose.Pose(16).Position.X;
y = pose.Pose(16).Position.Y;
W = pose.Pose(16).Orientation.W;
X = pose.Pose(16).Orientation.X;
Y = pose.Pose(16).Orientation.Y;
Z = pose.Pose(16).Orientation.Z;
Orientation = quat2eul([W X Y Z]);
theta = Orientation(1);
odom_pose = [x;y;theta];

R = [cos(theta),-sin(theta);
    sin(theta),cos(theta)];

global_frame = R * [scan_x';scan_y'] + [x;y];

scan_x_global = global_frame(1:2:end);
scan_y_global = global_frame(2:2:end);

updateOccupancy(map,[(scan_x_global)' (scan_y_global)'],0.7);

% pose2=[x, y, theta(1)];
maxrange=double(laserScan.RangeMax);
angles(index)=[];
ranges(index)=[];

insertRay(map,odom_pose',ranges,angles,maxrange);

occval = occupancyMatrix(map,"ternary");

imfill(occval,'holes');