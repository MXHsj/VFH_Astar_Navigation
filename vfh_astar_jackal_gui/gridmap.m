function []=gridmap(x_length,y_length,resolution)
% Version: 0.2

global laserScan pose ranges map

% x_origin=x_length/2;
% y_origin=y_length/2;

ranges(ranges>30) = [];

index = find(ranges>=9 & ranges<=30);

cart=readCartesian(laserScan);
% cart = pol2cart(angles,ranges);
scan_x=cart(:,1);
scan_y=cart(:,2);
scan_x(index) = [];
scan_y(index) = [];

x=pose.Pose.Pose.Position.X;
y=pose.Pose.Pose.Position.Y;
quat=[pose.Pose.Pose.Orientation.W pose.Pose.Pose.Orientation.X pose.Pose.Pose.Orientation.Y pose.Pose.Pose.Orientation.Z];
theta=quat2eul(quat);

R = [cos(theta(1)),-sin(theta(1));sin(theta(1)),cos(theta(1))];
global_frame = R*[scan_x';scan_y']+[x;y];
scan_x_global = global_frame(1:2:end);
scan_y_global = global_frame(2:2:end);

setOccupancy(map,[(scan_x_global*resolution)' (scan_y_global*resolution)'],1);

show(map)
xlabel('X');
ylabel('Y');
