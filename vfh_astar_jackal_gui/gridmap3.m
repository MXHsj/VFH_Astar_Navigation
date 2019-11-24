function [pose3] = gridmap3(map,resolution,pose0)
% Version: 0.3

global laserScan pose ranges angles

ranges(ranges>30) = [];

index = find(ranges>=9 & ranges<=30);

cart=readCartesian(laserScan);
% cart = pol2cart(angles,ranges);
scan_x=cart(:,1);
scan_y=cart(:,2);
scan_x(index) = [];
scan_y(index) = [];

x=pose.Pose.Pose.Position.X - pose0(1);
y=pose.Pose.Pose.Position.Y + pose0(2);
quat=[pose.Pose.Pose.Orientation.W pose.Pose.Pose.Orientation.X ...
    pose.Pose.Pose.Orientation.Y pose.Pose.Pose.Orientation.Z];
theta=quat2eul(quat);
theta = theta(1);

R = [cos(theta),-sin(theta);sin(theta),cos(theta)];
global_frame = R*[scan_x';scan_y']+[x;y];
scan_x_global = global_frame(1:2:end);
scan_y_global = global_frame(2:2:end);

updateOccupancy(map,[(scan_x_global*resolution)' (scan_y_global*resolution)'],0.7);

pose2=[x*resolution, y*resolution, theta(1)];
maxrange=double(laserScan.RangeMax);
angles(index)=[];
ranges(index)=[];

insertRay(map,pose2,ranges*resolution,angles,maxrange*resolution);
show(map)
xlabel('X');
ylabel('Y');

pose3=round(-(pose2(1:2)-[x_length*resolution+1,y_length*resolution+1]));