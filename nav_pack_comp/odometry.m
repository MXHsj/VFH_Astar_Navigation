%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: This function takes the initial pose of robot and outputs 
% the current pose over the iterations
% Version: 1.0
% Author: Xihan Ma
% Input: initial pose of the robot 
% Output: current pose of the robot in column vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [odom_pose] = odometry(pose)

odom_pos_x = pose.Pose.Pose.Position.X;
odom_pos_y = pose.Pose.Pose.Position.Y;

quat = [pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
    pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z];

odom_theta = quat2eul(quat);
odom_theta = odom_theta(1);

odom_pose = [odom_pos_x; odom_pos_y; odom_theta];
