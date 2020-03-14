function [v,w,dist2goal] = trajectory3(goal_pose,pose)

global laserScan

curr_pose = odometry(pose);

R_goal = [cos(curr_pose(3)),sin(curr_pose(3));...
          -sin(curr_pose(3)),cos(curr_pose(3))];

local = R_goal*(([goal_pose(1);goal_pose(2)] -...
    [curr_pose(1);curr_pose(2)]));

x_local = local(1);
y_local = local(2);
targetDir = atan2(y_local,x_local);

targetDir = double(targetDir);

[v,w] = VFH_nav(targetDir,laserScan);

dist2goal = ...
    sqrt((curr_pose(1)-goal_pose(1))*(curr_pose(1)-goal_pose(1))...
    +(curr_pose(2)-goal_pose(2))*(curr_pose(2)-goal_pose(2)));
        