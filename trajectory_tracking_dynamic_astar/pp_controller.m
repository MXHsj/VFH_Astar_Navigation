function [v,w] = pp_controller(goal_pose, odom_pose)

global controller

controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 0.65;
controller.Waypoints = goal_pose(1:2);

[v,w] = controller(odom_pose);

