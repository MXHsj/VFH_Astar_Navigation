function [v,w] = kin_controller(goal_pose, odom_pose)

Kp = 0.2;
Ka = 4.6;
Kb = -1.8;

beta = -atan2(goal_pose(2)-odom_pose(2), ...
    goal_pose(1)-odom_pose(1));
beta = rem((beta + pi),2*pi) - pi;

alpha = -beta - odom_pose(3);
alpha = rem((alpha + pi),2*pi) - pi;

rho = sqrt((odom_pose(1)-goal_pose(1))^2 ...
    + (odom_pose(2)-goal_pose(2))^2);

% control law
v = Kp*rho;
w = Ka*alpha + Kb*beta;

if v > 0.3
    v = 0.3;
elseif v < -0.3
    v = -0.3;
end

if w > 0.65
    v = (0.65/w)*v;
    w = 0.65;
elseif w < -0.65
    v = (-0.65/w)*v;
    w = -0.65;
end