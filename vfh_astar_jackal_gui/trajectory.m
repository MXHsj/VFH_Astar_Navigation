function [dist2goal,lookahead] = trajectory(waypoints)

global pose0 controller

controller = robotics.PurePursuit('DesiredLinearVelocity',0.3, ...
            'MaxAngularVelocity',0.65,'Waypoints',waypoints, ...
            'LookaheadDistance',1);

curr_pose = odometry(pose0);
[~,~,lookahead] = controller(curr_pose);

dist2goal = ...
    sqrt((curr_pose(1)-waypoints(end,1))*(curr_pose(1)-waypoints(end,1))...
    +(curr_pose(2)-waypoints(end,2))*(curr_pose(2)-waypoints(end,2)));
        