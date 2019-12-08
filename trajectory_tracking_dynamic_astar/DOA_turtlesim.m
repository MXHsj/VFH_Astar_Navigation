clc;
clear;
close all;

rosshutdown;    
ipaddress = 'localhost';
rosinit(ipaddress);

turtle1 = rospublisher('/turtle1/cmd_vel');
turtle1_velmsg = rosmessage(turtle1);

turtle1_position = rossubscriber('/turtle1/pose');
pose1 = receive(turtle1_position);

obstacle1 = rospublisher('/obstacle1/cmd_vel');
obs1_velmsg = rosmessage(obstacle1);
obstacle1_position = rossubscriber('/obstacle1/pose');

obstacle2 = rospublisher('/obstacle2/cmd_vel');
obs2_velmsg = rosmessage(obstacle2);
obstacle2_position = rossubscriber('/obstacle2/pose');

v = 0.2;
w = 0.2;

goal = [2; 10];

dist2goal = sqrt((pose1.X-goal(1))^2+(pose1.Y-goal(2))^2);
tolerence = 0.2;

DOA_flag = false;

while dist2goal >= tolerence
    
    %% obstacle1 control
    obs1_pose  = receive(obstacle1_position);
    
    if obs1_pose.X > 1 && obs1_pose.X < 10
        obs1_v = 0.5;
        obs1_w = 0;
    else
        obs1_v = 0;
        obs1_w = 0;
    end
    
    obs1_velmsg.Linear.X = obs1_v;
    obs1_velmsg.Angular.Z = obs1_w;
    send(obstacle1,obs1_velmsg);
    
    %% obstacle2_control
    obs2_pose  = receive(obstacle2_position);
    
    if obs2_pose.X > 1 && obs2_pose.X < 10
        obs2_v = 0.5;
        obs2_w = 0;
    else
        obs2_v = 0;
        obs2_w = 0;
    end
    
    obs2_velmsg.Linear.X = obs2_v;
    obs2_velmsg.Angular.Z = obs2_w;
    send(obstacle2,obs2_velmsg);
    
    %% turtle1 goal control
    pose1 = receive(turtle1_position);
    
    R = [cos(pose1.Theta), sin(pose1.Theta); ...
            -sin(pose1.Theta), cos(pose1.Theta)];
    
    goal_turtle = R*(goal - [pose1.X; pose1.Y]);
    
    rad_safe = 2;
    dist2obs1 = sqrt((pose1.X - obs1_pose.X)^2 + (pose1.Y - obs1_pose.Y)^2);
    dist2obs2 = sqrt((pose1.X - obs2_pose.X)^2 + (pose1.Y - obs2_pose.Y)^2);
    
    if dist2obs1 <= rad_safe || dist2obs2 <= rad_safe
        DOA_flag = true;
    else
        DOA_flag = false;
    end
    
    kp1 = 0.05;
    kp2 = 0.01;
    
    if DOA_flag
        obs1_turtle = R*([obs1_pose.X; obs1_pose.Y] - [pose1.X; pose1.Y]);
        obs2_turtle = R*([obs2_pose.X; obs2_pose.Y] - [pose1.X; pose1.Y]);
        
        if dist2obs1 < dist2obs2
%             A = 1/dist2obs1;
%             w = A * (- kp*rad2deg(atan2(obs1_turtle(2),obs1_turtle(1)))) + ...
%                     (1-A) * (- kp*rad2deg(atan2(obs2_turtle(2),obs2_turtle(1))));
                w = - kp2*rad2deg(atan2(obs1_turtle(2),obs1_turtle(1)));
        else
%             A = 1/dist2obs2;
%             w = A * (- kp*rad2deg(atan2(obs2_turtle(2),obs2_turtle(1)))) + ...
%                     (1-A) * (- kp*rad2deg(atan2(obs1_turtle(2),obs1_turtle(1))));
                w = - kp2*rad2deg(atan2(obs2_turtle(2),obs2_turtle(1)));
        end
    else
        w = kp1*rad2deg(atan2(goal_turtle(2),goal_turtle(1)));
    end
    
    v = 0.5;
    
    turtle1_velmsg.Linear.X = v;
    turtle1_velmsg.Angular.Z = w;
    
    fprintf('x: %f y: %f theta: %f\n', pose1.X, pose1.Y, pose1.Theta);
    
    dist2goal = sqrt((pose1.X-goal(1))^2+(pose1.Y-goal(2))^2);
    
%     fprintf('w: %f d: %f\n', w, dist2goal);

    send(turtle1,turtle1_velmsg);
    
end
