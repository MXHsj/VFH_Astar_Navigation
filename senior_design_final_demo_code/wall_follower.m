function [v,w] = wall_follower(laserScan)

% scan data for wall following
scan_angle = linspace(-135,135,length(laserScan.Ranges));

% hokuyo sector
front_index = scan_angle >= -10 & scan_angle <= 0;   % front region
right_index = scan_angle >= -135 & scan_angle <= -45; % right region
% left_index = scan_angle >= 45 & scan_angle <= 135;    % left region

front_dist = min(laserScan.Ranges(front_index));  % front distance
right_dist = mean(laserScan.Ranges(right_index)); % right distance
% left_dist = mean(laserScan.Ranges(left_index));   % left distance

% set initial velocity
v = 0.4;
w = 0;

% set param for speed control
threshold = 0.60;
error = right_dist(1) - threshold;
kw = 3.0;
pk = 0.1;
P = pk*error;

if front_dist(1) <= 1.7   % threshold of seeing the wall ahead
    v = 0.35;
    % v = kv/front_dist;
    w = kw/front_dist(1);
    % fprintf('front: %f\n',front_dist(1));
else
    if right_dist(1) < threshold
        w = P;
    elseif right_dist(1) > threshold
        w = -P;
    end
    % fprintf('right: %f\n',right_dist(1));
end

% set upper & lower limits to velocity
if v > 0.5
    v = 0.5;
elseif v < -0.5
    v = 0;
end

if w > 0.5
    w = 0.5;
elseif w < -0.5
    w = -0.5;
end
