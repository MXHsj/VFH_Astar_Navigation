%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: the function takes the target direction and gives angular
% velocity command through VFH +
% Version: 1.0
% author: Xihan Ma
% Input: target direction in radius
% Output: linear velocity in meterper sec & angular velocity inradius 
% per second
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [v,w] = VFH_nav(targetDir, laserScan)

global vfh
ranges = double(laserScan.Ranges);
angles = double(laserScan.readScanAngles);
steerDir= vfh(ranges, angles, targetDir);
max_w = 0.65;
% show(vfh)

% Calculate velocities
if ~isnan(steerDir) % If steering direction is valid
    v = 0.4;
    w = exampleHelperComputeAngularVelocity(steerDir, max_w);
else 
    v = 0.0;
    w = max_w;
end
