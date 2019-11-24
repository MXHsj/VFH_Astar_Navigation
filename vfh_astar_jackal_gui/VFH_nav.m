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

% Calculate velocities
if ~isnan(steerDir) % If steering direction is valid
    v = 0.35;
    w = exampleHelperComputeAngularVelocity(steerDir, 0.65);
else % Stop and search for valid direction
    v = 0.0;
    w = 0.5;
end
