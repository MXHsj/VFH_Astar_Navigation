%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 
% Version: 1.0
% Author: Xihan Ma
% Input: rgb image from kinect
% Output: corner flag
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [iscorner] = go2Corner(Img)

% global laserScan

% bw(:,:) = Img(:,:,1)>120 & Img(:,:,2)<90 & Img(:,:,3)<90; % red corner night
bw(:,:) = Img(:,:,1)>130 & Img(:,:,2)<60 & Img(:,:,3)<60; % red corner day

% scan_angle = linspace(-135,135,length(laserScan.Ranges));
% front_index = scan_angle >= -10 & scan_angle <= 10;   % front region
% front_dist = mean(laserScan.Ranges(front_index));  % front distance

% image filtering
bw = medfilt2(bw,[10 10]);      % mediam filtering
bw2 = bwareaopen(bw,200,8);    % delete isolate pixels
bw2 = imfill(bw2,'holes');

wall_area = sum(sum(bw2 == 1));
fprintf('wall_area: %f\n',wall_area);
if  wall_area > 8000 
    iscorner = true;
else
    iscorner = false;
end
% fprintf('corner flag: %d area: %d dist: %f\n',iscorner,wall_area,front_dist);

