%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 
% Version: 1.0
% Author: Xihan Ma
% Input: rgb image from kinect
% Output: corner flag
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [v,w,dist2goal] = trackCorner(Img)

global laserScan

% bw(:,:) = Img(:,:,1)>120 & Img(:,:,2)<90 & Img(:,:,3)<90; % red corner night
bw(:,:) = Img(:,:,1)>130 & Img(:,:,2)<60 & Img(:,:,3)<60; % red corner day

scan_angle = linspace(-135,135,length(laserScan.Ranges));
front_index = scan_angle >= -15 & scan_angle <= 15;   % front region
dist2goal = min(laserScan.Ranges(front_index));  % front distance

% image filtering
% bw = medfilt2(bw,[10 10]);      % mediam filtering
bw2 = bwareaopen(bw,600,8);    % delete isolate pixels
bw2 = imfill(bw2,'holes');

kwp = 0.003;

img_center_col = size(Img,2)/2 - 100;
[~,c] = find(bw2 == 1);
c_center = mean(c);
error = img_center_col - c_center;

w = kwp*error;

if w > 0.4
    w = 0.4;
elseif w < -0.4
    w = -0.4;
end

fprintf('dist2goal: %f, w: %f\n',dist2goal,w);
v = 0.2;




