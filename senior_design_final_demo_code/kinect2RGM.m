%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function name: kinect2RGB
%
% description: this function takes the input rgb image, converts it to
% binary by selecting green color from the image, applies multiple
% filtering strategies, and returns the filtered binary image
%
% input: image received from /kinect2/qhd/image_color_rect topic in ROS 
%
% output: uint8 rgb image 
%
% author: Xihan Ma
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Img = kinect2RGM(color)

color_b=color.Data(1:3:end);
color_g=color.Data(2:3:end);
color_r=color.Data(3:3:end);

color_b=reshape(color_b,[color.Width,color.Height])';
color_g=reshape(color_g,[color.Width,color.Height])';
color_r=reshape(color_r,[color.Width,color.Height])';

RGB(:,:,1)=color_r;
RGB(:,:,2)=color_g;
RGB(:,:,3)=color_b;

Img = uint8(RGB);