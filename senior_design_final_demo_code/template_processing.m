%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function name: template_processing
%
% description: this function takes the input rgb image, converts it to
% binary by selecting green color from the image, applies multiple
% filtering strategies, and returns the filtered binary image
%
% input: source template image (rgb)
%
% output: filtered binary image
%
% author: Xihan Ma
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [bwtemplate] = template_processing(Img)
% %% convert Img to binary by sorting green color
% bw(:,:) = Img(:,:,1)<90 & Img(:,:,2)>100 & Img(:,:,3)<90;

%% convert Img to binary by sorting red color
bw(:,:) = Img(:,:,1)>100 & Img(:,:,2)<90 & Img(:,:,3)<90;

%% apply discrete-cosine-transform to get rid of noise
% bw_dct = dct2(bw);
% m = size(Img,1);
% n = size(Img,2);
% I=zeros(m,n);
% I(1:m/7,1:n/7)=1;   % high frequency filtering
% bw_mod=bw_dct.*I;
% bw2=idct2(bw_mod);

% bw2 = imbinarize(bw2);      % convert to binary image
bw2 = bwareaopen(bw,400,8);    % delete isolate pixels
bw2 = medfilt2(bw2,[15 15]);    % mediem filter
bw2 = bwfill(bw2,'holes');

%% cut the image
% contour = edge(bw2,'sobel');     % extract edge using sobel
% contour = edge(bw,'canny');      % extract edge using canny
cut = bw2;
sum_row(1:size(cut,1),1) = (cut == 1)*ones(size(cut,2),1);
sum_col(1,1:size(cut,2)) = ones(1,size(cut,1))*(cut == 1);
cut(sum_row==0,:) = [];
cut(:,sum_col'==0) = [];


%% output
bwtemplate = imresize(cut,[899,899]);