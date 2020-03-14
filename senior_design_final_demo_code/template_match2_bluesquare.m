%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function name: template_match2_bluesquare
%
% description: shape recognition for senior design game 
%
% input: source image (rgb) to be matched, circle template image, rectangle
% template image
%
% output: circle matrix, square matrix
%
% author: Xihan Ma
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sqaure_matrix] = ...
    template_match2_bluesquare(Img,circle_template,rectangle_template)

bw(:,:) = Img(:,:,1)<40 & Img(:,:,2)<60 & Img(:,:,3)>70;   % blue night

% HSV
% bw(:,:) = abs((Img(:,:,1))-0.62)<0.2 & abs(Img(:,:,2)-0.85)<0.3 & abs(Img(:,:,3)-0.376)<0.4; 
    
% image filtering
% bw = medfilt2(bw,[20 20]);
bw2 = bwareaopen(bw,1000,8);    % delete isolate pixels
bw2 = imfill(bw2,'holes');

% split multiple objects
[~,L,N] = bwboundaries(bw2);

% recognize shape
if N>=1
    % for n = 1:N
        cut = L;
        % cut image
        sum_row(1:size(cut,1),1) = (cut == 1)*ones(size(cut,2),1);
        sum_col(1,1:size(cut,2)) = ones(1,size(cut,1))*(cut == 1);
        cut(sum_row==0,:) = [];
        cut(:,sum_col'==0) = [];
        
        % unify the size of two images if green color detected
        row = size(rectangle_template,1);
        col = size(rectangle_template,2);
        cut = imresize(cut,[row,col]);
        
        % compare similarity and give results
        var_circle = sum(abs(circle_template(:,:)-cut(:,:)));
        circle_likelihood = sum(var_circle);
        
        var_rectangle = sum(abs(rectangle_template(:,:)-cut(:,:)));
        rectangle_likelihood = sum(var_rectangle);
        
        if circle_likelihood < rectangle_likelihood
            % disp('cricle detected!!!');
            sqaure_matrix = NaN;
        else
            % disp('rectangle detected!!!');
            sqaure_matrix = L == 1;
        end
        
    % end
else
    % disp('no object detected!!!');
    sqaure_matrix = NaN;
end