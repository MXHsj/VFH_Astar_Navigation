function [globalpose]=astar2global(astarpose)
% Version 0.2

global map pose0

resolution=map.Resolution;
x_length=map.GridSize(1)/resolution;
y_length=map.GridSize(2)/resolution;


globalx(:,1)=astarpose(:,1)./resolution-x_length/2+pose0(1);
globaly(:,1)=astarpose(:,2)./resolution-y_length/2+pose0(2);

% globalx(:,1)=astarpose(:,1)./resolution-x_length/2;
% globaly(:,1)=astarpose(:,2)./resolution-y_length/2;

globalpose=[globalx,globaly];