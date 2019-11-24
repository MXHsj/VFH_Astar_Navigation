function [globalpose]=astar2global2(astarpose)
% Version 0.3

global map

resolution=map.Resolution;
x_length=map.GridSize(1)/resolution;
y_length=map.GridSize(2)/resolution;


globalx(:,1)=astarpose(:,1)./resolution-x_length/2;
globaly(:,1)=astarpose(:,2)./resolution-y_length/2;

globalpose=[globalx,globaly];