function [astarpose]=global2astar2(globalpose)
%Version: 0.2

global map pose0

resolution=map.Resolution;
x_length=map.GridSize(1)/resolution;
y_length=map.GridSize(2)/resolution;


astarx(:,1)=round((globalpose(:,1)- pose0(1)+x_length/2).*resolution);
% astary(:,1)=round((globalpose(:,2)+ pose0(2)+y_length/2).*resolution);
astary(:,1)=round((globalpose(:,2)- pose0(2)+y_length/2).*resolution);

% astarx(:,1)=round((globalpose(:,1)+x_length/2).*resolution);
% astary(:,1)=round((globalpose(:,2)+y_length/2).*resolution);

astarpose=[astarx,astary];