function [ax,ay,tf] = replan(occval,x,y,Goal,t)
global ax1 pathplan start_pnt obstacles

occval=fliplr(occval');
index=occval==0;
occval(occval==1)=0;
occval(index)=1;
[row,col] = find(occval == 0);

OptimalPath = astar2([x,y],Goal(1:2));
[globalx, globaly] = astar2global2(OptimalPath);

delete(pathplan)
delete(obstacles)
obstacles = plot(ax1,row,col,'.k');
pathplan = plot(ax1,OptimalPath(:,1),OptimalPath(:,2));
start_pnt = plot(ax1,OptimalPath(1,1),OptimalPath(1,2),'xm');

range_goal = sqrt((Goal(1)-x)*(Goal(1)-x)+(Goal(2)-y)*(Goal(2)-y));
tf = t + range_goal/0.3;
num_samp = length(OptimalPath);
time_samp = linspace(t,tf,num_samp);
% time_samp = t:tf/(num_samp-1):tf;

order = 5;
A = zeros(2*num_samp,order+1);
A1 = zeros(num_samp,order+1);
A2 = zeros(num_samp,order+1);
for i = 1:num_samp
    for j = 1:order+1
        A1(i,j) = time_samp(i)^(j-1);
    end
end
for i = 1:num_samp
    for j = 2:order+1
        A2(i,j) = (j-1)*time_samp(i)^(j-2);
    end
end
A(1:2:2*num_samp-1,:) = A1;
A(2:2:2*num_samp,:) = A2;

vd_x = 0.4;
vd_y = 0.4;

bx = vd_x*ones(2*num_samp,order+1);
bx(1:2:2*num_samp-1) = globalx.*ones(num_samp,1);
bx(2) = 0; bx(end) = 0;

by = vd_y*ones(2*num_samp,order+1);
by(1:2:2*num_samp-1) = globaly.*ones(num_samp,1);
by(2) = 0; by(end) = 0;

ax = A\bx;
ay = A\by;
