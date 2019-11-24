function [OptimalPath]=astar(start,goal)
% Version: 0.2

global map resolution

x_length=map.GridSize(1)/resolution;
y_length=map.GridSize(2)/resolution;
occval = occupancyMatrix(map,"ternary");

occval=fliplr(occval');
% index=occval==0;
% occval(occval==1)=0;
% occval(index)=1;


[Start]=global2astar(start);
[Goal]=global2astar(goal);
StartX=Start(1);
StartY=Start(2);
GoalX=Goal(1);
GoalY=Goal(2);


% x=start(1)- pose0(1);
% y=start(2)+ pose0(2);
% 
% 
% StartX=round((x+x_length/2)*resolution);
% StartY=round((y+y_length/2)*resolution);
% 
% 
% GoalX=round((goal(1)- pose0(1)+x_length/2)*resolution);
% GoalY=round((goal(2)+ pose0(2)+y_length/2)*resolution);

GoalRegister=int8(zeros(x_length*resolution,y_length*resolution));
GoalRegister(GoalX,GoalY)=1;


Connecting_Distance=16;

OptimalPath=ASTARPATH(StartY,StartX,occval,GoalRegister,Connecting_Distance);

OptimalPath=flipud(OptimalPath);


