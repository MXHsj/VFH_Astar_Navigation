function [OptimalPath]=astar3(start,goal)
% Version: 0.4

global map

resolution=map.Resolution;
x_length=map.GridSize(1)/resolution;
y_length=map.GridSize(2)/resolution;
occval = occupancyMatrix(map,"ternary");

occval=fliplr(occval');
% index=occval==0;
% occval(occval==1)=0;
% occval(index)=1;


[Start]=global2astar2(start);
[Goal]=global2astar2(goal);
if Start==Goal
    OptimalPath=Start;
    
else
    
    StartX=Start(1);
    StartY=Start(2);
    GoalX=Goal(1);
    GoalY=Goal(2);
    
    
    GoalRegister=int8(zeros(x_length*resolution,y_length*resolution));
    GoalRegister(GoalX,GoalY)=1;
    
    
    Connecting_Distance=16;
    
    OptimalPath=ASTARPATH(StartY,StartX,occval,GoalRegister,Connecting_Distance);
    
    OptimalPath=flipud(OptimalPath);
end

