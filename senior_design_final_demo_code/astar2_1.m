function [OptimalPath]=astar2_1(start,goal,map_temp)
% Version: 0.3

% global map

resolution=map_temp.Resolution;
x_length=map_temp.GridSize(1)/resolution;
y_length=map_temp.GridSize(2)/resolution;
occmat = occupancyMatrix(map_temp,"ternary");

occmat=fliplr(occmat');
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
    
    OptimalPath=ASTARPATH(StartY,StartX,occmat,GoalRegister,Connecting_Distance);
    
    OptimalPath=flipud(OptimalPath);
end