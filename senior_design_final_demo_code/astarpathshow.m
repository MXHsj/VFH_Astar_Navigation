function [] = astarpathshow(OptimalPath)
global map

occval = occupancyMatrix(map,"ternary");

occval=fliplr(occval');
index=occval==0;
occval(occval==1)=0;
occval(index)=1;

[row,col] = find(occval == 0);
figure
plot(row,col,'.k')
hold on;axis equal;
plot(OptimalPath(end,1),OptimalPath(end,2),'xb')
plot(OptimalPath(:,1),OptimalPath(:,2),'-r')
