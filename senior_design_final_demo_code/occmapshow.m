function [] = occmapshow(occval)

index=occval==1;
occval(occval==0)=1;
occval(index)=0;
occval(occval==-1)=0.5;
imshow(occval)