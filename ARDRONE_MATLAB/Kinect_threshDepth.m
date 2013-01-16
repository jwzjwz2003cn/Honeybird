%prototype method 1 with depth matrix and binary filter
function [bw] = threshDepth(noTarget,target)
filterDepth=min(noTarget(noTarget>0)); 
%for each frame Target
target(target>filterDepth)=0; 
level=graythresh(target);
bw=im2bw(target,level);
%filter for noise
bw = bwareaopen(bw, 50);
end