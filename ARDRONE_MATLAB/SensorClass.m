classdef SensorClass < handle
    properties (SetAccess = private)
        SensorHandles;
        hblob;
        Dn;
    end
    
    methods
        %constructor
        function KR = SensorClass(s)
            %   s is a path directory to a config xml file and can be something like 
            %   'D:\Michael\Fourth Year Design Project\Kinect_Matlab_version1f\Config\SamplesConfig.xml'
            %   the apostrophes should be included in s
            KR.SensorHandles=mxNiCreateContext(s);
            
            %   edit this to calibrate the performance of the blob
            %   analysis. you can turn boundingbox off here
            KR.hblob = vision.BlobAnalysis( ...
                'AreaOutputPort', false, ...
                'CentroidOutputPort', true, ...
                'BoundingBoxOutputPort', true, ...
                'OutputDataType', 'single', ...
                'MaximumCount', 2, ...
                'MinimumBlobArea', 0, ...
                'MaximumBlobArea', intmax ...
                );		
            KR.Dn=mxNiDepth(KR.SensorHandles);
        end
        %calibrate background
        function Calibrate(KR)
            KR.Dn=mxNiDepth(KR.SensorHandles);
        end
        %obtain the drone and ball position
        function [positionD, positionB] = TrackFrame(KR)
            
            mxNiUpdateContext(KR.SensorHandles);
            
            %   input depth image
            Real = mxNiDepthRealWorld(KR.SensorHandles);
            Depth = mxNiDepth(KR.SensorHandles);
            
            %   obtain binary image
            bImage = ThreshDepth(KR.Dn,Depth);
            
            %tracking
            [centroid, bbox] = step(KR.hblob, bImage);
            
            %if nothing is detected, return null matrix
            if (isempty(centroid))
                positionD = [];
                positionB = [];
            else
                positionDB = zeros(length(centroid(:,1)), 4);
                %iterate through the blobs to create a position matrix
                for j = 1:length(centroid(:,1))
                    %position matrix where each each row represent a blob.
                    %Column 1 is the Area of the blob
                    %Column 2 is the x coordinate
                    %Column 3 is the y coordinate
                    %Column 4 is the z coordinate
                    positionj = [ bbox(j,3)*bbox(j,4)/((Real(int16(centroid(j,2)),int16(centroid(j,1)),3))^-2) ...
                        Real(int16(centroid(j,2)),int16(centroid(j,1)),1) ...
                        Real(int16(centroid(j,2)),int16(centroid(j,1)),2) ...
                        Real(int16(centroid(j,2)),int16(centroid(j,1)),3)];
                    positionDB(j,:) = positionj;
                end
                %sort the position matrix based on the area
                positionDB = sort(positionDB, 1, 'descend');
                %position of the drone is the 1st row of the sorted matrix
                positionD = [positionDB(1,2); ...
                    positionDB(1,3); ...
                    positionDB(1,4)];
                %position of the balls are the remaining rows of the
                %position matrix
                if (length(positionDB(:,1)) == 1)
                    positionB = [];
                else
                    positionB = zeros(3,length(centroid(:,1))-1);
                    
                    for j = 2:length(centroid(:,1))
                        positionB(:,j-1) = [positionDB(j,2); ...
                        positionDB(j,3); ...
                        positionDB(j,4)];
                    end
                end
            end
        end
        %Disconnect from Kinect (rarely used)
        function CloseConnection(KR)
            mxNiDeleteContext(KR.SensorHandles);
        end
    end
end
%thresholds depth image to binary image
function [BW] = ThreshDepth(noTarget,target)
filterDepth=min(noTarget(noTarget>0));
%for each frame Target
target(target>filterDepth)=0;
level=graythresh(target);
bw=im2bw(target,level);
%filter for noise
bw=bwareaopen(bw,50);
SE = strel('arbitrary', 10);
BW = imclose(bw,SE);
end