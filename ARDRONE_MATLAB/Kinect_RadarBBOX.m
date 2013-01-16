classdef Kinect_RadarBBOX < handle
    properties (SetAccess = private)
        KinectHandles;
        hblobD;
        hblobB;
        Dn;
        positionDronePrev;
        hshapeins;
        h1;
        htextinsX;
        htextinsY;
        htextinsZ;
    end
    
    methods
        %constructor
        function KR = Kinect_RadarBBOX()
            KR.KinectHandles=mxNiCreateContext( ...
                'D:\Michael\Fourth Year Design Project\Kinect_Matlab_version1f\Config\SamplesConfig.xml');
                %'E:\Program Files\kinect_matlab\Config\SamplesConfig.xml');
            KR.hblobD = vision.BlobAnalysis( ...
                'AreaOutputPort', false, ...
                'CentroidOutputPort', true, ...
                'BoundingBoxOutputPort', true, ...
                'OutputDataType', 'single', ...
                'MaximumCount', 1, ...
                'MinimumBlobArea', 0, ...
                'MaximumBlobArea', intmax ...
                );
            KR.hblobB = vision.BlobAnalysis( ...
                'AreaOutputPort', false, ...
                'CentroidOutputPort', true, ...
                'BoundingBoxOutputPort', false, ...
                'OutputDataType', 'single', ...
                'MaximumCount', 1, ...
                'MinimumBlobArea', 0, ...
                'MaximumBlobArea', 0 ...
                );
            KR.hshapeins = vision.ShapeInserter( ...
                'BorderColor', 'Custom', ...
                'CustomBorderColor', 65535);
            
            KR.htextinsX = vision.TextInserter( ...
                'Text', '%4d', ...
                'Location',  [1 1], ...
                'Color', [255 255 255], ...
                'FontSize', 16);
            
            KR.htextinsY = vision.TextInserter( ...
                'Text', '%4d', ...
                'Location',  [1 30], ...
                'Color', [255 255 255], ...
                'FontSize', 16);
            
            KR.htextinsZ = vision.TextInserter( ...
                'Text', '%4d', ...
                'Location',  [1 60], ...
                'Color', [255 255 255], ...
                'FontSize', 16);
            KR.Dn=mxNiDepth(KR.KinectHandles);
            D=KR.Dn;
            D=permute(D,[2 1]);
            subplot(1,1,1),KR.h1=imshow(D,[0 9000]); colormap();
            
            
        end
        %calibrate background
        function Calibrate(KR)
            KR.Dn=mxNiDepth(KR.KinectHandles);
        end
        %obtain the drone and ball position
        function [positionDrone,positionBall] = TrackFrame(KR)
            mxNiUpdateContext(KR.KinectHandles);
            X = mxNiDepthRealWorld(KR.KinectHandles);
            D = mxNiDepth(KR.KinectHandles);
            y = ThreshDepth(KR.Dn,D);
            %track drone
            [centroidD, bboxD] = step(KR.hblobD, y);
            if (isempty(centroidD))
                positionDrone = [NaN;NaN;NaN];
            else
                positionDrone = [X(int16(centroidD(2)),int16(centroidD(1)),1); ...
                    X(int16(centroidD(2)),int16(centroidD(1)),2); ...
                    X(int16(centroidD(2)),int16(centroidD(1)),3)];
                if (positionDrone == [0;0;0])
                    positionDrone = KR.positionPrev;
                end
            end
            KR.positionDronePrev = positionDrone;
            D = step(KR.hshapeins, D, bboxD);
            D = permute(D,[2 1]);
            text = int16(positionDrone);
            D = step(KR.htextinsX, D, text(1));
            D = step(KR.htextinsY, D, text(2));
            D = step(KR.htextinsZ, D, text(3));
            set(KR.h1,'CDATA',D);
            drawnow;
            %track ball
            if (isempty(centroidD)==0)
                y(bboxD(2):bboxD(2)+bboxD(4)-1,bboxD(1):bboxD(1)+bboxD(3)-1)=0;
            end
            [centroidB] = step(KR.hblobB, y);
            if (isempty(centroidB))
                positionBall = [];
            else
                positionBall = zeros(3,length(centroidB(:,1)));
                for j=1:length(centroidB(:,1))
                    positionBallj = [X(int16(centroidB(j,2)),int16(centroidB(j,1)),1); ...
                        X(int16(centroidB(j,2)),int16(centroidB(j,1)),2); ...
                        X(int16(centroidB(j,2)),int16(centroidB(j,1)),3)];
                    positionBall(:,j) = positionBallj;
                end
            end
        end
        %Disconnect from Kinect (rarely used)
        function CloseConnection(KR)
            mxNiDeleteContext(KR.KinectHandles);
        end
    end
end
%thresholds depth image to binary image
function [bw] = ThreshDepth(noTarget,target)
filterDepth=min(noTarget(noTarget>0));
%for each frame Target
target(target>filterDepth)=0;
level=graythresh(target);
bw=im2bw(target,level);
%filter for noise
bw = bwareaopen(bw, 50);
end






