%SAMPLE_XML_PATH='Config/SamplesConfig.xml';
%SAMPLE_XML_PATH='E:\Program Files\kinect_matlab\Config\SamplesConfig.xml';
SAMPLE_XML_PATH='D:\Michael\Fourth Year Design Project\Kinect_Matlab_version1f\Config\SamplesConfig.xml';

% Start the Kinect Process
%filename='E:\Program Files\kinect_matlab\Example\ball.oni';
filename='D:\Michael\Fourth Year Design Project\Kinect_Matlab_version1f\Example\drone.oni';
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH,filename);

% To use the Kinect hardware use :
%KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

%load('E:\Program Files\MATLAB\R2012a\bin\nothing.mat');

hblobD = vision.BlobAnalysis( ...
    'AreaOutputPort', false, ...
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true, ...
    'OutputDataType', 'single', ...
    'MaximumCount', 1, ...
    'MinimumBlobArea', 3000, ...
    'MaximumBlobArea', intmax ...
    );
%Min was 5000

hblobB = vision.BlobAnalysis( ...
    'AreaOutputPort', false, ...
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true, ...
    'OutputDataType', 'single', ...
    'MaximumCount', 5, ...
    'MinimumBlobArea', 5, ...
    'MaximumBlobArea', 2000 ...
    );
%Min was 5
%Max blob was 2000

hshapeins = vision.ShapeInserter( ...
    'BorderColor', 'Custom', ...
    'CustomBorderColor', 65535);

htextinsX = vision.TextInserter( ...
    'Text', '%4d', ...
    'Location',  [1 1], ...
    'Color', [255 255 255], ...
    'FontSize', 16);

htextinsY = vision.TextInserter( ...
    'Text', '%4d', ...
    'Location',  [1 30], ...
    'Color', [255 255 255], ...
    'FontSize', 16);

htextinsZ = vision.TextInserter( ...
    'Text', '%4d', ...
    'Location',  [1 60], ...
    'Color', [255 255 255], ...
    'FontSize', 16);

D=mxNiDepth(KinectHandles);D=permute(D,[2 1]);
X=mxNiDepthRealWorld(KinectHandles);
subplot(1,1,1),h1=imshow(D); colormap();

i=1;
while(1) 
    
    %track drone%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mxNiUpdateContext(KinectHandles);
    
    if(  mod(i,1) == 0)
        tic
        %real world matrix
        X = mxNiDepthRealWorld(KinectHandles);
        %depth matrix
        D=mxNiDepth(KinectHandles);
        
        image = D;
        y = threshDepth(Dn,image);
        [centroidD, bboxD] = step(hblobD, y);
        
        if (isempty(centroidD))
            positionDrone = [NaN;NaN;NaN]
        else
            positionDrone = [X(int16(centroidD(2)),int16(centroidD(1)),1); ...
                X(int16(centroidD(2)),int16(centroidD(1)),2); ...
                X(int16(centroidD(2)),int16(centroidD(1)),3)];
            
            if (positionDrone == [0;0;0])
                positionDrone = positionPrevious;
            end
            D = step(hshapeins, D, bboxD);
            positionDrone   
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %track ball
        if (isempty(centroidD)==0)
            y(bboxD(2):bboxD(2)+bboxD(4)-1,bboxD(1):bboxD(1)+bboxD(3)-1)=0;
        end
        
        [centroidB,bboxB] = step(hblobB, y);
        
        if (isempty(centroidB))
            positionBall = [];
        else
            positionBall = [];
            for j=1:length(centroidB(:,1))
                positionBallj = [X(int16(centroidB(j,2)),int16(centroidB(j,1)),1); ...
                    X(int16(centroidB(j,2)),int16(centroidB(j,1)),2); ...
                    X(int16(centroidB(j,2)),int16(centroidB(j,1)),3)];
                positionBall = [positionBall positionBallj];
            end
            D = step(hshapeins, D, bboxB);
        end
     
        positionPrevious = positionDrone;
        
        D=permute(D,[2 1]);
        text = int16(positionDrone);
        D = step(htextinsX, D, text(1));
        D = step(htextinsY, D, text(2));
        D = step(htextinsZ, D, text(3));
        set(h1,'CDATA',D);
        drawnow;
        toc
        %waitforbuttonpress;
        
        i=0;
    end
    i=i+1;
end

% Stop the Kinect Process
mxNiDeleteContext(KinectHandles);

