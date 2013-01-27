%SAMPLE_XML_PATH='Config/SamplesConfig.xml';
SAMPLE_XML_PATH='SamplesConfig.xml';

% To use the Kinect hardware use :
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

figure;

display('recording');
% Capture 20 Frames
for i=1:200
    tic
    display(i);
    mxNiUpdateContext(KinectHandles);
    D=mxNiDepth(KinectHandles);
    D=permute(D,[2 1]);
    imshow(D,[0 9000]);
    drawnow;
    toc
end

display('finish recording');

% Stop the Kinect Process
mxNiDeleteContext(KinectHandles);

 