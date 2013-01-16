function [Dn] = calibrate()
%SAMPLE_XML_PATH='E:\Program Files\kinect_matlab\Config\SamplesConfig.xml';
SAMPLE_XML_PATH='D:\Michael\Fourth Year Design Project\Kinect_Matlab_version1f\Config\SamplesConfig.xml';
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);
Dn=mxNiDepth(KinectHandles);
end