xmlPath = 'C:\Users\Frank\Documents\GitHub\Honeybird\ARDRONE_MATLAB\SamplesConfig.xml';
kinect = SensorClass(xmlPath);
kinect.Calibrate();

track(kinect);