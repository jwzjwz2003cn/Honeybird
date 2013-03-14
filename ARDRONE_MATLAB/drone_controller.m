classdef drone_controller < handle;
    
    properties
        droneobject
        kinect
        xmlPath
    end
    
    methods
        function obj = drone_controller()
            obj.droneobject = drone;
            obj.xmlPath = '/Users/James/Documents/GitHub/Honeybird/ARDRONE_MATLAB/SamplesConfig.xml';
			obj.kinect = SensorClass(obj.xmlPath);
        end
    end
end
