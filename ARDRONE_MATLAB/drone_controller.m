classdef drone_controller < handle;
    
    properties
        drone_object
        kinect
        xmlPath
    end
    
    methods
        function obj = drone_controller()
            obj.drone_object = drone;
            obj.xmlPath = 'C:\Users\Frank\Documents\GitHub\Honeybird\ARDRONE_MATLAB\SamplesConfig.xml';
			obj.kinect = SensorClass(obj.xmlPath);
            obj.kinect.Calibrate();
        end
		
		function move_x(obj, x_ref)
			Kp = -0.6;
			Ki = 0;
			Kd = 0.7;
			x_pos = 0;
			roll_setting=0;
			roll_max = 0.5;
			yaw_setting = 0;
			
			err_x=0;
			err_x1=0;	
 
			for i = 1: 10
				%get drone's position
				[drone_pos, ball_pos] = TrackFrame(obj.kinect);
                
                if (isempty(drone_pos) || (1 && all(drone_pos == 0)))
                    
                     disp('drone not found');

                else
                    
                    x_pos = drone_pos(1)
				
                    %controller
                    err_x = (x_ref - x_pos)

                    roll_setting = Kp*err_x + Ki*(err_x+err_x1) + Kd*(err_x-err_x1);
                    %roll_setting = Kp*err_x;
                    roll_setting = roll_setting/1500;

                
                     % Save the err[k-1]
                    err_x1 = err_x;
                    % Saturator
                    if (roll_setting > roll_max)
                        roll_setting = roll_max;
                    elseif (roll_setting < -roll_max)
                        roll_setting = -roll_max;
                    end

                    % out of range
                    if(x_pos > 1200 || x_pos < -1200)
                        roll_setting = 0;
                    end
                
                    if(isnan(roll_setting) == 1)
                        roll_setting = 0;
                    end
                    % Call the control function
                    obj.drone_object.xyzyaw(roll_setting,0,0,0)
                    
                    
                end

			end
		end
		
		function move_y(obj, y_ref)
			Kp = 0.3;
			Ki = 0;
			Kd = 0.4;
			y_pos = 0;
			
			pitch_setting=0;
			%yaw_setting = 0;
			pitch_max = 0.8;
			
			err_y=0;
			err_y1=0;

            
			for i = 1:5
				%get the drone y position
				[drone_pos, ball_pos] = TrackFrame(obj.kinect);
                
                if (isempty(drone_pos) || (1 && all(drone_pos == 0)))
                    
                   disp('drone not found');
                else
                   y_pos = drone_pos(3); % Is this correct?
				
                   % calculate the error
                   err_y = y_ref - y_pos;
				
                   % controller
                    pitch_setting = Kp*err_y + Ki*(err_y+err_y1) + Kd*(err_y-err_y1)
                    pitch_setting = pitch_setting/1000;

                    % Save the err[k-1]
                    err_y1 = err_y;
				
                    % Saturator
                    if (pitch_setting > pitch_max)
                        pitch_setting = pitch_max;
                    elseif (pitch_setting < -pitch_max)
                        pitch_setting = -pitch_max;
                    end
				
                    % Out of Range
                    if(y_pos < 500 || y_pos > 3000)
                        pitch_setting = 0;
                    end
				
                    if(isnan(pitch_setting) == 1)
                       pitch_setting = 0;
                    end
                    % Call the control function
                    obj.drone_object.xyzyaw(0,pitch_setting,0,0)
                
                  end
            

			end
			
		end
		
		 function move_z(obj,z_ref)
            Kp = 0.8;
			Ki = 0.5;
			Kd = 0;

			z_setting=0;
			err_z=0;
			err_z1=0;
			
			yaw_setting = 0;
			
			z_max = 0.3;
            
            for i = 1:5
                obj.drone_object.get_altitude;
                z_pos = obj.drone_object.altitude;
                err_z = z_ref - z_pos;
                
                z_setting = Kp*err_z + Ki*(err_z+err_z1) + Kd*(err_z-err_z1)
				z_setting = z_setting / 1000
                
                err_z1 = err_z;
                
                if (z_setting > z_max)
					z_setting = z_max;
                elseif (z_setting < -z_max)
					z_setting = -z_max;
                end
                
                if(z_pos < 300 || z_pos > 2000)
					z_setting = 0;
                    break
                end
                
                obj.drone_object.xyzyaw(0,0,z_setting,0)
            end
        end	
		
    end
end