classdef drone_controller < handle;
    
    properties
        drone_object
        kinect
        xmlPath
        cord
        settings
    end
    
    methods
        function obj = drone_controller()
            obj.drone_object = drone;
            obj.xmlPath = 'C:\Users\Frank\Documents\GitHub\Honeybird\ARDRONE_MATLAB\SamplesConfig.xml';
			obj.kinect = SensorClass(obj.xmlPath);
            obj.kinect.Calibrate();
            obj.cord = [];
            obj.settings = [];
            
            
        end
        
        function record_cord(obj, x, y, z)
            
            [m,n] = size(obj.cord);
            obj.cord(m+1,:) = [x,y,z];
            
            
        end
        
        function record_setting(obj, x_setting, y_setting, z_setting)
            
            
            [m,n] = size(obj.settings);
            obj.settings(m+1,:) = [x_setting, y_setting, z_setting];
        end
        
        function obj = move_xyz(obj,x_ref,y_ref,z_ref)
%*******************Parameters for x control******************
            Kp_x = -0.6;
			Ki_x = 0;
			Kd_x = 0.7;
            x_pos = 0;
			roll_setting=0;
			roll_max = 0.5;
            err_x=0;
			err_x1=0;
    
%*******************Parameters for y control******************
            Kp_y = 0.3;
			Ki_y = 0;
			Kd_y = 0.4;
			y_pos = 0;
			pitch_setting=0;
			%yaw_setting = 0;
			pitch_max = 0.8;
			
			err_y=0;
			err_y1=0;

%*******************Parameters for z control******************            
            Kp_z = 0.8;
			Ki_z = 0.5;
			Kd_z = 0;

			z_setting=0;
			err_z=0;
			err_z1=0;
			
			yaw_setting = 0;
			
			z_max = 0.3;
            
%*********************Start of Controller ****************************           
            for i = 1:5

            %Capture Current Drone Data
            obj.drone_object.get_altitude;
            [drone_pos, ball_pos] = TrackFrame(obj.kinect);
            
            %Check for 0 in sensor data
             if (isempty(drone_pos) || (1 && all(drone_pos == 0)))    
                 disp('drone not found');
             else
                 %Get x y z coordinate
                 x_pos = drone_pos(1);
                 y_pos = drone_pos(3);
                 z_pos = obj.drone_object.altitude;           

                 %calculate error
                 err_x = (x_ref - x_pos)
                 err_y = y_ref - y_pos;
                 err_z = z_ref - z_pos;


                 % x controller
                 roll_setting = Kp_x*err_x + Ki_x*(err_x+err_x1) + Kd_x*(err_x-err_x1);
                 roll_setting = roll_setting/1500;

                 % y controller
                 pitch_setting = Kp_y*err_y + Ki_y*(err_y+err_y1) + Kd_y*(err_y-err_y1)
                 pitch_setting = pitch_setting/1500;

                 % z controller
                 z_setting = Kp_z*err_z + Ki_z*(err_z+err_z1) + Kd_z*(err_z-err_z1)
                 z_setting = z_setting / 2000   

                 % Save the err[k-1]
                 err_x1 = err_x;
                 err_y1 = err_y;
                 err_z1 = err_z;
                
                 

                 % x Saturator
                 if (roll_setting > roll_max)
                    roll_setting = roll_max;
                 elseif (roll_setting < -roll_max)
                    roll_setting = -roll_max;
                 end

                  % y Saturator
                  if (pitch_setting > pitch_max)
                      pitch_setting = pitch_max;
                  elseif (pitch_setting < -pitch_max)
                      pitch_setting = -pitch_max;
                  end
            %{      
                 % z saturator
                 if (z_setting > z_max)
					z_setting = z_max;
                 elseif (z_setting < -z_max)
					z_setting = -z_max;
                 end
%}
                 % x out of range
                 if(x_pos > 1200 || x_pos < -1200)
                    roll_setting = 0;
                 end

                 % y Out of Range
                 if(y_pos < 500 || y_pos > 3000)
                      pitch_setting = 0;
                 end
                 
                 if(z_pos < 300 || z_pos > 2000)
					z_setting = 0;
                    %break
                 end                 

                 if(isnan(pitch_setting) == 1)
                      pitch_setting = 0;
                 end

                 if(isnan(roll_setting) == 1)
                     roll_setting = 0;
                 end     
                 
                 %record the data
                 obj.record_cord(x_pos, y_pos, z_pos);
                 obj.record_setting(roll_setting, pitch_setting, z_setting);
             
             end
             
             

             obj.drone_object.xyzyaw(roll_setting,pitch_setting,z_setting,0)
            end
        end
             
             %************************ controller *******************
		
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
                
                c
            end
        end	
		
    end
end