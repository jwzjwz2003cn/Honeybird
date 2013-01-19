classdef drone < handle

    properties
        ARcontrol
        ARnav
        ARc = udp('192.168.1.1', 5556, 'LocalPort', 5556);
        ARn = udp('192.168.1.1', 5554, 'LocalPort', 5554);
        state
        battery
        pitch
        roll        
        yaw
        altitude  % millimeters
        vx
        vy
        vz
		kinect
        
    end
    
    methods
        
        function obj = drone()
            
            obj.battery = 0;
            obj.ARcontrol = control(obj.ARc);
            obj.ARnav = navdata(obj.ARn, obj.ARcontrol);
            obj.state = 0 ;          
            obj.pitch = 0;
            obj.roll = 0;
            obj.yaw = 0;
            obj.altitude = 0;
            obj.vx = 0;
            obj.vy = 0;
            obj.vz = 0;
			%obj.kinect = Kinect_RadarBBOX;
            
        end
		
		function x_moveTo(obj, x_ref_array)
			Kp = 0.6;
			Ki = 0;
			Kd = 0.25;
			x_pos = 0;
			n = numel(x_ref_array); %The number of reference points
			roll_setting=0;
			roll_max = 0.8;
			yaw_setting = 0;
			
			err_x=0;
			err_x1=0;	
            
            obj.get_yaw % degree
            yaw_offset = obj.yaw; % will be equal to the drone's yaw at stationary facing opposite to the camera
			
			for i = 1: n
				%get drone's position
				[drone_pos,ball_pos] = obj.kinect.TrackFrame();
				x_pos = drone_pos(1);
				
				%controller
                x_ref = x_ref_array(i);
				err_x = -(x_ref - x_pos);
				
				% Save the err[k-1]
				err_x1 = err_x;
				
				
				roll_setting = Kp*err_x + Ki*(err_x+err_x1) + Kd*(err_x-err_x1);
				%roll_setting = Kp*err_x;
                roll_setting = roll_setting/1500;

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
				
                %calculate the correct yaw setting
				yaw_setting = obj.yaw_control(0, yaw_offset);
                
            if(isnan(roll_setting) == 1)
                roll_setting = 0;
            end
				% Call the control function
				obj.x_control(roll_setting,yaw_setting)
				pause(0.05);
			end
		end
		
		function y_moveTo(obj, y_ref_array)
			Kp = 0.5;
			Ki = 0;
			Kd = 0.25;
			%roll_setting;
			%pitch_setting;
			y_pos = 0;
			
			n = numel(y_ref_array); %The number of reference points
			
			pitch_setting=0;
			yaw_setting = 0;
			pitch_max = 0.8;
			
			err_y=0;
			err_y1=0;


			for i = 1:n
				%get the drone y position
				[drone_pos,ball_pos] = obj.kinect.TrackFrame();
				y_pos = drone_pos(3);
				
				% calculate the error
                y_ref = y_ref_array(i);
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
				if(y_pos < 300 || y_pos > 3000)
					pitch_setting = 0;
				end
				
				yaw_setting = obj.yaw_control(0);
				
                if(isnan(pitch_setting) == 1)
                    pitch_setting = 0;
                end
				% Call the control function
				obj.y_control(pitch_setting,yaw_setting)
				pause(0.05);
			end
			
		end
		
		% z control
		function z_moveTo(obj, z_ref)
			Kp = 0.8;
			Ki = 0.5;
			Kd = 0.25;

			z_setting=0;
			err_z=0;
			err_z1=0;
			
			yaw_setting = 0;
			
			z_max = 0.8;
			
			n = numel(z_ref);

			for i = 1:n
				obj.get_altitude;
				err_z = z_ref(i) - obj.altitude;
				
				z_setting = Kp*err_z + Ki*(err_z+err_z1) + Kd*(err_z-err_z1)
				
				err_z1 = err_z;
				z_setting = z_setting / 1500;
				
				if (z_setting > z_max)
					z_setting = z_max;
                elseif (z_setting < -z_max)
					z_setting = -z_max;
				end
				
				% Out of Range
				if(z_pos < 300 || z_pos > 2000)
					z_setting = 0;
				end
				
				yaw_setting = obj.yaw_control(0);
				obj.z_control(z_setting,yaw_setting)
				pause(0.05);
			end
        end
        
		function yaw_setting = yaw_control(obj, yaw_ref,yaw_offset)
			
			Kp = 0.5;
			Ki = 0;
			Kd = 0.25;
			
			err_yaw = 0;
			err_yaw1 = 0;
			current_yaw = 0;

			yaw_max = 0.8;
			
           
            %calculate the real yaw of the drone
            obj.get_yaw % degree
            current_yaw = obj.yaw - yaw_offset
               
            err_yaw = yaw_ref - current_yaw;
            err_yaw1 = err_yaw;
            yaw_setting = Kp*err_yaw + Ki*(err_yaw+err_yaw1) + Kd*(err_yaw-err_yaw1);

            yaw_setting = yaw_setting/100

            if(yaw_setting > yaw_max)
                yaw_setting = yaw_max;
            elseif(yaw_setting < -yaw_max)
                yaw_setting = -yaw_max;
            end
            
            if(isnan(yaw_setting) == 1)
                yaw_setting = 0;
            end
   
		end
        
		function yaw_setting = yaw_control_test(obj, yaw_ref_array)
			obj.get_yaw % degree
            yaw_offset = obj.yaw; % will be equal to the drone's yaw at stationary facing opposite to the camera
			Kp = 0.5;
			Ki = 0;
			Kd = 0.25;
			
			err_yaw = 0;
			err_yaw1 = 0;
			current_yaw = 0;
			n = numel(yaw_ref_array);
			yaw_max = 0.8;
			
            for i = 1:n
                %calculate the real yaw of the drone
                obj.get_yaw % degree
                current_yaw = obj.yaw - yaw_offset
                
                yaw_ref = yaw_ref_array(i);
                err_yaw = yaw_ref - current_yaw;
                err_yaw1 = err_yaw;
                yaw_setting = Kp*err_yaw + Ki*(err_yaw+err_yaw1) + Kd*(err_yaw-err_yaw1);

                yaw_setting = yaw_setting/100

                if(yaw_setting > yaw_max)
                    yaw_setting = yaw_max;
                elseif(yaw_setting < -yaw_max)
                    yaw_setting = -yaw_max;
                end

                obj.yaw_set(yaw_setting);
                
                pause(0.05);
            end 
		end
		
		function yaw_set(obj, speed)
            obj.reset_wdg();
            speed_dec = (obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,0,0,speed_dec);
        end
				
		function z_control(obj, gaz,yaw)
            obj.reset_wdg();
            gaz_dec = (obj.float2dec(gaz));
            yaw_dec = (obj.float2dec(yaw));
            prog_cmd = 3;
            obj.ARcontrol.at_pcmd(prog_cmd,0,0,gaz_dec,yaw_dec);
        end	
		
		function y_control(obj, pitch,yaw)
            obj.reset_wdg();
            pitch_dec = (obj.float2dec(pitch));
            yaw_dec = (obj.float2dec(yaw));
            prog_cmd = 3;
            obj.ARcontrol.at_pcmd(prog_cmd,0,pitch_dec,0,yaw_dec); 
        end
		
        function x_control(obj, roll,yaw)
            obj.reset_wdg();
            roll_dec = (obj.float2dec(roll));
            yaw_dec = (obj.float2dec(yaw));
            prog_cmd = 3;
            obj.ARcontrol.at_pcmd(prog_cmd,roll_dec,0,0,yaw_dec);             
        end
        
        function dec_num = float2dec(obj, float_num)
            single_float = single(float_num);
            signed_hex = num2hex(single_float);
            dec_num = typecast(uint32(hex2dec(signed_hex)),'int32');                        
        end
        
        function terminate(obj)
            obj.ARcontrol.terminate();
            obj.ARnav.terminate();
        end
        
        function takeoff(obj)
            obj.reset_wdg();
            ref_arg = 290718208;
            obj.ftrim();
            obj.ARcontrol.at_ref(ref_arg);
            
        end
        
        function land(obj)
            obj.reset_wdg();
            ref_arg = 290717696;
            obj.ARcontrol.at_ref(ref_arg);          
        end
        
                
        function reset(obj)
            obj.reset_wdg();
            ref_arg = 290717952;
            obj.ARcontrol.at_ref(ref_arg); 
        end
        
        function move_forward(obj, speed)
            obj.reset_wdg();
            speed_dec = -(obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,speed_dec,0,0);       
        end
        
        function move_backward(obj, speed)
            obj.reset_wdg();
            speed_dec = (obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,speed_dec,0,0); 
        end
        
        function roll_left(obj, speed)
            obj.reset_wdg();
            speed_dec = -(obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,speed_dec,0,0,0);             
        end
        
        function roll_right(obj, speed)
            obj.reset_wdg();
            speed_dec = (obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,speed_dec,0,0,0);   
        end
            
        function go_up(obj, speed)
            obj.reset_wdg();
            speed_dec = (obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,0,speed_dec,0);
        end
        
        function go_down(obj, speed)
            obj.reset_wdg();
            speed_dec = -(obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,0,speed_dec,0);
        end

        
        function rotate_left(obj, speed)
            obj.reset_wdg();
            speed_dec = -(obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,0,0,speed_dec);
        end
        
        function rotate_right(obj, speed)
            obj.reset_wdg();
            speed_dec = (obj.float2dec(speed));
            prog_cmd = 1;
            obj.ARcontrol.at_pcmd(prog_cmd,0,0,0,speed_dec);
        end

        
        function ftrim(obj)
            obj.reset_wdg();
            obj.ARcontrol.at_ftrim();
        end

        function reset_ports(obj)
            tic;
            obj.ARcontrol.reset_port;
            obj.ARnav.reset_port;
            toc;
           % disp('reseting ports');
        end
        
        function reset_wdg(obj)
            obj.ARcontrol.at_comwdg();
        end
        
        function get_battery(obj)
            obj.reset_ports;
            obj.battery = obj.ARnav.decode_battery;
        end
        
        function get_pitch(obj)
            obj.reset_ports;
            obj.pitch = obj.ARnav.decode_pitch;
        end
        
        function get_roll(obj)
            obj.reset_ports;
            obj.roll = obj.ARnav.decode_roll;
        end
        
        function get_yaw(obj)
            obj.reset_ports;
            obj.yaw = obj.ARnav.decode_yaw;
        end
        
        function get_altitude(obj, hObject, evt)
            obj.reset_ports;
            alt = obj.ARnav.decode_altitude;
            obj.altitude = alt;
        end
        
        function get_vx(obj)
            obj.reset_ports;
            obj.vx = obj.ARnav.decode_vx;
        end
        
        function get_vy(obj)
            obj.reset_ports;
            obj.vy = obj.ARnav.decode_vy;
        end
        
        function get_vz(obj)
            obj.reset_ports;
            obj.vz = obj.ARnav.decode_vz; 
        end
        
        function track_roundel(obj)
            obj.reset_wdg();
            obj.ARcontrol.at_flymode();
        end
    end
    
end

