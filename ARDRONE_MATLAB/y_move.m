function x_moveTo(obj, y_ref)
			Kp = 0.25;
			Ki = 0;
			Kd = 0.25;
			%pitch_setting;
			%pitch_setting;
			y_pos = 0;
			pitch_setting=0;
			pitch_max = 0.8;
			
			err_y=0;
			err_y1=0;


			while(err_y > 10 || err_y < -10)

				[drone_pos,ball_pos] = obj.kinect.TrackFrame();
				
                while(isempty(drone_pos) == 1)
                    disp('the drone_pos is empty');
                end
                
                y_pos = drone_pos(1);
				err_y = (y_ref - y_pos);
				pitch_setting = Kp*err_y + Ki*(err_y+err_y1) + Kd*(err_y-err_y1)
				%pitch_setting = Kp*err_y;
				pitch_setting = pitch_setting/1500;

				% Save the err[k-1]
				err_y1 = err_y;
				
				% Saturator
				if (pitch_setting > pitch_max)
					pitch_setting = pitch_max;
                elseif (pitch_setting < -pitch_max)
					pitch_setting = -pitch_max;
                end
                
                if(isnan(pitch_setting) == 1)
                   pitch_setting = 0;
                   disp('roll setting is nan');
                end
				
				% out of range
				if(y_pos > 3000 || y_pos < 300)
					pitch_setting = 0;
				end
				
				% Call the control function
				obj.y_control(pitch_setting)
				pause(0.05);
			end
			
			for i = 1:10
				[drone_pos,ball_pos] = obj.kinect.TrackFrame();
                
                 while(isempty(drone_pos) == 1)
                    disp('the drone_pos is empty');
                 end
                
                 
				y_pos = drone_pos(1);
				err_y = -(y_ref - y_pos);
				
				pitch_setting = Kp*err_y + Ki*(err_y+err_y1) + Kd*(err_y-err_y1)
				err_y1 = err_y;
                
				pitch_setting = pitch_setting/1500;
				
				
				% Saturator
				if (pitch_setting > pitch_max)
					pitch_setting = pitch_max;
                elseif (pitch_setting < -pitch_max)
					pitch_setting = -pitch_max;
				end
				
				% out of range
				if(y_pos > 3000 || y_pos < 300)
					pitch_setting = 0;
                end
                
                if(isnan(pitch_setting) == 1)
                   pitch_setting = 0;
                   disp('roll setting is nan');
                end
				
				obj.y_control(pitch_setting)
				pause(0.05);
			end
		end