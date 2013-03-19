classdef ffield < handle;

    properties
        ardrone  
        ardrone_dummy
        goal
        ball
        ball_radius
        count
        time 
        droneObject
        kinect
        droneMoved
        xmlPath
        counter
        b_counter
        drone_path
        new_path
        ball_path
        eva_path
        cord
        settings
    end
    
    methods
        
        function obj = ffield()
            obj.time = 1;
            obj.ardrone = [0,0,10,0,0,0];
            obj.ardrone_dummy = [0,0,0,0,0,0];
            obj.goal = [70, 12, 10, 0, 0, 0];
            obj.ball = [];
     
            obj.ball_radius = 0;
            
            obj.droneObject = drone;
            obj.droneMoved = false;
            obj.xmlPath = 'C:\Users\Frank\Documents\GitHub\Honeybird\ARDRONE_MATLAB\SamplesConfig.xml';
            obj.kinect = SensorClassBBOX(obj.xmlPath);
            obj.calibrate_kinect;
            obj.counter = 0;
            obj.b_counter = 0;
            
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
%{        
        function drone_cp =  map_drone (obj, drone_pos)%map out the drone's head, left and right control points
            
            h = 1.4; % drone's dimension constant
            l = 0.7; % drone's dimension constant
            
            x = drone_pos(1);
            y = drone_pos(2);
            z = drone_pos(3);
            yaw = drone_pos(4); 
            pitch = drone_pos(5);
            roll = drone_pos(6);
            
            drone_cp = zeros(3,3);
            
            % drone head kinematics
            drone_cp(1,1) = x+h*cos(yaw)*cos(pitch);
            drone_cp(1,2) = y+h*sin(yaw)*cos(pitch);
            drone_cp(1,3) = z-h*sin(pitch);
            
            % drone left kinematics
            drone_cp(2,1) = x + l*cos(yaw)*sin(pitch)*sin(roll)-l*sin(yaw)*sin(roll);
            drone_cp(2,2) = y + l*sin(yaw)*sin(pitch)*sin(roll)+l*cos(yaw)*cos(roll);
            drone_cp(2,3) = z + l*cos(pitch)*sin(roll);
            
            %drone right kinematics
            drone_cp(3,1) = x - l*cos(yaw)*sin(pitch)*sin(roll)+l*sin(yaw)*cos(roll);
            drone_cp(3,2) = y - l*sin(yaw)*sin(pitch)*sin(roll)-l*cos(yaw)*cos(roll);
            drone_cp(3,3) = z - l*cos(pitch)*sin(roll);
            
            
            
        end
        
        function fa = force_attr(obj, ardrone, goal)
            
            fa = zeros(1,3);                      
            att_const = 0.1;
            unity_dist = 1;
            
            d = obj.find_distance(ardrone, goal);
            if (d < unity_dist)
                fa = att_const * (goal - ardrone);
            else
                fa = unity_dist * att_const * (goal-ardrone)/d; %increase the field when close
            end
            
        end
    %}    
        function fp = force_rep(obj, ardrone, ball, ball_radius)
            fp = zeros(1,3);
            rep_const = 200;
            unity_dist = 2000;
            bp = 270;
            gain = 100;
            ball_size = size(ball);
            
            for i = 1:ball_size(1,1)
                d = obj.find_distance(ardrone, ball(i,:))- ball_radius;
                if (d < unity_dist)
                   % fp = fp + rep_const * (1/unity_dist - 1/d)/(d^2)*(ball(i,:)- ardrone); 
                   fp = fp + (1/(d-bp))*((ball(i,:)- ardrone)/d*3);
                else
                    fp = fp; %if too far, no force acting on it
                end
            end        
        end
        
        function dist = find_distance(obj, p1, p2)
            
            d_x = p1(1) - p2(1);
            d_y = p1(2) - p2(2);
            d_z = p1(3) - p2(3);
            
            dist = sqrt(d_x^2 + d_y^2 +d_z^2);
        end
        
%{        
        function drone_disp = forceOnhead(obj, drone_pos, force)
            
            drone_disp = zeros(1,6);
            h = 1.4;
            l = 0.7;

            yaw = drone_pos( 4 );
            pitch = drone_pos( 5 );
            roll = drone_pos( 6 );


            %equation in hw 2 theta head
            jacobian = [ 1, 0, 0;
                    0, 1, 0;
                    0, 0, 1;
                    -h*sin(yaw)*cos(pitch), h*cos(yaw)*cos(pitch), 0;
                    -h*cos(yaw)*sin(pitch), -h*sin(yaw)*cos(pitch), -h*cos(pitch);
                    0, 0, 0];

            drone_disp = (jacobian * force')';
        end
        
        function drone_disp = forceOnleft(obj, drone_pos, force)
            drone_disp = zeros( 1, 6 );

            h = 1.4;
            l = 0.7;

            yaw = drone_pos( 4 );
            pitch = drone_pos( 5 );
            roll = drone_pos( 6 );


% equation in hw2 theta left
            jacobian = [ 1, 0, 0;
                      0, 1, 0;
                      0, 0, 1;
                      -l*sin(yaw)*sin(pitch)*sin(roll) - l * cos(yaw) * cos(roll), l*cos(yaw)*sin(pitch)*sin(roll)-l*cos(yaw)*cos(roll), 0;
                      l*cos(yaw)*cos(pitch)*sin(roll), l*sin(yaw)*cos(pitch)*sin(roll), -l*sin(pitch)*sin(roll);
                     l*cos(yaw)*sin(pitch)*cos(roll) + l*sin(yaw)*sin(pitch), l*sin(yaw)*sin(pitch)*cos(roll) - l*cos(yaw)*sin(roll), l*cos(pitch)*cos(roll)];

            drone_disp = (jacobian * force')';
            
        end
    
        function drone_disp = forceOnright(obj, drone_pos, force)
            drone_disp = zeros(1,6);
            h = 1.4; % drone's dimension constant
            l = 0.7; % drone's dimension constant
            yaw = drone_pos(4); 
            pitch = drone_pos(5);
            roll = drone_pos(6);
            
            jacobian = [ 1, 0, 0;
                         0, 1, 0;
                         0, 0, 1;
         l*sin(yaw)*sin(pitch)*sin(roll) + l * cos(yaw) * cos(roll), -l*cos(yaw)*sin(pitch)*sin(roll)+l*cos(yaw)*cos(roll), 0;
         -l*cos(yaw)*cos(pitch)*sin(roll), -l*sin(yaw)*cos(pitch)*sin(roll), l*sin(pitch)*sin(roll);
         -l*cos(yaw)*sin(pitch)*cos(roll) - l*sin(yaw)*sin(pitch), -l*sin(yaw)*sin(pitch)*cos(roll) + l*cos(yaw)*sin(roll), -l*cos(pitch)*cos(roll)];
            
            drone_disp = (jacobian * force')';
        end
        
        function dist = dist_drone(obj, drone_pos, goal)
            
            mapped_drone = obj.map_drone(drone_pos);
            mapped_goal = obj.map_drone(goal);
            
            dist_head = obj.find_distance(mapped_drone(1,:), mapped_goal(1,:));
            dist_left = obj.find_distance(mapped_drone(2,:), mapped_goal(2,:));
            dist_right = obj.find_distance(mapped_drone(3,:), mapped_goal(3,:));
            
            dist = (dist_head + dist_left + dist_right)/3;
            
        end
%}        
        function update_dronePos(obj)
            %can probably take out the attraction force
            [drone_pos, ball_pos] = TrackFrameBBOX(obj.kinect);
            obj.counter = obj.counter + 1;
            
            
         
            
            if (isempty(drone_pos) || (1 && all(drone_pos == 0)))
                
                disp('drone not found');
                
            else
                obj.ardrone = [drone_pos',0,0,0];
                %obj.ardrone
                disp('drone detected');
                %obj.counter = obj.counter + 1;
                %obj.drone_path(obj.counter,:) = obj.ardrone(1:3);
                
                if (isempty(ball_pos) || (1 && any(ball_pos == 0)))
                     disp('balls not detected');
                    % obj.ardrone_dummy = obj.ardrone;
                else
                    disp('balls  detected');
                    obj.ball = ball_pos';
                    %obj.ball
                    obj.b_counter = obj.b_counter +1;
                    obj.ball_path(obj.b_counter,:) = obj.ball(1,:);
                    obj.ball_radius = 80;
                    %get kinect ball position and whether there is a ball or not
                    %mapped_goal = obj.map_drone(obj.goal); 
                    %mapped_drone = obj.map_drone(obj.ardrone_dummy); %get kinect drone position
                    f_rep = obj.force_rep(drone_pos', obj.ball, obj.ball_radius) % calculate the forces acted on each control points
                    f_rep = f_rep*2000;
                    %f(2, :) = obj.force_rep(mapped_drone(2,:), obj.ball, obj.ball_radius);
                    %f(3, :) = obj.force_rep(mapped_drone(3,:), obj.ball, obj.ball_radius);        
                    %head_disp = obj.forceOnhead( obj.ardrone, f(1, :) ); % calculate the change in position for each of the control points affected by the force
                    %obj.ardrone_dummy = obj.ardrone_dummy + f_rep;   
                    %obj.ardrone_dummy = obj.ardrone;
                    new_cord = obj.ardrone(1:3)+f_rep;
                     if (f_rep(1) ~= 0)
                        %obj.move_xyz(new_cord(1), new_cord(2), new_cord(3));
                         obj.droneObject.xyzyaw(f_rep(1),f_rep(2),0,0);
                        % obj.droneObject.xyzyaw(-f_rep(1),-f_rep(2),0,0);
                        % obj.droneMoved = true;
                     end
              
                
                end
                %obj.eva_path(obj.counter,:) = obj.ardrone_dummy(1:3);
                
            end

            

        end
            
        
        function calibrate_kinect(obj)
            
            obj.kinect.Calibrate();
        end
        
        function run(obj)
            obj.counter = 0;
            obj.b_counter = 0;
            obj.drone_path = [];
            obj.ball_path = [];
            obj.eva_path = [];
            obj.ball_radius = 0;
            obj.droneObject.ftrim();
            pause(1);
            obj.droneObject.takeoff();
            pause(5);
            obj.move_z(1500);
            while (obj.droneMoved == false)
                tic;
                obj.update_dronePos;   %get kinnect position
                toc;
               % obj.map_drone(obj.ardrone);   %main algorithm
               if (obj.counter > 200)
                   break;
               end

            end
            pause(3);
            obj.droneObject.land();
           % pause(5);

            %obj.droneObject.land();
            obj.droneMoved = false;
           % plot3( obj.drone_path(:,1), obj.drone_path(:,2), obj.drone_path(:,3) );
            %plot3( obj.eva_path(:,1), obj.eva_path(:,2), obj.eva_path(:,3) );
        end
        
        function reset(obj)
            obj.time = 1;
            obj.ardrone = [0,0,10,0,0,0];
            obj.goal = [70, 12, 10, 0, 0, 0];
            obj.ball = [35.8, 6.7, 0;
                        50.0, 30.0, 10];                        
            obj.ball_radius = 1;
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
            obj.droneObject.get_altitude;
            [drone_pos, ball_pos] = TrackFrame(obj.kinect);
            
            %Check for 0 in sensor data
             if (isempty(drone_pos) || (1 && any(drone_pos == 0)))    
                 disp('drone not found');
             else
                 %Get x y z coordinate
                 x_pos = drone_pos(1);
                 y_pos = drone_pos(3);
                 z_pos = obj.droneObject.altitude;           

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
             
             

             obj.droneObject.xyzyaw(roll_setting,pitch_setting,z_setting,0)
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
                    obj.droneObject.xyzyaw(roll_setting,0,0,0)
                    
                    
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
                    obj.droneObject.xyzyaw(0,pitch_setting,0,0)
                
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
                obj.droneObject.get_altitude;
                z_pos = obj.droneObject.altitude;
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
                
                obj.droneObject.xyzyaw(0,0,z_setting,0)
            end
        end	
        
    end
    
end

