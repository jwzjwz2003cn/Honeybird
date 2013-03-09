classdef ffield < handle;

    properties
        ardrone  %turn it into a drone object
        goal
        ball
        ball_radius
        count
        time 
        droneObject
        kinect
        droneMoved;
        xmlPath
    end
    
    methods
        
        function obj = ffield()
            obj.time = 1;
            obj.ardrone = [0,0,10,0,0,0];
            obj.goal = [70, 12, 10, 0, 0, 0];
            obj.ball = [35.8, 6.7, 0;
                        50.0, 30.0, 10];
                         

            obj.ball_radius = 0;
            
            obj.droneObject = drone;
            obj.droneMoved = false;
            obj.xmlPath = '/Users/Frank/Honeybird/ARDRONE_MATLAB/SamplesConfig.xml';
            obj.kinect = SensorClassBBOX(obj.xmlPath');
            
        end
        
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
        
        function fp = force_rep(obj, ardrone, ball, ball_radius)
            fp = zeros(1,3);
            rep_const = 0.4;
            unity_dist = 7;
            ball_size = size(ball);
            
            for i = 1:ball_size(1,1)
                d = obj.find_distance(ardrone, ball(i,:))- ball_radius;
                if (d < unity_dist)
                    fp = fp + rep_const * (1/unity_dist - 1/d)/(d^2)*(ball(i,:)- ardrone); 
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
        
        function update_dronePos(obj)
            %can probably take out the attraction force
            mapped_drone = obj.map_drone(obj.ardrone); %get kinect drone position
            %get kinect ball position and whether there is a ball or not
            mapped_goal = obj.map_drone(obj.goal); 
            f(1, :) = obj.force_rep(mapped_drone(1,:), obj.ball, obj.ball_radius); % calculate the forces acted on each control points
            f(2, :) = obj.force_rep(mapped_drone(2,:), obj.ball, obj.ball_radius);
            f(3, :) = obj.force_rep(mapped_drone(3,:), obj.ball, obj.ball_radius);        
            head_disp = obj.forceOnhead( obj.ardrone, f(1, :) ); % calculate the change in position for each of the control points affected by the force
            left_disp = obj.forceOnleft( obj.ardrone, f(2, :) );
            right_disp = obj.forceOnright( obj.ardrone, f(3, :) );
            avg_disp = (head_disp+left_disp+right_disp)/3;
            obj.ardrone = obj.ardrone + avg_disp;   
            coordinate = obj.ardrone(1:3)   %new position
            
            if (obj.ball_radius ~= 0)
                obj.droneObject.move_forward(1);
                obj.droneObject.move_forward(1); 
                obj.droneMoved = true;
            end
        end
            
        
        function calibrate_kinect(obj)
            
            obj.kinect.calibrate();
        end
        
        function run(obj)
            
            
            obj.droneObject.takeoff();
            while (obj.droneMoved == false)
                obj.update_dronePos;   %get kinnect position
               % obj.map_drone(obj.ardrone);   %main algorithm

            end
            obj.droneObject.land();
        end
        
        function reset(obj)
            obj.time = 1;
            obj.ardrone = [0,0,10,0,0,0];
            obj.goal = [70, 12, 10, 0, 0, 0];
            obj.ball = [35.8, 6.7, 0;
                        50.0, 30.0, 10];                        
            obj.ball_radius = 1;
        end
        
    end
    
end

