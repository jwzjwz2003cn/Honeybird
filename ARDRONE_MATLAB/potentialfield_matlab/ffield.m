classdef ffield < handle;

    properties
        drone
        goal
        ball
        ball_radius
        count
        time 
    end
    
    methods
        
        function obj = ffield()
            obj.time = 1;
            obj.drone = [0,0,10,0,0,0];
            obj.goal = [70, 12, 10, 0, 0, 0];
            obj.ball = [35.8, 6.7, 0;
                        50.0, 30.0, 10];
                         

            obj.ball_radius = 1;
            
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
        
        function fa = force_attr(obj, drone, goal)
            
            fa = zeros(1,3);                      
            att_const = 0.1;
            unity_dist = 1;
            
            d = obj.find_distance(drone, goal);
            if (d < unity_dist)
                fa = att_const * (goal - drone);
            else
                fa = unity_dist * att_const * (goal-drone)/d; %increase the field when close
            end
            
        end
        
        function fp = force_rep(obj, drone, ball, ball_radius)
            fp = zeros(1,3);
            rep_const = 0.4;
            unity_dist = 7;
            ball_size = size(ball);
            
            for i = 1:ball_size(1,1)
                d = obj.find_distance(drone, ball(i,:))- ball_radius;
                if (d < unity_dist)
                    fp = fp + rep_const * (1/unity_dist - 1/d)/(d^2)*(ball(i,:)- drone); 
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
            
            mapped_drone = obj.map_drone(obj.drone);
            mapped_goal = obj.map_drone(obj.goal);
            f(1, :) = obj.force_attr(mapped_drone(1,:), mapped_goal(1,:)) + obj.force_rep(mapped_drone(1,:), obj.ball, obj.ball_radius); % calculate the forces acted on each control points
            f(2, :) = obj.force_attr(mapped_drone(2,:), mapped_goal(2,:)) + obj.force_rep(mapped_drone(2,:), obj.ball, obj.ball_radius);
            f(3, :) = obj.force_attr(mapped_drone(3,:), mapped_goal(3,:)) + obj.force_rep(mapped_drone(3,:), obj.ball, obj.ball_radius);        
            head_disp = obj.forceOnhead( obj.drone, f(1, :) ); % calculate the change in position for each of the control points affected by the force
            left_disp = obj.forceOnleft( obj.drone, f(2, :) );
            right_disp = obj.forceOnright( obj.drone, f(3, :) );
            avg_disp = (head_disp+left_disp+right_disp)/3;
            obj.drone = obj.drone + avg_disp;   
            coordinate = obj.drone(1:3)
                        
        end
            
        function run(obj)
            obj.time = 1;
            fig = figure; 
            set(fig, 'DoubleBuffer','On')
            t = [];
            ball_tr1 = [];
            ball_tr2 = [];
            
            while (obj.dist_drone(obj.drone, obj.goal) > 0.15)
                obj.update_dronePos; 
                mapped_drone = obj.map_drone(obj.drone);
                
                t(obj.time,:) = obj.drone(1:3);    
                plot(obj.drone(1:3), obj.time);
                hold on
                t_h(obj.time,:) = mapped_drone(1,:);
                t_l(obj.time,:) = mapped_drone(2,:);
                t_r(obj.time,:) = mapped_drone(3,:);
                ball_tr1(obj.time,:) = obj.ball(1,:);
                ball_tr2(obj.time,:) = obj.ball(2,:);
                
                
                if obj.time > 900
                    break;
                end
                obj.time = obj.time+1;
                
% plot the trajactories                
            size_ball  = size(obj.ball);
            for i = 1:size_ball(1)
                hold off
                %hold on;
                row = obj.ball(i,:);
                [x,y,z] = sphere();
                surf(x*obj.ball_radius+row(1), y*obj.ball_radius+row(2), z*obj.ball_radius+row(3));
                axis ([0 70 -3 30 0 20]);
                drawnow expose;
                
            end
            
            plot3( ball_tr1(:,1), ball_tr1(:,2), ball_tr1(:,3));
            hold on
            plot3( ball_tr2(:,1), ball_tr2(:,2), ball_tr2(:,3));
            plot3( t(:,1), t(:,2), t(:,3) );
            axis ([0 70 -3 30 0 20]);
            drawnow expose;
                obj.ball(1,3) = obj.ball(1,3) + 0.03;    %controlling ball's velocity
                obj.ball(2,2) = obj.ball(2,2) - 0.045;
                
            end
            
            for i = 1:size_ball(1)
                hold on;
                row = obj.ball(i,:);
                [x,y,z] = sphere();
                surf(x*obj.ball_radius+row(1), y*obj.ball_radius+row(2), z*obj.ball_radius+row(3));
                axis ([0 70 -3 30 0 20]);
                drawnow expose;
                
            end
                        

            xlabel('x');
            ylabel('y');
            zlabel('z');

        end
        
        function reset(obj)
            obj.time = 1;
            obj.drone = [0,0,10,0,0,0];
            obj.goal = [70, 12, 10, 0, 0, 0];
            obj.ball = [35.8, 6.7, 0;
                        50.0, 30.0, 10];                        
            obj.ball_radius = 1;
        end
        
    end
    
end

