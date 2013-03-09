clear;
init = [0, 0, 1.0, 0, 0, 0];     % drone's initial state
obs = [5, 0, 0.2;                % set up obstacles
    5, 0, 1.8;
    5, 1.6, 1.0;
    5, -1.6, 1.0];
radius = 0.6;                    % set up the radius of the obstacles

work = trans_conf_work( init ); % transfer the initial configuration into head, left and rights control points in 3x3 matrix
goal = [7, 1.2, 1, 0, 0, 0];    % set a goal state
goal_work = trans_conf_work( goal );   % transfer the goal state into head, left and right control points in 3x3 matrice

x = init;
figure(1);  %create a figure

count = 0;


while( dist_conf( x, goal) > 0.15 ) % while each of the 3 control points is more than 0.15 unit from the goal
    count = count + 1;
    log(count, :) = x;
    x_work = trans_conf_work( x );
    f(1, :) = f_att(x_work(1,:), goal_work(1,:)) + f_rep(x_work(1,:), obs, radius); % calculate the forces acted on each control points
    f(2, :) = f_att(x_work(2,:), goal_work(2,:)) + f_rep(x_work(2,:), obs, radius);
    f(3, :) = f_att(x_work(3,:), goal_work(3,:)) + f_rep(x_work(3,:), obs, radius);        

    u1 = u_head( x, f(1, :) ) / 3; % calculate the change in position for each of the control points affected by the force
    u2 = u_left( x, f(2, :) ) / 3;
    u3 = u_right( x, f(3, :) ) / 3;
    x = x + u1 + u2 + u3
    
    x_work = trans_conf_work( x );  %
    plot3( x_work(:,1), x_work(:,2), x_work(:,3) );
    t(count,:) =  x_work(1,:);  %plot drone's head only
    dist_conf( x, goal );
    
    if count > 900
        break;
    end

%    pause(1)
end;

size_obs  = size(obs);
for i = 1:size_obs(1)
    hold on;
    row = obs(i,:)
    [x,y,z] = sphere();
    surf(x*radius+row(1), y*radius+row(2), z*radius+row(3));
    
end
t
plot3( t(:,1), t(:,2), t(:,3) );
axis ([0 7 -3 3 0 2]);
