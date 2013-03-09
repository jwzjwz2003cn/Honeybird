function dist = dist_conf( x, goal )

x_work = trans_conf_work( x );
goal_work = trans_conf_work( goal );

d_h = dist_point( x_work(1, :), goal_work(1, : ) );
d_l = dist_point( x_work(2, :), goal_work(2, : ) );
d_r = dist_point( x_work(3, :), goal_work(3, : ) );

dist = ( d_h + d_l + d_r ) / 3;

