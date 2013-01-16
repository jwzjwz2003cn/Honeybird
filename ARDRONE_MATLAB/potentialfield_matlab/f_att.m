function force = f_att( work, goal )

force = zeros(1, 3 );

d_goal = 1.;
c_att = 0.1;

dist = dist_point(work, goal );
if ( dist < d_goal )
    force = c_att * ( goal - work ) ;
else
    force = d_goal * c_att * ( goal - work ) / dist ;
end
