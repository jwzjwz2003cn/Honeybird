function force = f_rep( work, obs, radius )

force = zeros(1, 3 );

Q = 1;
c_rel = 0.05;

force = 0;
obs_size = size( obs );

for i=1:obs_size(1, 1)
    dist = dist_point(work, obs(i,:) ) - radius;  
    if ( dist < Q )
        force = force + c_rel * ( 1 / Q - 1 / dist ) / ( dist ^ 2 ) * ( obs(i,:) - work ) ;
    else
        force = force + [0, 0, 0];
    end
end