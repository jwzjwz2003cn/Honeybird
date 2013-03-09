function dist = dist_point( p1, p2 )

dx = p1( 1 ) - p2( 1 );
dy = p1( 2 ) - p2( 2 );
dz = p1( 3 ) - p2( 3 );

dist = sqrt( dx ^ 2 + dy ^ 2 + dz ^ 2 );

