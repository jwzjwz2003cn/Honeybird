function d = trans_conf_work( conf )

h = 1.4;
l = 0.7;

x = conf(1);
y = conf(2);
z = conf(3);
a = conf(4);
b = conf(5);
r = conf(6);

d = zeros( 3, 3 );
% head
d( 1, 1 ) = x + h * cos( a ) * cos( b );
d( 1, 2 ) = y + h * sin( a ) * cos( b );
d( 1, 3 ) = z - h * sin( b );

%left
d( 2, 1 ) = x + l * cos( a ) * sin( b ) * sin( r ) - l * sin( a ) * cos( r );
d( 2, 2 ) = y + l * sin( a ) * sin( b ) * sin( r ) + l * cos( a ) * cos( r );
d( 2, 3 ) = z + l * cos( b ) * sin( r );

%right
d( 3, 1 ) = x - l * cos( a ) * sin( b ) * sin( r ) + l * sin( a ) * cos( r );
d( 3, 2 ) = y - l * sin( a ) * sin( b ) * sin( r ) - l * cos( a ) * cos( r );
d( 3, 3 ) = z - l * cos( b ) * sin( r );
