function u = u_head( x, f )
u = zeros( 1, 6 );

h = 1.4;
l = 0.7;

a = x( 4 );
b = x( 5 );
r = x( 6 );


%equation in hw 2 theta head
Trans = [ 1, 0, 0;
         0, 1, 0;
         0, 0, 1;
         -h*sin(a)*cos(b), h*cos(a)*cos(b), 0;
         -h*cos(a)*sin(b), -h*sin(a)*cos(b), -h*cos(b);
         0, 0, 0];
f
u = (Trans * f')';
     
     