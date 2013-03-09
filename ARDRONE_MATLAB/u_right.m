function u = u_right( x, f )
u = zeros( 1, 6 );

h = 1.4;
l = 0.7;

a = x( 4 );
b = x( 5 );
r = x( 6 );

%equation in hw 2 theta right
Trans = [ 1, 0, 0;
         0, 1, 0;
         0, 0, 1;
         l*sin(a)*sin(b)*sin(r) + l * cos(a) * cos(r), -l*cos(a)*sin(b)*sin(r)+l*cos(a)*cos(r), 0;
         -l*cos(a)*cos(b)*sin(r), -l*sin(a)*cos(b)*sin(r), l*sin(b)*sin(r);
         -l*cos(a)*sin(b)*cos(r) - l*sin(a)*sin(r), -l*sin(a)*sin(b)*cos(r) + l*cos(a)*sin(r), -l*cos(b)*cos(r)];

u = (Trans * f')';
     
     