function [x_T, P_T] = cartesian2turn(x_C, P_C)  %#codegen
% Transform [dx,vx,ax,dy,vy,ay]' to [dx,dy,h,omega,s, 0]'
% see Blackman, Popoli, "Modern Tracking Systems", S. 229

dx = x_C(1);
dy = x_C(4);
vx = x_C(2);
vy = x_C(5);
ax = x_C(3);
ay = x_C(6);

h = atan2(vy,vx);
s = sqrt(vx^2 + vy^2);

if s > 1
    
    omega = (vx*ay - vy*ax)/(s^2);

    a32 = -vy/(s^2);
    a35 = vx/(s^2);
    a42 = (ay*(vy^2 - vx^2) + 2*vx*vy*ax)/(s^4);
    a45 = (ax*(vy^2 - vx^2) - 2*vx*vy*ay)/(s^4);
    a43 = a32;
    a46 = a35;
    a52 = vx/s;
    a55 = vy/s;
    
    A_T = [1    0   0   0   0   0;
           0    0   0   1   0   0;
           0    a32 0   0   a35 0;
           0    a42 a43 0   a45 a46;
           0    a52 0   0   a55 0;
           0    0   0   0   0   0];

    x_T = [dx dy h omega s 0]';
    P_T = A_T*P_C*A_T';
    
else

    omega = 0;
    
    x_T = [dx dy h omega s 0]';
    P_T = [P_C(1,1) 0           0           0   0                     0;
           0        P_C(4,4)    0           0   0                     0;
           0        0           (pi/2)^2    0   0                     0;
           0        0           0           1   0                     0;
           0        0           0           0   max(P_C(2,2),P_C(5,5))   0;
           0        0           0           0   0                     0];
    
end


end