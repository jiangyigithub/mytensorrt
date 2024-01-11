function [x_C, P_C] = turn2cartesian(x_T, P_T, vx_ego) %#codegen
% Transformation von [dx,dy,h,omega,s, 0]' zu [dx,vx,ax,dy,vy,ay]'
% siehe Blackman, Popoli, "Modern Tracking Systems", S. 229

dx = x_T(1);
dy = x_T(2);
h = x_T(3);
omega = x_T(4);
s = x_T(5);

vx = s*cos(h);
vy = s*sin(h);
ax = -omega*vy;
ay = omega*vx;


A_C = [1  0   0   0   0             0;
       0  0   -vy 0   cos(h)        0;
       0  0   -ay -vy -omega*sin(h) 0;
       0  1   0   0   0             0;
       0  0   vx  0   sin(h)        0;
       0  0   ax  vx  omega*cos(h)  0];
 
x_C = [dx vx ax dy vy ay]';

% Für Varianzen muss die Relativgeschwindigkeit verwendet werden,
% da die Varianzen bei schnellen Objekten sonst riesig werden können (Unsicherheit im Heading schlägt durch?)
% Diese Lösung ist numerisch instabil :-(
% A_C(5,3) = A_C(5,3) - vx_ego;
% A_C(6,4) = A_C(6,4) - vx_ego;

P_C = A_C*P_T*A_C';

% % Bei Initialisierung können Varianzen der Beschleunigung 0 werden
% P_C(3,3) = max(P_C(3,3),1);
% P_C(6,6) = max(P_C(6,6),1);

end