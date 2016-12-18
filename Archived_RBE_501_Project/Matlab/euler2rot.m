function R = euler2rot(vector)
z = deg2rad(vector(1));
y = deg2rad(vector(2));
x = deg2rad(vector(3));

% Stepwise R fomula
% Rx = [1 0 0;
%     0 cx -sx;
%     0 sx cx];
% 
% Ry = [cy 0 sy;
%     0 1 0;
%     -sy 0 cy];
% 
% Rz = [cz -sz 0;
%     sz cz 0;
%     0 0 1];
% 
% R = Rz*Ry*Rx;

% Solved Symbollic Rotation Matrix (optimized)
sx = sin(x);
sy = sin(y);
sz = sin(z);
cx = cos(x);
cy = cos(y);
cz = cos(z);

R = [cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy;...
    cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx;...
    -sy, cy*sx, cx*cy];

% Note in projectGUI that not all of the terms of the rotation matrix are
% needed.  Getting rid of these terms saves computation time.
% R = [0, cz*sx*sy - cx*sz, 0;...
%     0, cx*cz + sx*sy*sz, 0;...
%     -sy, 0, cx*cos(y)];