function [x, y, z] = updatePlotFinger3D(a1, a2, a3, q1, q2, q3, q4)
%% Create intermediate transformations matrices
% dh2mat(theta, d, a, alpha, deg)

% % used to convert world frame to model frame
% orient1 = dh2mat(0, 0, 0, -90, 1);
% orient2 = dh2mat(-90, 0, 0, 0, 1);
% orient = orient1*orient2;
% 
% % transformations of linkages
% T1 = dh2mat(-q1, 0, 0, -90, 1);
% T2 = dh2mat(-q2, 0, a1, 0, 1);
% T3 = dh2mat(-q3, 0, a2, 0, 1);
% T4 = dh2mat(-q4, 0, a3, 0, 1);
% 
% %% Create combined transformation matrices
% p1 = orient*T1;
% p2 = p1*T2;
% p3 = p2*T3;
% p4 = p3*T4;
% 
% %% Create vector of points
% x = [0, p1(1,4), p2(1,4), p3(1,4), p4(1,4)];
% y = [0, p1(2,4), p2(2,4), p3(2,4), p4(2,4)];
% z = [0, p1(3,4), p2(3,4), p3(3,4), p4(3,4)];

%% Optimized
% creates vector of x y and z points to be plotted (see above for methods)
x = [ 0, 0, -a1*cosd(q2)*sind(q1), -sind(q1)*(a2*cosd(q2 + q3) + a1*cosd(q2)), -sind(q1)*(a2*cosd(q2 + q3) + a1*cosd(q2) + a3*cosd(q2 + q3 + q4))];

y = [ 0, 0, a1*sind(q2), a2*sind(q2 + q3) + a1*sind(q2), a2*sind(q2 + q3) + a1*sind(q2) + a3*sind(q2 + q3 + q4)];

z = [ 0, 0, a1*cosd(q1)*cosd(q2), cosd(q1)*(a2*cosd(q2 + q3) + a1*cosd(q2)), cosd(q1)*(a2*cosd(q2 + q3) + a1*cosd(q2) + a3*cosd(q2 + q3 + q4))];