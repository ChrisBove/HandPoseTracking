clc;
clear all;
close all;
syms x y z;
mocap = csvread('shaky_static_mocap.csv',7,0);

mov_link =( mocap(:,43:51));
static_link =(mocap(:,52:60));
numRows = numel(mov_link)/9;

% numRows = numRows/4;% take only aportion of dataset to anaylze

%% Pre allocate points

P1 = zeros(numRows,3);
P2 = zeros(numRows,3);
P3 = zeros(numRows,3);

S1 = zeros(numRows,3);
S2 = zeros(numRows,3);
S3 = zeros(numRows,3);

vec=[x y z];

plane_angle_tan=zeros(numRows,1);
angle_tan=zeros(numRows,1);
sign=zeros(numRows,1);

time = zeros(1,numRows);
%% Find angle between between two planes

for i=1:numRows

P1 = [static_link(i,1),static_link(i,2),static_link(i,3)];
P2 = [static_link(i,4),static_link(i,5),static_link(i,6)];
P3 = [static_link(i,7),static_link(i,8),static_link(i,9)];

S1 = [mov_link(i,1),mov_link(i,2),mov_link(i,3)];
S2 = [mov_link(i,4),mov_link(i,5),mov_link(i,6)];
S3 = [mov_link(i,7),mov_link(i,8),mov_link(i,9)];
% digits(3);

normal_mov = (cross(P1-P2,P1-P3));
normal_static = (cross(S1-S2,S1-S3));

planefunction_mov =vpa(dot(normal_mov,vec-S1));
planefunction_static=vpa(dot(normal_static,vec-P1));

c1x = coeffs(planefunction_mov,x);
c1y = coeffs(planefunction_mov,y);
c1z = coeffs(planefunction_mov,z);

c2x = (coeffs(planefunction_static,x));
c2y = (coeffs(planefunction_static,y));
c2z = (coeffs(planefunction_static,z));

n1 = [c1x(1,2), c1y(1,2),c1z(1,2)];
n2 = [c2x(1,2), c2y(1,2),c2z(1,2)];

sign = double(subs(planefunction_static,[x,y,z],[S2(1,1),S2(1,2),S2(1,3)]));
plane_angle_tan = rad2deg(atan2(norm(cross(n1,n2)),abs(dot(n1,n2))));
    
if sign<0
        angle_tan(i) = -(plane_angle_tan);
else
        angle_tan(i) = abs(plane_angle_tan);
end

 time(1,i) = i*0.01;
end

%% plot data

figure
hold on
plot(time,angle_tan,'r');
title('Difference in angle between plane of static link and movable link');
