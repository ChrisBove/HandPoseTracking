clc;
clear all;
close all;


imu_data = csvread('shaky_static_imu.csv');
load('wrkspace_mocap_shaky_static.mat','angle_tan');

imu_data=imu_data(:,1:4);
area = numel(imu_data);
total_rows = area/4;
numRows_eachDataset = (total_rows)/2;

imu_static = zeros((numRows_eachDataset),4);
imu_mov = zeros((numRows_eachDataset),4);

diff_angle_quat=zeros((numRows_eachDataset),4);
imu_mov_edited = zeros((numRows_eachDataset),4);
time = zeros(1,numRows_eachDataset);
static_euler = zeros(numRows_eachDataset,3);
mov_euler = zeros(numRows_eachDataset,3);
static_euler_default = zeros(numRows_eachDataset,3);
mov_euler_default = zeros(numRows_eachDataset,3);

k=1;
%% Store moving and static links in different matrices
for i=1:2:total_rows-1

imu_static(k,:) = imu_data(i,:);
imu_mov(k,:) = imu_data(i+1,:);
imu_mov_edited(k,:) = [imu_mov(k,1)+0.97629,imu_mov(k,2),imu_mov(k,3)+0.21643,imu_mov(k,4);];
k=k+1;
end


%% Convert each quaternion to Euler 
for i=1:numRows_eachDataset
static_euler(i,:) = quaternionToEuler(imu_static(i,:));
mov_euler(i,:) = quaternionToEuler(imu_mov(i,:));

time(1,i) = i*0.01;
end

%% Conert euler angle to deg
static_euler_deg = rad2deg(static_euler);
mov_euler_deg = rad2deg(mov_euler);

% Difference in angle between static and moving link
diff = ( mov_euler_deg-static_euler_deg);

% Absolute difference between Mocap diff and imu diff
error = abs((angle_tan)) - abs(diff(:,2));

%% Plot data


figure
hold on
plot(time,diff(:,2),'b');
plot(time,(angle_tan),'r');
hold off
ylabel('Rotation angle(deg)');
xlabel('Time(s)');
title('IMU and MoCap data');

figure
plot(time,(error),'b');
ylabel('Rotation angle(deg)');
xlabel('Time(s)');
title('Drift');

