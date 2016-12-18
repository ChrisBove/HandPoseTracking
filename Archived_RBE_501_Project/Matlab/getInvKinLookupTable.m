function IKTable = getInvKinLookupTable(l1, l2, l3, q4_q3ratio)
% create 3D grid of possible joint angles
[q1, q2, q3] = meshgrid(-30:1:30, -30:1:90, -2:1:90);

% determine q4 based on hand relationships
q4 = q4_q3ratio*q3;

% compute the x values corresponding to the joint angles
x = -sind(q1).*(l2*cosd(q2 + q3) + l1.*cosd(q2) + l3.*cosd(q2 + q3 + q4));

% compute the y values corresponding to the joint angles
y = l2.*sind(q2 + q3) + l1.*sind(q2) + l3.*sind(q2 + q3 + q4);

% compute the z values corresponding to the joint angles
z = cosd(q1).*(l2.*cosd(q2 + q3) + l1.*cosd(q2) + l3.*cosd(q2 + q3 + q4));

% create table of x y z values and their corresponding joint angles
IKTable = [x(:),y(:),z(:),q1(:),q2(:),q3(:),q4(:)];