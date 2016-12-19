function [result] = quaternionToEuler(row_vector) % ZYX Euler angles
w = row_vector(1,1);
x = row_vector(1,2);
y = row_vector(1,3);
z = row_vector(1,4);

 sqw = w*w;
 sqx = x*x;
 sqy = y*y;
 sqz = z*z;

roll = atan2(2.0*(x*y+z*w),(sqx-sqy-sqz+sqw));
pitch = -asin(2.0*(x*z-y*w)/(sqx+sqy+sqz+sqw));
yaw = atan2(2.0*(y*z+x*w),(-sqx-sqy+sqz+sqw));

result = [roll, pitch,yaw];
end
