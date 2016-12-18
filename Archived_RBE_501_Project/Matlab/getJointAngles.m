function [theta1, theta2, theta3, theta4] = getJointAngles(xtip, ytip, ztip, IKtable)
% find length of IKtable
[length, ~] = size(IKtable);

% temporary variables for minimum distance
theta1 = IKtable(1,4);
theta2 = IKtable(1,5);
theta3 = IKtable(1,6);
theta4 = IKtable(1,7);

% compute an initial shortest distance from desired point to test point
bestDistance = (xtip - IKtable(1,1))^2 + (ytip - IKtable(1,2))^2 + (ztip - IKtable(1,3))^2;

% search the lookup table for an endpoint that is closer to the desired point
for i = 2:length  
    distance = (xtip - IKtable(i,1))^2 + (ytip - IKtable(i,2))^2 + (ztip - IKtable(i,3))^2;

    if distance < bestDistance
        theta1 = IKtable(i,4);
        theta2 = IKtable(i,5);
        theta3 = IKtable(i,6);
        theta4 = IKtable(i,7);
        bestDistance = distance;
    end
end