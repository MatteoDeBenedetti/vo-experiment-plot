function [roll] = quaternion2roll(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% if an invalid quaternion is passed
if (numel(q) < 4)
    disp("quaternion2roll(q) usage: must pass a 4 elements vector or n-by-4 array ")

% if a single quaternion is passed
elseif (numel(q) == 4)
    w=q(1); x=q(2); y=q(3); z=q(4); 
    sinr_cosp = +2.0*(w*x + y*z);
    cosr_cosp = +1.0 - 2.0*(x*x + y*y);
    roll = atan2(sinr_cosp, cosr_cosp);

% if an array of quaternions is passed
else 
    w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4); 
    sinr_cosp = +2.0 * (w.*x + y.*z);
    cosr_cosp = +1.0 - 2.0 * (x.*x + y.*y);
    roll = atan2(sinr_cosp, cosr_cosp);
    
end

end

