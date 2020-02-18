function [pitch] = quaternion2pitch(q)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% if an invalid quaternion is passed
if (numel(q) < 4)
    disp("quaternion2heading(q) usage: must pass a 4 elements vector or n-by-4 array ")

% if a single quaternion is given
elseif (numel(q) == 4)
    w=q(1); x=q(2); y=q(3); z=q(4);
    sinp = +2.0 * (w*y - z*x);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);
    end
    
% if an array of quaternions is passed
else
    w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
    sinp = +2.0 * (w.*y - z.*x);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);
    end
    
end

end

