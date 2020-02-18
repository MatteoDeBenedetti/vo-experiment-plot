function [hdg] = quaternion2heading(q)
%UNTITLED Summary of this function goes here
%   INPUT: q: quaternion expressed as wxyz (aka: abcd or re,im)

% if an invalid quaternion is passed
if (numel(q) < 4)
    disp("quaternion2heading(q) usage: must pass a 4 elements vector or n-by-4 array ")

% if a single quaternion is given
elseif (numel(q) == 4)
    w=q(1); x=q(2); y=q(3); z=q(4); 
    hdg = asin(2*x*y + 2*z*w);

% if an array of quaternions is passed
else 
    w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4); 
    hdg = asin(2*x.*y + 2*z.*w);
    
end

end

