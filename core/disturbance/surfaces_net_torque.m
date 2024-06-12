function net_torque = surfaces_net_torque(surface_barycenters, forces)
% Gets inclinations of surfaces relative to vec
%INPUTS:
%   surface_barycenters: 3xn matrix of surface barycenters
%   forces: 3xn matrix of forces on each surface
%OUTPUTS:
%   net_torque: 3x1 vector of the net torque
torques = cross(surface_barycenters,forces,1);
net_torque = sum(torques,2);
end

