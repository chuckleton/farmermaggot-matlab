function ellipsoid = energy_ellipsoid(I,T)
% Gets energy ellipsoid parameters given omega, I, and T (principal axes)
%INPUTS:
%   I: 3x3 intertia tensor
%   T: Rotational Kinetic Energy (principal axes)
%OUTPUTS:
%   center: Momentum ellipsoid center in principal axes [xc yc zc]
%   semi_axis_lengths: Semi axis lengths [xr yr zr]
arguments
    I (3,3) double
    T (1,1) double
end

%% Center
xc = 0;
yc = 0;
zc = 0;
center = [xc yc zc];

%% Semi Axes
a = sqrt(2*T / I(1,1));
b = sqrt(2*T / I(2,2));
c = sqrt(2*T / I(3,3));

semi_axis_lengths = [a b c];

ellipsoid = Ellipsoid(center,semi_axis_lengths);
end
