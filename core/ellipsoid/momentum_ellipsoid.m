function ellipsoid = momentum_ellipsoid(I,L)
% Gets momentum ellipsoid parameters given omega, I, and L (principal axes)
%INPUTS:
%   I: 3x3 inertia tensor
%   L: Angular momentum (principal axes)
%OUTPUTS:
%   center: Momentum ellipsoid center in principal axes [xc yc zc]
%   semi_axis_lengths: Semi axis lengths [xr yr zr]
arguments
    I (3,3) double
    L (1,3) double
end

%% Center
xc = 0;
yc = 0;
zc = 0;
center = [xc yc zc];

%% Semi Axes
a = norm(L) / I(1,1);
b = norm(L) / I(2,2);
c = norm(L) / I(3,3);

semi_axis_lengths = [a b c];

ellipsoid = Ellipsoid(center,semi_axis_lengths);
end

