function plot_ellipsoid(center,semi_axes,facecolor,facealpha,label)
% Plots an ellipsoid on the current figure
%INPUTS:
%   center: [xc yc zc]
%   semi_axes: [rx ry rz]

[X,Y,Z] = ellipsoid(center(1), center(2), center(3), ...
    semi_axes(1), semi_axes(2), semi_axes(3));
surf(X,Y,Z,'FaceColor',facecolor,'FaceAlpha',facealpha,'EdgeColor','none', ...
    'DisplayName',label);
end

