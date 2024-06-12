function [barycenter, surface_size, normal] = surface_props(corners)
%SURFACE_PROPS Computes surface properties from input of surface corners
%INPUTS:
%   corners: Nx3 matrix of corner coordinates
%OUTPUTS
%   barycenter: barycenter of the surface
%   size: size of the surface (units^2)
%   normal: normal (unit) vector to the surface
%% 1. Compute the plane the corners lie in
[normal,V,~,max_dist] = best_fit_plane(corners);
% If the max_dist is greater than 0.005 (assuming [m] units -> 5mm)
% Give a warning
if max_dist > 0.005
    fprintf("WARNING: corners may not be coplanar (max dist %f)",max_dist);
end

%% 2. Get the 2D coordinates of the points in the plane
% By taking dot-products with the vectors defining the plane (V)
corners_2D = zeros(size(corners,1),2);
for i = 1:size(corners,1)
    corners_2D(i,1) = dot(V(:,1),corners(i,:));
    corners_2D(i,2) = dot(V(:,2),corners(i,:));
end

%% Create a polyshape from the 2D corners
pshape = polyshape(corners_2D(:,1),corners_2D(:,2));

%% Compute the barycenter
% Get the centroid in the projected coordinates
[centroid_x, centroid_y] = centroid(pshape);
% Project back to 3D
barycenter = centroid_x.*V(:,1) + centroid_y.*V(:,2);

%% Compute the surface size
surface_size = area(pshape);
end

