function [surfaces] = get_rect_prism_surfaces(center,l,w,h)
% Gets the surfaces for a rectangular prism from the center and dimensions
% (prism must be aligned with axes)
%INPUTS:
%   center: the center of the prism (3-vector)
%   l: the y-dimension of the prism
%   w: the x-dimension of the prism
%   h: the z-dimension of the prism
%OUTPUTS:
%   surfaces: A 6x4x3 matrix of the surfaces (6 surfaces, 4 points in each)
%% 1. Get the 8 corners of the prism
offsets = zeros(8,3);
offsets(1,:) = [w l h];
offsets(2,:) = [-w l h];
offsets(3,:) = [w -l h];
offsets(4,:) = [w l -h];
offsets(5,:) = [-w -l h];
offsets(6,:) = [w -l -h];
offsets(7,:) = [-w l -h];
offsets(8,:) = [-w -l -h];

corners = center + offsets;

%% Get the 6 surfaces of the prism
surfaces = zeros(6,4,3);

surfaces(1,:,:) = [corners(2,:);corners(1,:);corners(3,:);corners(5,:)]; % +h
surfaces(2,:,:) = [corners(6,:);corners(4,:);corners(7,:);corners(8,:)]; % -h
surfaces(3,:,:) = [corners(1,:);corners(4,:);corners(6,:);corners(3,:)]; % +w
surfaces(4,:,:) = [corners(5,:);corners(2,:);corners(7,:);corners(8,:)]; % -w
surfaces(5,:,:) = [corners(2,:);corners(1,:);corners(4,:);corners(7,:)]; % +l
surfaces(6,:,:) = [corners(5,:);corners(3,:);corners(6,:);corners(8,:)]; % -l
end

