function [surface_barycenters,surface_sizes,surface_normals] = sc_surface_props()
%SC_SURFACE_PROPS Returns the surface barycenters, sizes, and normal
%vectors for the spacecraft
%% Central body surfaces
CB_center = [0 -0.157 0];
CB_l = 1.22;
CB_w = 0.76;
CB_h = 0.92;
CB_surfaces = get_rect_prism_surfaces(CB_center,CB_l,CB_w,CB_h);

%% Solar panel surfaces
SP1_center = [-1.081 0 0];
SP_l = 0.6;
SP_w = 1.2;
SP_h = 0.0;
SP_surfaces = get_rect_prism_surfaces(SP1_center,SP_l,SP_w,SP_h);
% We can ignore elements 3+ in the SP surfaces since it is 0-height
SP1_surfaces = SP_surfaces(1:2,:,:);

SP2_center = [1.081 0 0];
SP_surfaces = get_rect_prism_surfaces(SP2_center,SP_l,SP_w,SP_h);
% We can ignore elements 3+ in the SP surfaces since it is 0-height
SP2_surfaces = SP_surfaces(1:2,:,:);

%% Create a combined surfaces matrix
surfaces = [CB_surfaces;SP1_surfaces;SP2_surfaces];

%% Get the surface properties
surface_barycenters = zeros(size(surfaces,1),3);
surface_sizes = zeros(size(surfaces,1),1);
surface_normals = zeros(size(surfaces,1),3);

for i=1:size(surfaces,1)
    [barycenter,surface_size,normal] = surface_props(squeeze(surfaces(i,:,:)));
    surface_barycenters(i,:) = barycenter;
    surface_sizes(i,:) = surface_size;
    surface_normals(i,:) = normal;
end

end

